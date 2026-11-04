/**
 * @file dap_offline.c
 * @brief 离线烧录 D4:板上烧录引擎实现
 *
 * 流程(RESET_PROGRAM 挂起目标 → 装载 loader → 擦/写/校 → RESET_RUN):
 *   1. 拿线锁(3s 超时:在线调试会话占着就报 busy,不无限等)
 *   2. swd_set_target_state_sw(RESET_PROGRAM):连 DP/AP、halt、
 *      VC_CORERESET + SYSRESETREQ,核心停在复位向量
 *   3. 读 DP IDCODE 记档(诊断用,不做家族硬断言——IDCODE 不是家族唯一凭证)
 *   4. 按镜像 family 查算法注册表,拷 blob 进目标 SRAM
 *   5. init(解锁) → 逐页 erase → 1KB 分页 program → 回读全片算 CRC 比对
 *   6. uninit(上锁) → RESET_RUN(硬复位 + 撤调试电源,目标脱手起飞)
 *   7. 还线锁,在线调试自动恢复
 */
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "components/DAP/config/DAP_config.h"
#include "components/DAP/include/debug_cm.h"
#include "components/DAP/include/swd_host.h"
#include "components/DAP/include/target_family.h"
#include "soc/gpio_struct.h"

#include "main/dap_cmd.h"
#include "main/dap_flash_algo.h"
#include "main/dap_offline.h"
#include "main/dap_store.h"

static const char *TAG = "dap.offline";

#define OFFLINE_TASK_STACK 6144     /* swd_read/write_memory 的缓冲与调用层不浅 */
#define OFFLINE_TASK_PRIO  5
#define WIRE_LOCK_TIMEOUT_MS 3000   /* 在线调试占线时的让位时限 */
#define FILE_CHUNK 1024

static SemaphoreHandle_t s_wire_lock;
static dap_off_status_t  s_st;      /* 单写入者(引擎任务),state 最后写 */
static uint8_t s_chunk[FILE_CHUNK];

void dap_offline_init(void) {
    if (!s_wire_lock) {
        s_wire_lock = xSemaphoreCreateMutex();
    }
}

SemaphoreHandle_t dap_offline_wire_lock(void) {
    return s_wire_lock;
}

void dap_offline_get_status(dap_off_status_t *out) {
    if (out) *out = s_st;
}

static void set_progress(uint8_t pct) {
    s_st.pct = pct;
}

/*
 * swd_host.c(DAPLink 血统)依赖 target_family.c 提供 nRESET 驱动,本仓没编
 * 译那份文件——以前 swd_host 是死代码,链接缺口的第一次暴露就是 D4。在这
 * 里补:开漏脚,PIN_nRESET 由 DAP_config.h 按目标板给出(S3=13)。
 *   assert:输出使能 + 拉低(复位有效)
 *   release:关输出,上拉把线抬回去(复位解除)
 * 直接写寄存器与 DAP_config 的裸寄存器风格保持一致(不混 gpio 驱动)。
 */
void swd_set_target_reset(uint8_t asserted) {
    if (asserted) {
        GPIO.out_w1tc = 1u << PIN_nRESET;
        GPIO.enable_w1ts = 1u << PIN_nRESET;
    } else {
        GPIO.enable_w1tc = 1u << PIN_nRESET;
    }
}

static void fail(int err, const char *msg) {
    strlcpy(s_st.msg, msg, sizeof(s_st.msg));
    s_st.err = err;
    s_st.state = DAP_OFF_FAIL;   /* state 最后写:读侧看到 FAIL 时字段已齐 */
    ESP_LOGW(TAG, "flash fail: %s (err=%d idcode=%08x)",
             msg, err, s_st.idcode);
}

/* ---------------- 引擎主体(引擎任务上下文) ---------------- */

static void offline_flash_task(void *arg) {
    int slot = (int)(intptr_t)arg;
    const dap_flash_algo_t *algo = NULL;
    bool wire_held = false;
    bool algo_inited = false;
    dap_store_meta_t m;

    s_st.state = DAP_OFF_RUNNING;
    s_st.slot = slot;
    s_st.pct = 0;
    s_st.idcode = 0;
    s_st.err = 0;
    s_st.msg[0] = '\0';

    if (!dap_store_get(slot, &m)) {
        fail(DAP_ERR_SLOT_EMPTY, "slot_empty");
        goto out;
    }
    algo = dap_flash_algo_find(m.family);
    if (!algo) {
        fail(DAP_ERR_FAMILY, "family_not_registered");
        goto out;
    }
    if (m.addr % algo->page_size != 0) {
        fail(DAP_ERR_BADARG, "addr_not_page_aligned");
        goto out;
    }
    if (m.size == 0 || (m.size & 1) || (m.addr & 1)) {
        fail(DAP_ERR_BADARG, "size_or_addr_must_be_halfword");
        goto out;
    }

    /* 1. 线锁:在线调试进行中就明确失败,别吊死 GUI */
    if (xSemaphoreTake(s_wire_lock, pdMS_TO_TICKS(WIRE_LOCK_TIMEOUT_MS)) != pdTRUE) {
        fail(DAP_ERR_BUSY, "debug_session_active");
        goto out;
    }
    wire_held = true;

    /* 2. 连接 + 挂到复位向量 */
    if (!swd_set_target_state_sw(RESET_PROGRAM)) {
        fail(DAP_ERR_TARGET, "target_connect_fail");
        goto out;
    }
    set_progress(3);

    /* 3. IDCODE 记档 */
    uint32_t idcode = 0;
    swd_read_dp(DP_IDCODE, &idcode);
    s_st.idcode = idcode;
    ESP_LOGI(TAG, "target idcode=%08x family=%s slot=%d size=%u",
             idcode, m.family, slot, (unsigned)m.size);

    /* 4. 拷 loader 进目标 SRAM */
    if (!swd_write_memory(algo->prog.algo_start,
                          (const uint8_t *)algo->prog.algo_blob,
                          algo->prog.algo_size)) {
        fail(DAP_ERR_TARGET, "load_algo_fail");
        goto out;
    }

    /* 5. init(解锁闪存) */
    if (!swd_flash_syscall_exec(&algo->prog.sys_call_s, algo->prog.init,
                                m.addr, 0, 0, 0, FLASHALGO_RETURN_BOOL)) {
        fail(DAP_ERR_TARGET, "algo_init_fail");
        goto out;
    }
    algo_inited = true;
    set_progress(5);

    /* 6. 逐页擦除(5% → 30%) */
    {
        uint32_t pages = (m.size + algo->page_size - 1) / algo->page_size;
        for (uint32_t i = 0; i < pages; i++) {
            uint32_t adr = m.addr + i * algo->page_size;
            if (!swd_flash_syscall_exec(&algo->prog.sys_call_s,
                                        algo->prog.erase_sector,
                                        adr, 0, 0, 0, FLASHALGO_RETURN_BOOL)) {
                fail(DAP_ERR_TARGET, "erase_fail");
                goto out;
            }
            set_progress((uint8_t)(5 + 25 * (i + 1) / pages));
        }
    }

    /* 7. 分页写入(30% → 70%),源数据直读镜像文件 */
    {
        FILE *f = fopen(dap_store_path(slot), "rb");
        if (!f) {
            fail(DAP_ERR_INTERNAL, "open_image_fail");
            goto out;
        }
        uint32_t written = 0;
        while (written < m.size) {
            uint32_t want = m.size - written;
            if (want > algo->prog.program_buffer_size) {
                want = algo->prog.program_buffer_size;
            }
            if (fread(s_chunk, 1, want, f) != want) {
                fclose(f);
                fail(DAP_ERR_INTERNAL, "read_image_fail");
                goto out;
            }
            /* 尾块补零到半字对齐(PY32 按半字编程,补零不影响 CRC 已在
             * 上传时定死,回读校验只比 m.size 字节) */
            if (want & 1) {
                s_chunk[want] = 0;
                want++;
            }
            if (!swd_write_memory(algo->prog.program_buffer, s_chunk, want) ||
                !swd_flash_syscall_exec(&algo->prog.sys_call_s,
                                        algo->prog.program_page,
                                        m.addr + written, want,
                                        algo->prog.program_buffer, 0,
                                        FLASHALGO_RETURN_BOOL)) {
                fclose(f);
                fail(DAP_ERR_TARGET, "program_fail");
                goto out;
            }
            written += want;
            set_progress((uint8_t)(30 + 40 * written / m.size));
        }
        fclose(f);
    }

    /* 8. 回读全片算 CRC(70% → 95%),与上传时定死的 CRC 终值比对 */
    {
        uint32_t crc = 0;
        uint32_t done = 0;
        while (done < m.size) {
            uint32_t want = m.size - done;
            if (want > sizeof(s_chunk)) want = sizeof(s_chunk);
            if (!swd_read_memory(m.addr + done, s_chunk, want)) {
                fail(DAP_ERR_TARGET, "verify_read_fail");
                goto out;
            }
            crc = dap_store_crc32(crc, s_chunk, want);
            done += want;
            set_progress((uint8_t)(70 + 25 * done / m.size));
        }
        if (crc != m.crc32) {
            fail(DAP_ERR_CRC, "verify_crc_mismatch");
            goto out;
        }
    }

    /* 9. 收尾:loader uninit(闪存上锁)+ 硬复位放飞目标 */
    swd_flash_syscall_exec(&algo->prog.sys_call_s, algo->prog.uninit,
                           0, 0, 0, 0, FLASHALGO_RETURN_BOOL);
    algo_inited = false;
    swd_set_target_state_sw(RESET_RUN);
    set_progress(100);

    s_st.err = 0;
    strlcpy(s_st.msg, "ok", sizeof(s_st.msg));
    s_st.state = DAP_OFF_DONE;
    ESP_LOGI(TAG, "flash done: slot=%d %u bytes -> %08x",
             slot, (unsigned)m.size, m.addr);

out:
    if (algo_inited) {
        swd_flash_syscall_exec(&algo->prog.sys_call_s, algo->prog.uninit,
                               0, 0, 0, 0, FLASHALGO_RETURN_BOOL);
        swd_off();
    }
    if (wire_held) {
        xSemaphoreGive(s_wire_lock);
    }
    vTaskDelete(NULL);
}

bool dap_offline_start(int slot, char *reason) {
    if (!s_wire_lock) {
        strlcpy(reason, "engine_not_ready", 48);
        return false;
    }
    if (s_st.state == DAP_OFF_RUNNING) {
        strlcpy(reason, "flash_in_progress", 48);
        return false;
    }
    if (!dap_store_mounted()) {
        strlcpy(reason, "storage_not_mounted", 48);
        return false;
    }
    if (!dap_store_get(slot, NULL)) {
        strlcpy(reason, "slot_empty", 48);
        return false;
    }

    if (xTaskCreate(offline_flash_task, "dap_offline", OFFLINE_TASK_STACK,
                    (void *)(intptr_t)slot, OFFLINE_TASK_PRIO, NULL) != pdPASS) {
        strlcpy(reason, "task_create_fail", 48);
        return false;
    }
    return true;
}
