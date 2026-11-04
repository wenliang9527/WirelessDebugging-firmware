/**
 * @file dap_store.c
 * @brief 离线烧录 D1:烧录镜像存储层实现
 *
 * 并发模型:三段式上传(TCP 任务或 @FPUT console 任务)与命令查询
 * (console 任务)可能同时进来,一把互斥锁护住"文件句柄 + 暂存元数据 +
 * NVS 索引"。烧录引擎(D4)只读已 finalize 的槽位文件,不走这把锁。
 */
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_littlefs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

#include "main/dap_store.h"

#define FS_MOUNT_POINT   "/img"
#define FS_PARTITION_LABEL "storage"
#define NVS_NAMESPACE    "imgstore"
#define NVS_KEY_CUR      "cur"
#define NVS_KEY_SEQ      "seq"
#define META_MAGIC       0x544F5244 /* "DROT" */

static const char *TAG = "dap_store";

typedef struct {
    bool           active;
    int            slot;
    dap_store_meta_t meta;   /* begin 时暂存,finish 时落 NVS */
    uint32_t       crc;      /* 写入过程增量累计 */
    size_t         written;
    FILE          *fp;
} upload_ctx_t;

static bool              s_mounted = false;
static SemaphoreHandle_t s_lock;
static upload_ctx_t      s_up;

/* ------------------------------------------------------------------ */
/* CRC-32/zlib:反射多项式 0xEDB88320,初值/终值异或 0xFFFFFFFF。       */
/* 逐位实现,不依赖 ROM 函数的变体差异——PC 端 zlib.crc32 必须逐位一致。 */
/* ------------------------------------------------------------------ */
uint32_t dap_store_crc32(uint32_t prev, const uint8_t *data, size_t len) {
    uint32_t crc = ~prev; /* 调用方传 0 起步;增量续算时 prev 传上次返回值 */
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;
}

/* ------------------------------------------------------------------ */
/* NVS 索引:key = meta0..meta7(blob = dap_store_meta_t)              */
/*          全局键:cur(u8 当前槽)、seq(u32 上传计数器)             */
/* ------------------------------------------------------------------ */
static void meta_key(char *buf, size_t n, int slot) {
    snprintf(buf, n, "meta%d", slot);
}

static bool nvs_read_meta(nvs_handle_t h, int slot, dap_store_meta_t *out) {
    char key[8];
    meta_key(key, sizeof(key), slot);
    dap_store_meta_t m;
    size_t len = sizeof(m);
    if (nvs_get_blob(h, key, &m, &len) != ESP_OK || len != sizeof(m) ||
        m.magic != META_MAGIC) {
        return false;
    }
    /* 索引在而文件丢失(理论不该发生,格式化后残留)视为空槽 */
    struct stat st;
    if (stat(dap_store_path(slot), &st) != 0 || (uint32_t)st.st_size != m.size) {
        return false;
    }
    if (out) *out = m;
    return true;
}

static esp_err_t nvs_write_meta(nvs_handle_t h, int slot, const dap_store_meta_t *m) {
    char key[8];
    meta_key(key, sizeof(key), slot);
    esp_err_t err = nvs_set_blob(h, key, m, sizeof(*m));
    if (err == ESP_OK) err = nvs_commit(h);
    return err;
}

/* 当前槽标记的收口:指向空槽时回落到最旧(seq 最小),全空则 -1 */
static int resolve_current(nvs_handle_t h) {
    uint8_t cur = 0xFF;
    nvs_get_u8(h, NVS_KEY_CUR, &cur);
    if (cur < DAP_STORE_MAX_SLOT && nvs_read_meta(h, cur, NULL)) {
        return cur;
    }
    int best = -1;
    uint32_t best_seq = 0;
    dap_store_meta_t m;
    for (int i = 0; i < DAP_STORE_MAX_SLOT; i++) {
        if (!nvs_read_meta(h, i, &m)) continue;
        if (best < 0 || m.seq < best_seq) {
            best = i;
            best_seq = m.seq;
        }
    }
    if (best >= 0) {
        nvs_set_u8(h, NVS_KEY_CUR, (uint8_t)best);
        nvs_commit(h);
    }
    return best;
}

/* ------------------------------------------------------------------ */
/* 对外 API                                                             */
/* ------------------------------------------------------------------ */
esp_err_t dap_store_init(void) {
    s_lock = xSemaphoreCreateMutex();

    esp_vfs_littlefs_conf_t conf = {
        .base_path = FS_MOUNT_POINT,
        .partition_label = FS_PARTITION_LABEL,
        .format_if_mount_failed = true, /* 首次挂载(全新分区)即格式化 */
    };
    esp_err_t err = esp_vfs_littlefs_register(&conf);
    if (err != ESP_OK) {
        /* 存储层坏不应拖死调试器主功能:记日志降级,命令层报"未挂载" */
        ESP_LOGE(TAG, "mount '%s' failed: %d(离线烧录不可用)", FS_PARTITION_LABEL, err);
        s_mounted = false;
        return err;
    }
    s_mounted = true;

    /* 清理历史 .tmp 残留(上传中途断电的证据,留着也只会占空间) */
    int cleaned = 0;
    for (int i = 0; i < DAP_STORE_MAX_SLOT; i++) {
        char tmp[24];
        snprintf(tmp, sizeof(tmp), "%s.tmp", dap_store_path(i));
        if (unlink(tmp) == 0) cleaned++;
    }

    size_t total = 0, used = 0;
    if (esp_littlefs_info(FS_PARTITION_LABEL, &total, &used) == ESP_OK) {
        ESP_LOGI(TAG, "mounted %s: %u/%u KB used, %d tmp cleaned, cur=slot %d",
                 FS_MOUNT_POINT,
                 (unsigned)(used / 1024),
                 (unsigned)(total / 1024),
                 cleaned, dap_store_current());
    }
    return ESP_OK;
}

bool dap_store_mounted(void) {
    return s_mounted;
}

const char *dap_store_path(int slot) {
    static char path[16];
    snprintf(path, sizeof(path), FS_MOUNT_POINT "/slot%d.bin", slot);
    return path;
}

bool dap_store_get(int slot, dap_store_meta_t *out) {
    if (slot < 0 || slot >= DAP_STORE_MAX_SLOT || !s_mounted) return false;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
    bool ok = nvs_read_meta(h, slot, out);
    nvs_close(h);
    return ok;
}

int dap_store_count(void) {
    int n = 0;
    for (int i = 0; i < DAP_STORE_MAX_SLOT; i++) {
        if (dap_store_get(i, NULL)) n++;
    }
    return n;
}

int dap_store_free_slot(void) {
    for (int i = 0; i < DAP_STORE_MAX_SLOT; i++) {
        if (!dap_store_get(i, NULL)) return i;
    }
    return -1;
}

esp_err_t dap_store_begin(int slot, const dap_store_meta_t *meta) {
    if (!s_mounted) return ESP_ERR_INVALID_STATE;
    if (slot < 0 || slot >= DAP_STORE_MAX_SLOT || !meta ||
        meta->size == 0 || meta->name[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    /* 互斥锁从 begin 持到 finish/abort;若上一会话还占着(TCP 与 USB 兜底
     * 抢用、或上次异常中断没走到收尾),直接拒绝而不是死等。 */
    if (s_up.active) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_lock, portMAX_DELAY);

    char tmp[24];
    snprintf(tmp, sizeof(tmp), "%s.tmp", dap_store_path(slot));
    s_up.fp = fopen(tmp, "wb");
    if (!s_up.fp) {
        xSemaphoreGive(s_lock);
        return ESP_FAIL;
    }
    s_up.active = true;
    s_up.slot = slot;
    s_up.meta = *meta;
    s_up.meta.magic = META_MAGIC;
    s_up.crc = 0;
    s_up.written = 0;
    return ESP_OK;
}

esp_err_t dap_store_write(const uint8_t *data, size_t len) {
    if (!s_mounted || !s_up.active) return ESP_ERR_INVALID_STATE;
    /* 锁由 begin 持有到 finish/abort,write 只会来自同一上传会话 */
    if (!s_up.fp) return ESP_ERR_INVALID_STATE;
    if (s_up.written + len > s_up.meta.size) return ESP_ERR_INVALID_SIZE;
    if (len && fwrite(data, 1, len, s_up.fp) != len) return ESP_FAIL;
    s_up.crc = dap_store_crc32(s_up.crc, data, len); /* crc==0 起步,见头文件 */
    s_up.written += len;
    return ESP_OK;
}

esp_err_t dap_store_finish(void) {
    if (!s_mounted || !s_up.active) return ESP_ERR_INVALID_STATE;

    esp_err_t err = ESP_OK;
    do {
        fclose(s_up.fp);
        s_up.fp = NULL;

        if (s_up.written != s_up.meta.size) { err = ESP_ERR_INVALID_SIZE; break; }
        if (s_up.crc != s_up.meta.crc32)    { err = ESP_ERR_INVALID_CRC;  break; }

        char tmp[24], dst[16];
        snprintf(tmp, sizeof(tmp), "%s.tmp", dap_store_path(s_up.slot));
        snprintf(dst, sizeof(dst), "%s", dap_store_path(s_up.slot));
        unlink(dst);            /* 覆盖旧镜像 */
        if (rename(tmp, dst) != 0) { err = ESP_FAIL; break; }

        nvs_handle_t h;
        if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
            err = ESP_FAIL;
            break;
        }
        uint32_t seq = 0;
        nvs_get_u32(h, NVS_KEY_SEQ, &seq);
        s_up.meta.seq = ++seq;
        nvs_set_u32(h, NVS_KEY_SEQ, seq);
        err = nvs_write_meta(h, s_up.slot, &s_up.meta);
        nvs_close(h);
    } while (0);

    if (err != ESP_OK) {
        char tmp[24];
        snprintf(tmp, sizeof(tmp), "%s.tmp", dap_store_path(s_up.slot));
        unlink(tmp);
    }
    s_up.active = false;
    xSemaphoreGive(s_lock);
    return err;
}

void dap_store_abort(void) {
    if (!s_up.active) return;
    if (s_up.fp) fclose(s_up.fp);
    char tmp[24];
    snprintf(tmp, sizeof(tmp), "%s.tmp", dap_store_path(s_up.slot));
    unlink(tmp);
    s_up.active = false;
    xSemaphoreGive(s_lock);
}

esp_err_t dap_store_rename(int slot, const char *name) {
    if (!s_mounted) return ESP_ERR_INVALID_STATE;
    if (slot < 0 || slot >= DAP_STORE_MAX_SLOT || !name || !name[0]) {
        return ESP_ERR_INVALID_ARG;
    }
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return ESP_FAIL;
    dap_store_meta_t m;
    if (!nvs_read_meta(h, slot, &m)) {
        nvs_close(h);
        return ESP_ERR_NOT_FOUND;
    }
    strlcpy(m.name, name, sizeof(m.name));
    esp_err_t err = nvs_write_meta(h, slot, &m);
    nvs_close(h);
    return err;
}

esp_err_t dap_store_del(int slot) {
    if (!s_mounted) return ESP_ERR_INVALID_STATE;
    if (slot < 0 || slot >= DAP_STORE_MAX_SLOT) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return ESP_FAIL;
    if (!nvs_read_meta(h, slot, NULL)) {
        nvs_close(h);
        return ESP_ERR_NOT_FOUND;
    }
    unlink(dap_store_path(slot));
    char key[8];
    meta_key(key, sizeof(key), slot);
    nvs_erase_key(h, key);
    nvs_commit(h);
    resolve_current(h);   /* 删的是当前槽时在这里回落 */
    nvs_close(h);
    return ESP_OK;
}

esp_err_t dap_store_set_current(int slot) {
    if (!s_mounted) return ESP_ERR_INVALID_STATE;
    if (slot < 0 || slot >= DAP_STORE_MAX_SLOT) return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return ESP_FAIL;
    if (!nvs_read_meta(h, slot, NULL)) {
        nvs_close(h);
        return ESP_ERR_NOT_FOUND;
    }
    nvs_set_u8(h, NVS_KEY_CUR, (uint8_t)slot);
    esp_err_t err = nvs_commit(h);
    nvs_close(h);
    return err;
}

int dap_store_current(void) {
    if (!s_mounted) return -1;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return -1;
    int cur = resolve_current(h);
    nvs_close(h);
    return cur;
}

esp_err_t dap_store_crc_verify(int slot, uint32_t *out_crc) {
    dap_store_meta_t m;
    if (!dap_store_get(slot, &m)) return ESP_ERR_NOT_FOUND;

    FILE *f = fopen(dap_store_path(slot), "rb");
    if (!f) return ESP_FAIL;
    uint32_t crc = 0;
    uint8_t buf[1024];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        crc = dap_store_crc32(crc, buf, n);
    }
    fclose(f);
    if (out_crc) *out_crc = crc;
    return crc == m.crc32 ? ESP_OK : ESP_ERR_INVALID_CRC;
}
