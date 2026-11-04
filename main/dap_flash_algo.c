/**
 * @file dap_flash_algo.c
 * @brief 离线烧录 D4:目标家族烧录算法注册表实现
 *
 * PY32 条目:loader 源码在 flash_algo/py32f0/(自写,STM32F1 同构闪存
 * 控制器序列),blob 与地址头由 gen_blob.py 从交叉编译产物生成。
 */
#include <string.h>

#include "main/dap_flash_algo.h"
#include "main/flash_algo/py32f0/py32f0_addrs.h"

extern const uint32_t py32f0_blob[PY32F0_BLOB_WORDS];

/* verify=0:不加载目标侧校验函数,由引擎回读 + 板端 CRC 兜底 */
static const dap_flash_algo_t s_algos[] = {
    {
        .family = "PY32",
        .page_size = PY32F0_PAGE_SIZE,
        .flash_base = PY32F0_FLASH_BASE,
        .prog = {
            .init = PY32F0_ADDR_INIT,
            .uninit = PY32F0_ADDR_UNINIT,
            .erase_chip = 0,                 /* 不做整片擦,逐页擦 */
            .erase_sector = PY32F0_ADDR_ERASE_SECTOR,
            .program_page = PY32F0_ADDR_PROGRAM_PAGE,
            .verify = 0,
            .sys_call_s = {
                .breakpoint = PY32F0_ADDR_BKPT,
                .static_base = 0x20000160u,  /* loader 无静态数据,占位 */
                .stack_pointer = PY32F0_STACK_TOP,
            },
            .program_buffer = PY32F0_BUFFER,
            .algo_start = PY32F0_ADDR_INIT,
            .algo_size = PY32F0_BLOB_WORDS * 4,
            .algo_blob = py32f0_blob,
            .program_buffer_size = PY32F0_BUFFER_SIZE,
            .algo_flags = 0,
        },
    },
};

const dap_flash_algo_t *dap_flash_algo_find(const char *family) {
    if (!family || !family[0]) return NULL;
    for (size_t i = 0; i < sizeof(s_algos) / sizeof(s_algos[0]); i++) {
        if (strcmp(s_algos[i].family, family) == 0) return &s_algos[i];
    }
    return NULL;
}
