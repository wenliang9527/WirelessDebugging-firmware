/**
 * @file dap_flash_algo.h
 * @brief 离线烧录 D4:目标家族烧录算法注册表
 *
 * 每个家族一个条目:RAM loader blob + DAPLink program_target_t 约定的
 * 入口地址/RAM 安排 + 页大小/闪存基址。新家族(AT32/STM32)在此追加
 * 条目即可,引擎(dap_offline.c)不感知家族细节。
 */
#ifndef __DAP_FLASH_ALGO_H__
#define __DAP_FLASH_ALGO_H__

#include <stdint.h>
#include "components/DAP/include/flash_blob.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *family;      /* 与镜像索引里的 family 字符串精确匹配,如 "PY32" */
    uint32_t page_size;      /* 擦除页大小(字节),镜像基址必须对齐到它 */
    uint32_t flash_base;     /* 家族闪存基址,如 0x08000000 */
    program_target_t prog;   /* DAPLink 约定的完整算法描述 */
} dap_flash_algo_t;

/** 按家族名找算法,未注册返回 NULL */
const dap_flash_algo_t *dap_flash_algo_find(const char *family);

#ifdef __cplusplus
}
#endif

#endif /* __DAP_FLASH_ALGO_H__ */
