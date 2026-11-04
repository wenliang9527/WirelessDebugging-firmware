/**
 * @file dap_store.h
 * @brief 离线烧录 D1:烧录镜像存储层(LittleFS + NVS 索引)
 *
 * 设计要点:
 *   - 镜像文件放 storage 分区(LittleFS,挂载点 /img,见 partitions.csv),
 *     上限 DAP_STORE_MAX_SLOT 个槽位,文件名固定 /img/slotN.bin
 *   - 元数据(名字/CRC/家族/基址/序号)存 NVS 命名空间 imgstore,不用
 *     索引文件——两处一致性比省几个 NVS 字节贵
 *   - 上传走三段式 begin→write*→finish:临时文件 + finish 时 CRC/长度
 *     双核对通过才落 NVS,中途断电只会留下被忽略的 .tmp 残留
 *   - 板上没有时间源(NTP 未配),mtime 语义改为上传序号 seq(NVS 计数器),
 *     "最新上传" = seq 最大
 *   - CRC 算法固定为 CRC-32/zlib(反射多项式 0xEDB88320,初值/终值异或
 *     0xFFFFFFFF),与 Python zlib.crc32 逐位一致,PC/板两端不会串味
 */
#ifndef __DAP_STORE_H__
#define __DAP_STORE_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DAP_STORE_MAX_SLOT   8
#define DAP_STORE_NAME_LEN   32   /* 含 NUL,显示名最长 31 字节 */
#define DAP_STORE_FAMILY_LEN 8    /* 含 NUL:"PY32"/"AT32"/"STM32F1" 恰好放下 */

/** 一条镜像的元数据(NVS blob 原样落盘,加 magic 便于日后迁移) */
typedef struct {
    uint32_t magic;                      /* 结构版本 */
    uint32_t size;                       /* 字节数 */
    uint32_t crc32;                      /* CRC-32/zlib */
    uint32_t seq;                        /* 上传序号(板上无时钟,用它表达新旧) */
    uint32_t addr;                       /* 目标烧录基址,通常 0x08000000 */
    char     name[DAP_STORE_NAME_LEN];   /* 显示名 */
    char     family[DAP_STORE_FAMILY_LEN]; /* 烧录算法家族 */
} dap_store_meta_t;

/** 挂载 storage 分区(失败自动格式化;仍失败则标记未挂载,不炸启动) */
esp_err_t dap_store_init(void);

/** 分区是否可用(未挂载时下列 API 均返回 ESP_ERR_INVALID_STATE) */
bool dap_store_mounted(void);

/** 第 slot 槽是否有镜像(有则把元数据拷到 *out;out 可为 NULL) */
bool dap_store_get(int slot, dap_store_meta_t *out);

/** 当前镜像总数 */
int dap_store_count(void);

/** 第一个空槽(无则 -1) */
int dap_store_free_slot(void);

/**
 * 上传三段式。begin 打开临时文件并暂存元数据(CRC/size 由调用方随头帧
 * 给出);write 追加;finish 复算 CRC、核对长度,全部通过才改名落 NVS。
 */
esp_err_t dap_store_begin(int slot, const dap_store_meta_t *meta);
esp_err_t dap_store_write(const uint8_t *data, size_t len);
esp_err_t dap_store_finish(void);
void      dap_store_abort(void);   /* 放弃本次上传(关文件、删临时) */

/** 删除槽位。删的是当前槽时,当前标记自动落到最旧(seq 最小)的槽。 */
esp_err_t dap_store_del(int slot);

/** 设为当前烧录文件(全局唯一) */
esp_err_t dap_store_set_current(int slot);

/** 当前槽号(无当前/全部为空返回 -1) */
int dap_store_current(void);

/** 镜像文件路径("/img/slotN.bin"),指向内部静态缓冲,勿长期持有 */
const char *dap_store_path(int slot);

/** 从文件重算 CRC 并与索引核对(烧录前最后一道保险)。*out_crc 可为 NULL。 */
esp_err_t dap_store_crc_verify(int slot, uint32_t *out_crc);

/** CRC-32/zlib(与 Python zlib.crc32 一致),增量式:prev 初值传 0 */
uint32_t dap_store_crc32(uint32_t prev, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* __DAP_STORE_H__ */
