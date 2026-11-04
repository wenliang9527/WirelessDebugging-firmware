/**
 * @file dap_offline.h
 * @brief 离线烧录 D4/D5:板上烧录引擎(绕过 CMSIS-DAP 命令分发,直调 swd_host)
 *
 * 触发:@FLASH(GUI 经命令通道)或 GPIO0 长按(D5)。两者都只是往
 * 任务队列里塞事件,引擎单任务串行执行,天然不会自我并发。
 *
 * 与在线调试互斥:一把"线锁"(dap_offline_wire_lock)。烧录会话整体
 * 持锁(拿不到=在线调试进行中,报 busy);在线路径(DAP_handle 的
 * DAP_ProcessCommand、websocket 的 DAP_ExecuteCommand)逐命令持锁,
 * 烧录期间它们排队等待,烧完自动恢复——会话不散,只是停摆几秒。
 */
#ifndef __DAP_OFFLINE_H__
#define __DAP_OFFLINE_H__

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DAP_OFF_IDLE = 0,
    DAP_OFF_RUNNING,
    DAP_OFF_DONE,    /* 上一次烧录成功(读到的是结果,不是实时状态) */
    DAP_OFF_FAIL,
} dap_off_state_t;

typedef struct {
    dap_off_state_t state;
    int      slot;
    uint8_t  pct;         /* 0..100(粗粒度:擦5-30/写30-70/校70-95/复位98) */
    uint32_t idcode;      /* 本次连上的目标 DP IDCODE(0=没连上),诊断用 */
    int      err;         /* 失败时: dap_cmd.h 的 DAP_ERR_* */
    char     msg[48];     /* 失败原因(人话短句) */
} dap_off_status_t;

/** 建线锁(在 DAP 数据面任务起来之前调用,main.c) */
void dap_offline_init(void);

/** 线锁句柄:DAP_handle/websocket 逐命令持锁用 */
SemaphoreHandle_t dap_offline_wire_lock(void);

/**
 * 触发一次烧录(立即返回,引擎后台跑,进度看 dap_offline_get_status)。
 * reason 指向 ≥48 字节的缓冲,被拒时填人话原因。
 * 返回 false 的情形:引擎已在跑 / 无此槽 / 家族未注册 / 参数非法。
 */
bool dap_offline_start(int slot, char *reason);

/** 当前状态快照(单写入者任务,msg/pct 先写、state 最后写,读侧无锁) */
void dap_offline_get_status(dap_off_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* __DAP_OFFLINE_H__ */
