/**
 * @file dap_cmd.h
 * @brief 产品化 P1:USB(COM18)命令通道
 *
 * 线格式(ASCII 文本行,\n 结尾):
 *   PC -> 板   @命令 [参数...]
 *   板 -> PC   @OK ... / @ERR <码> <说明> / @DATA ... / @EVT ...
 *
 * 板端 IDF 日志不带 '@' 前缀,PC 侧丢弃即可。
 */
#ifndef __DAP_CMD_H__
#define __DAP_CMD_H__

#include <stdbool.h>
#include "esp_err.h"

/** 错误码(与 PC 工具约定,便于程序判断而非靠字符串匹配) */
enum {
    DAP_ERR_OK            = 0,
    DAP_ERR_BADARG        = 400,  /* 参数缺失/格式错 */
    DAP_ERR_UNKNOWN       = 404,  /* 未知命令 */
    DAP_ERR_STATE         = 405,  /* 当前状态不允许(如 AP 下扫描) */
    DAP_ERR_MODE_AP       = 409,  /* 需要在 STA 模式(扫描/连接) */
    DAP_ERR_BUSY          = 423,  /* 上一命令未完成 */
    DAP_ERR_TIMEOUT       = 408,  /* 等待超时 */
    DAP_ERR_INTERNAL      = 500,
    /* WiFi 断连原因码直接透传 IDF 值 */
    DAP_ERR_NO_AP_FOUND   = 201,
    DAP_ERR_AUTH_FAIL     = 202,
    DAP_ERR_HANDSHAKE     = 204,
};

/** 运行模式 */
typedef enum {
    DAP_MODE_STA = 0,
    DAP_MODE_AP  = 1,
} dap_mode_t;

/** 初始化命令通道(装 USJ 驱动 + 起接收任务)。须在 console 切到 USJ 后调用。 */
esp_err_t dap_cmd_init(void);

/** 通道是否就绪 */
bool dap_cmd_is_ready(void);

/**
 * 标记 WiFi 基座是否已完成初始化。
 * wifi_init() 在 STA 模式下阻塞等 IP,期间必须禁止 @MODE 改动 WiFi 状态,
 * 否则会与其启动流程竞争(实测会导致 AP 被拉起两次)。
 */
void dap_cmd_set_wifi_ready(bool ready);

/* --- 响应输出(供命令实现与 wifi 侧事件复用) --- */
void dap_cmd_reply(const char *fmt, ...);
void dap_cmd_ok(const char *fmt, ...);
void dap_cmd_err(int code, const char *msg);
void dap_cmd_data(const char *fmt, ...);

/* --- 模式(实现在 wifi_handle.c,避免协议层直接碰 WiFi) --- */
esp_err_t   dap_cmd_set_mode(dap_mode_t mode);
const char *dap_cmd_mode_name(void);
const char *dap_cmd_ap_ssid(void);

#endif
