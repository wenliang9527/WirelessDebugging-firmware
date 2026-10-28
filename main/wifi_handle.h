#ifndef _WIFI_HANDLE_H_
#define _WIFI_HANDLE_H_

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "main/dap_config.h"

/** 扫描结果条目 */
typedef struct {
    char    ssid[DAP_CFG_SSID_LEN];
    int8_t  rssi;
    uint8_t authmode;   /* wifi_auth_mode_t */
    uint8_t channel;
} dap_scan_result_t;

/**
 * WiFi 硬件初始化 + 按 NVS 记录的偏好模式启动。
 * STA 模式下最多阻塞 STA_BOOT_WAIT_MS 等 IP;AP 模式下立即返回。
 */
void wifi_init(void);

/**
 * 前台扫描周围 AP。
 * 仅 STA / APSTA 模式可用(纯 AP 模式下 IDF 不支持扫描),否则返回
 * ESP_ERR_NOT_SUPPORTED —— 调用方应上报 @ERR 409 MODE_AP。
 * 结果按 RSSI 降序、按 SSID 去重。
 */
esp_err_t dap_wifi_scan(dap_scan_result_t *out, int max, int *found);

/**
 * 手动连接指定 AP。异步:立刻返回,结果由事件回调记录,
 * 用 dap_wifi_get_manual_result() 轮询。
 */
esp_err_t dap_wifi_connect_manual(const char *ssid, const char *password);

/** 是否有手动连接正在进行 */
bool dap_wifi_busy(void);

/** 取手动连接结果 */
void dap_wifi_get_manual_result(bool *done, bool *ok, int *reason);

#endif
