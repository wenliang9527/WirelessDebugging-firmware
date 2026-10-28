/**
 * @file dap_led.h
 * @brief P2.5:板载 WS2812 状态指示灯
 *
 * 背景:上游 wifi_handle.c 里的 PIN_LED_WIFI_STATUS 是给别的板型写的普通
 * GPIO。"WiFi 连上亮灯"在这块 YD-ESP32-S3 上从未生效过 —— 板载 RGB LED
 * 是 WS2812(可寻址),DIN 接 GPIO48,必须用 RMT 送 ~800kHz 单总线时序。
 * 依据见 01_hardware/开发板资料.md §5.5(厂商原理图 V1.4)。
 *
 * 设计取舍:
 *   - 单独一个低优先级任务跑闪烁动画,事件回调只改目标状态,不阻塞 WiFi。
 *   - 亮度限幅:WS2812 全亮约 20mA/通道,刺眼且费电,统一按 15% 走。
 *   - 初始化失败不影响主功能(拿不到 RMT 通道就静默降级为"没有灯")。
 */
#ifndef __DAP_LED_H__
#define __DAP_LED_H__

#include <stdbool.h>
#include "esp_err.h"

/** 指示灯语义状态 */
typedef enum {
    DAP_LED_BOOT = 0,     /* 启动中:白色呼吸 */
    DAP_LED_CONNECTING,   /* STA 正在连接:黄色慢闪 */
    DAP_LED_STA_OK,       /* STA 已连上且拿到 IP:绿色常亮 */
    DAP_LED_AP_MODE,      /* AP 模式(等待配网):蓝色慢闪 */
    DAP_LED_FAULT,        /* 无可用网络 / 出错:红色快闪 */
} dap_led_state_t;

/**
 * 初始化 WS2812。失败只是没有灯,不影响 WiFi 与命令通道。
 * @return ESP_OK 表示灯可用
 */
esp_err_t dap_led_init(void);

/** 切换状态(线程安全,可在事件回调里调用) */
void dap_led_set(dap_led_state_t state);

/** 关灯(恢复出厂/重启前可用) */
void dap_led_off(void);

/** LED 是否真的可用(供 @STATUS 之类上报) */
bool dap_led_available(void);

/**
 * 运行时改亮度(1..100,超出范围自动夹紧)。
 * 立即生效,但**不落盘** —— 要持久化请走 @LED <pct>(它同时写 NVS)。
 */
void dap_led_set_brightness(uint8_t pct);

/** 当前生效亮度百分比 */
uint8_t dap_led_get_brightness(void);

#endif
