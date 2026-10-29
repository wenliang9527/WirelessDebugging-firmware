/**
 * @file wifi_handle.c
 * @brief WiFi 初始化、STA 主网连接、AP 模式启停
 *
 * 产品化主网模型(2026-10-29 定稿):
 *   - 不做多 SSID 轮转。主网 = 连接顺序逻辑 0(pin 过的最靠前,否则最近连的),
 *     连不上只按退避序列重试同一条(1/2/5/10/15/30s),绝不自动换网。
 *     理由:多网段环境里轮转会造成 IP 漂移,PC 端表现为"连接不稳定"。
 *   - 换主网只能走 PC 工具(@CONNECT / @PIN);恢复出厂后回到未配网态。
 *   - 新增 AP 模式:仅由 @MODE AP 命令触发,不自动降级(C2 约束)
 *   - 新增模式偏好 NVS 持久化,重启后保持上次模式
 */
#include "sdkconfig.h"

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/param.h>

#include "main/wifi_configuration.h"
#include "main/wifi_handle.h"
#include "main/dap_cmd.h"
#include "main/dap_config.h"
#include "main/dap_version.h"
#include "main/dap_led.h"
#include "main/uart_bridge.h"

#include "components/DAP/include/gpio_op.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"

#ifdef CONFIG_IDF_TARGET_ESP8266
    #define PIN_LED_WIFI_STATUS 15
#elif defined CONFIG_IDF_TARGET_ESP32
    #define PIN_LED_WIFI_STATUS 27
#elif defined CONFIG_IDF_TARGET_ESP32C3
    #define PIN_LED_WIFI_STATUS 10
#elif defined CONFIG_IDF_TARGET_ESP32S3
    /* 本板(YD-ESP32-S3)的 RGB LED 是 WS2812 接 GPIO48,不能用 GPIO 电平点。
     * 状态指示统一由 dap_led.c 用 RMT 驱动;这里保留宏只为不破坏上游引用。 */
    #define PIN_LED_WIFI_STATUS 48
    #define USE_WS2812_STATUS_LED 1
#else
    #error unknown hardware
#endif

/** 兼容宏:非 WS2812 板型仍走老的单色 LED 电平控制 */
#ifndef USE_WS2812_STATUS_LED
#define USE_WS2812_STATUS_LED 0
#endif

static inline void led_indicate(dap_led_state_t st)
{
#if USE_WS2812_STATUS_LED
    dap_led_set(st);
#else
    (void)st;
#endif
}

static const char *TAG = "wifi";

/* AP 参数 */
#define AP_SSID_PREFIX    "DAP-"
#define AP_CHANNEL        6
#define AP_MAX_CONN       4
/* 密码不再写死:由 dap_config_get_ap_password() 提供(首启随机生成、存 NVS)。
 * 原来这里是 #define AP_PASSWORD "12345678" —— 那个串在源码和所有文档里
 * 都是公开的,等于热点没密码。 */

/* 启动时最长等 IP 的时间。超时不算失败,只是让命令通道先能用;
 * 之后 STA 仍会在后台重试主网。 */
#define STA_BOOT_WAIT_MS  20000

/* 主网重试退避:连不上时按这个序列循环等待(毫秒)。
 * 单主网模型下没有"下一个网络"可换,重试太密只会空耗 RF;
 * 最长 30s 一拍,连上即归零。 */
static const int s_retry_backoff_ms[] = { 1000, 2000, 5000, 10000, 15000, 30000 };
#define RETRY_BACKOFF_N   (sizeof(s_retry_backoff_ms) / sizeof(s_retry_backoff_ms[0]))

/* 当前退避档位与定时器(断连回调里不能 delay,重连交给定时器) */
static volatile int s_retry_stage = 0;
static TimerHandle_t s_retry_timer = NULL;

/* 单次手动连接(@CONNECT)的等待上限 */
#define MANUAL_CONNECT_TIMEOUT_MS 20000

static EventGroupHandle_t wifi_event_group;

static volatile dap_mode_t s_mode = DAP_MODE_STA;
static char s_ap_ssid[33] = AP_SSID_PREFIX "0000";

const int IPV4_GOTIP_BIT = BIT0;

/* 自动重连要试的目标:主网 = 列表里 pin 过的最靠前一条,没 pin 就是第 0 条
 * (即"最近连上的")。连接顺序由 dap_config_conn_physical() 定义,
 * 这里永远只取逻辑 0,不轮转 —— 用户模型:连不上就等,换网走工具。 */
static void ssid_change(void);

/* ------------------------------------------------------------------ */
/* 手动连接(@CONNECT)状态                                             */
/* ------------------------------------------------------------------ */

static volatile bool     s_manual = false;     /* 正在执行手动连接 */
/* 手动连接时我们会主动 disconnect 一次。该动作同样会触发
 * STA_DISCONNECTED 回调,若不区分就会被误判成"连接失败"并清掉 s_manual,
 * 导致随后的 esp_wifi_connect() 返回 ESP_ERR_WIFI_CONN。
 * 这个标志标记"这次断连是自己发起的",回调里直接忽略。 */
static volatile bool     s_manual_switching = false;
static volatile int      s_manual_reason = 0;  /* 最近一次断连原因码 */
static volatile bool     s_manual_done = false;
static volatile bool     s_manual_ok = false;
static char              s_manual_ssid[DAP_CFG_SSID_LEN];
static char              s_manual_pass[DAP_CFG_PASS_LEN];

/* 扫描前也要主动 disconnect(否则 IDF 拒绝扫描:"STA is connecting")。
 * 这次断连同样是自己发起的,回调里必须忽略,否则会白白推进一次
 * SSID 轮转游标,把扫描前的连接目标打乱。 */
static volatile bool     s_scanning = false;

bool dap_wifi_busy(void) {
    return s_manual;
}

void dap_wifi_get_manual_result(bool *done, bool *ok, int *reason) {
    if (done)   *done = s_manual_done;
    if (ok)     *ok = s_manual_ok;
    if (reason) *reason = s_manual_reason;
}

/* ------------------------------------------------------------------ */
/* AP SSID:带 MAC 后缀,避免多台冲突                                   */
/* ------------------------------------------------------------------ */

static void ap_ssid_build(void) {
    uint8_t mac[6] = {0};
    if (esp_efuse_mac_get_default(mac) != ESP_OK) {
        snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s0000", AP_SSID_PREFIX);
        return;
    }
    snprintf(s_ap_ssid, sizeof(s_ap_ssid), "%s%02X%02X",
             AP_SSID_PREFIX, mac[4], mac[5]);
}

const char *dap_cmd_ap_ssid(void) {
    return s_ap_ssid;
}

const char *dap_cmd_mode_name(void) {
    return (s_mode == DAP_MODE_AP) ? "AP" : "STA";
}

/* ------------------------------------------------------------------ */
/* 事件处理                                                            */
/* ------------------------------------------------------------------ */

static esp_err_t event_handler(void *ctx, system_event_t *event) {
    system_event_info_t *info = &event->event_info;

    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        if (s_mode == DAP_MODE_STA) {
            /* STA 刚启动:写主网再连接。没有已保存网络时不连(未配网态)。 */
            ssid_change();
            esp_wifi_connect();
        }
        break;

    case SYSTEM_EVENT_STA_CONNECTED:
        if (s_manual) {
            ESP_LOGI(TAG, "manual connect: associated with %s", s_manual_ssid);
        }
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        led_indicate(DAP_LED_STA_OK);
        s_retry_stage = 0;   /* 连上归零,下次断连从头退避 */
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "STA got ip: %s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        if (s_manual) {
            /* 手动连接成功:落盘并结束 */
            s_manual_ok = true;
            s_manual_done = true;
            s_manual = false;
            if (dap_config_add(s_manual_ssid, s_manual_pass) != ESP_OK) {
                ESP_LOGW(TAG, "failed to persist %s", s_manual_ssid);
            }
        }
        break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
        led_indicate(DAP_LED_CONNECTING);
        ESP_LOGW(TAG, "disconnect reason: %d", (int)info->disconnected.reason);

#ifdef CONFIG_IDF_TARGET_ESP8266
        if (info->disconnected.reason == WIFI_REASON_BASIC_RATE_NOT_SUPPORT) {
            esp_wifi_set_protocol(ESP_IF_WIFI_STA,
                                  WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
        }
#endif
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);

#if (USE_UART_BRIDGE == 1)
        uart_bridge_close();
#endif

        if (s_scanning) {
            /* 扫描前主动断开的,不代表连接失败 —— 忽略 */
            ESP_LOGI(TAG, "scan: own disconnect ignored (reason=%d)",
                     (int)info->disconnected.reason);
            break;
        }
        if (s_manual && s_manual_switching) {
            /* 这是 dap_wifi_connect_manual() 主动断开旧连接引发的,
             * 不代表目标 AP 连接失败 —— 直接忽略,等真正的结果。 */
            ESP_LOGI(TAG, "manual: own disconnect ignored (reason=%d)",
                     (int)info->disconnected.reason);
            break;
        }
        if (s_manual) {
            /* 手动连接期间不自动重试:如实记录原因码交给 @CONNECT 上报,
             * 由回调侧恢复主网重连。 */
            s_manual_reason = (int)info->disconnected.reason;
            s_manual_ok = false;
            s_manual_done = true;
            s_manual = false;
            ESP_LOGW(TAG, "manual connect to %s failed, reason=%d",
                     s_manual_ssid, s_manual_reason);
            if (s_mode == DAP_MODE_STA) {
                ssid_change();
                esp_wifi_connect();
            }
            break;
        }

        /* 主网模型:没有"下一个网络"可换,按退避序列重试同一条。
         * 产品化约束 C2:不自动降级到 AP;换网只能走工具。 */
        if (s_mode == DAP_MODE_STA) {
            int wait = s_retry_backoff_ms[s_retry_stage];
            if (s_retry_stage < (int)RETRY_BACKOFF_N - 1) {
                s_retry_stage++;
            }
            ssid_change();
            ESP_LOGI(TAG, "retry main network in %d ms", wait);
            /* 事件回调里不能 vTaskDelay(卡住整个 WiFi 任务):改定时器周期
             * 后单发触发,回调里 esp_wifi_connect()。 */
            xTimerChangePeriod(s_retry_timer, pdMS_TO_TICKS(wait), 0);
        }
        break;

    case SYSTEM_EVENT_AP_START:
        ESP_LOGI(TAG, "AP started: ssid=%s", s_ap_ssid);
        led_indicate(DAP_LED_AP_MODE);
        break;

    case SYSTEM_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "AP client joined");
        break;

    case SYSTEM_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "AP client left");
        break;

    default:
        break;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* STA 轮转                                                            */
/* ------------------------------------------------------------------ */

static void ssid_change(void) {
    int n = dap_config_count();
    if (n <= 0) {
        /* 未配网态:没有任何已保存网络。什么都不做,等 PC 工具经 USB 配网。
         * LED 由 STA_START 分支置红色快闪提示。 */
        ESP_LOGW(TAG, "no configured network; waiting for provisioning via USB");
        led_indicate(DAP_LED_FAULT);
        return;
    }
    /* 主网 = 连接顺序里的逻辑 0:pin 过的最靠前一条,没 pin 就是最近连的。
     * 映射由 dap_config_conn_physical() 完成,这里永远只取 0,不轮转。 */
    int phys = dap_config_conn_physical(0);
    if (phys < 0) {
        phys = 0;
    }

    dap_cfg_ap_t ap;
    if (!dap_config_get(phys, &ap)) {
        return;
    }

    wifi_config_t wifi_config = {0};
    /* 用 "%.31s"/"%.63s" 显式限定长度:目标缓冲区(32/64)可能小于源
     * 数组容量(33/65),不加精度限定会触发 -Werror=format-truncation。 */
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%.31s", ap.ssid);
    snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%.63s", ap.password);
    ESP_LOGI(TAG, "main network: %s%s", wifi_config.sta.ssid,
             ap.pin ? " (pinned)" : "");

    /* 绝对不能用 ESP_ERROR_CHECK:它在失败时直接 abort,整块板子 panic 重启。
     *
     * 实测崩溃现场(2026-10-28):
     *   assert failed: esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config)
     *   abort() was called at PC 0x40382177 on core 0
     *
     * 触发条件:WiFi 驱动正处于连接中/扫描中时,set_config 返回
     * ESP_ERR_WIFI_STATE。而 ssid_change() 恰恰是在 STA_DISCONNECTED 回调
     * 里被调用的 —— 那一刻驱动正在从断开态往重连态过渡,状态极不稳定。
     *
     * 正确做法:失败就跳过这一轮。重试定时器会再来一次。 */
    esp_err_t cfg_err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    if (cfg_err != ESP_OK) {
        ESP_LOGW(TAG, "set_config(%s) failed: %s; retry later",
                 wifi_config.sta.ssid, esp_err_to_name(cfg_err));
    }
}

/* 重试定时器回调:到点后按主网配置发起连接。
 * 放在定时器里是因为 DISCONNECTED 回调不能 vTaskDelay。 */
static void retry_timer_cb(TimerHandle_t t) {
    (void)t;
    if (s_mode == DAP_MODE_STA && !s_manual && !s_scanning) {
        esp_wifi_connect();
    }
}

/* ------------------------------------------------------------------ */
/* 模式切换                                                            */
/* ------------------------------------------------------------------ */

static esp_err_t apply_sta_mode(void) {
    wifi_config_t cfg = {0};
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }
    /* 清空当前 STA 配置,避免沿用 AP 模式前的残留 */
    esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg);
    /* 不在这里写 SSID:esp_wifi_start() 会触发 STA_START,由事件处理统一
     * 写主网并连接(未配网态则置故障灯等待配网)。 */
    s_retry_stage = 0;
    return esp_wifi_start();
}

static esp_err_t apply_ap_mode(void) {
    wifi_config_t cfg = {0};
    cfg.ap.ssid_len = (uint8_t)strlen(s_ap_ssid);
    memcpy(cfg.ap.ssid, s_ap_ssid, cfg.ap.ssid_len);
    cfg.ap.ssid[cfg.ap.ssid_len] = '\0';
    /* 密码来自 NVS(首启随机生成)。get 保证非空 —— 空密码会让 AP 变开放
     * 网络,WPA2 的 authmode 又要求至少 8 位,配置会直接失败。 */
    const char *ap_pass = dap_config_get_ap_password();
    strncpy((char *)cfg.ap.password, ap_pass, sizeof(cfg.ap.password) - 1);
    cfg.ap.channel = AP_CHANNEL;
    cfg.ap.max_connection = AP_MAX_CONN;
    cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_wifi_set_config(ESP_IF_WIFI_AP, &cfg);
    if (err != ESP_OK) {
        return err;
    }
    return esp_wifi_start();
}

esp_err_t dap_cmd_set_mode(dap_mode_t mode) {
    if (mode == s_mode) {
        return ESP_OK;  /* 已是目标模式,幂等 */
    }

    ESP_LOGI(TAG, "switching mode -> %s", (mode == DAP_MODE_AP) ? "AP" : "STA");

    esp_wifi_disconnect();
    esp_wifi_stop();

    xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
#if (USE_UART_BRIDGE == 1)
    uart_bridge_close();
#endif

    s_mode = mode;
    esp_err_t err = (mode == DAP_MODE_AP) ? apply_ap_mode() : apply_sta_mode();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mode switch failed: %s", esp_err_to_name(err));
        /* 尽力恢复:回到 STA 以免设备失联 */
        s_mode = DAP_MODE_STA;
        esp_wifi_stop();
        apply_sta_mode();
        return err;
    }

    dap_config_set_mode_pref((mode == DAP_MODE_AP) ? 1 : 0);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* 扫描与手动连接(P2)                                                 */
/* ------------------------------------------------------------------ */

esp_err_t dap_wifi_scan(dap_scan_result_t *out, int max, int *found) {
    if (!out || !found || max <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    *found = 0;

    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }
    /* 纯 AP 模式下 WiFi 驱动不支持扫描(IDF: 仅 Station 或 Station+AP)。
     * 这是硬约束,不是本项目的选择。 */
    if (mode != WIFI_MODE_STA && mode != WIFI_MODE_APSTA) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* 关键:IDF 在 STA 处于 connecting 状态时禁止扫描,驱动会直接返回
     * ESP_FAIL 并在日志里写 "sta_scan: STA is connecting, scan are not allowed!"。
     *
     * 这不是边缘情况——自动轮转模式下板子会不停地在列表里换着试,几乎永远
     * 处在 connecting,导致 @SCAN 几乎必然失败。
     *
     * 判据不能用 esp_wifi_sta_get_ap_info:它在"连接中"时会返回上一次关联的
     * 残留记录(实测 ip 还是 0.0.0.0 时它就返回 OK 并给出 rssi/channel),
     * 据此判断会漏掉最常见的场景。所以这里无条件先断开,保证驱动回到 idle。
     * 代价是一次扫描会让当前连接尝试中断,扫完恢复。
     */
    s_scanning = true;
    esp_wifi_disconnect();
    /* 让驱动把 disconnect 处理完并进入 idle,否则 scan_start 仍会被拒。
     * 150ms 实测不够,给 300ms。 */
    vTaskDelay(pdMS_TO_TICKS(300));

    /* 前台扫描:block=true 让驱动扫完再返回,内部超时约 3-4s */
    wifi_scan_config_t sc = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
    };
    esp_err_t err = esp_wifi_scan_start(&sc, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "scan_start failed: %s", esp_err_to_name(err));
        goto restore;
    }

    uint16_t ap_num = 0;
    if (esp_wifi_scan_get_ap_num(&ap_num) != ESP_OK || ap_num == 0) {
        err = ESP_OK;  /* 0 个结果不算错误 */
        goto restore;
    }
    if (ap_num > 32) {
        ap_num = 32;
    }

    wifi_ap_record_t *recs = calloc(ap_num, sizeof(wifi_ap_record_t));
    if (!recs) {
        err = ESP_ERR_NO_MEM;
        goto restore;
    }
    uint16_t got = ap_num;
    err = esp_wifi_scan_get_ap_records(&got, recs);
    if (err != ESP_OK) {
        free(recs);
        goto restore;
    }

    /* 按 SSID 去重(同名多 BSSID 只留信号最强的),再按 RSSI 降序冒泡排序
     * —— 结果集很小(通常 <20),不必上 qsort 也足够快。 */
    int n = 0;
    for (uint16_t i = 0; i < got && n < max; i++) {
        if (recs[i].ssid[0] == '\0') {
            continue;  /* 隐藏 SSID 对配网无意义 */
        }
        int dup = -1;
        for (int k = 0; k < n; k++) {
            if (strncmp((char *)out[k].ssid, (char *)recs[i].ssid,
                        sizeof(out[k].ssid)) == 0) {
                dup = k;
                break;
            }
        }
        if (dup >= 0) {
            if (recs[i].rssi > out[dup].rssi) {
                out[dup].rssi = recs[i].rssi;
                out[dup].authmode = (uint8_t)recs[i].authmode;
                out[dup].channel = recs[i].primary;
            }
            continue;
        }
        memcpy(out[n].ssid, recs[i].ssid, sizeof(out[n].ssid));
        out[n].ssid[sizeof(out[n].ssid) - 1] = '\0';
        out[n].rssi = recs[i].rssi;
        out[n].authmode = (uint8_t)recs[i].authmode;
        out[n].channel = recs[i].primary;
        n++;
    }
    free(recs);

    for (int i = 1; i < n; i++) {
        dap_scan_result_t key = out[i];
        int j = i - 1;
        while (j >= 0 && out[j].rssi < key.rssi) {
            out[j + 1] = out[j];
            j--;
        }
        out[j + 1] = key;
    }

    *found = n;

restore:
    /* 扫完把自动轮转交回系统。
     * 扫描前无条件断开了,所以这里也要无条件恢复 —— 否则板子会停在
     * "断开且不再重连"的状态,整机失去 WiFi。 */
    s_scanning = false;
    if (s_mode == DAP_MODE_STA && !s_manual) {
        esp_wifi_connect();
    }
    return err;
}

esp_err_t dap_wifi_connect_manual(const char *ssid, const char *password) {
    if (!ssid || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_manual) {
        return ESP_ERR_INVALID_STATE;  /* 已有一次在跑 */
    }
    if (s_mode != DAP_MODE_STA) {
        return ESP_ERR_INVALID_STATE;  /* AP 模式下不能连 */
    }

    strncpy(s_manual_ssid, ssid, sizeof(s_manual_ssid) - 1);
    s_manual_ssid[sizeof(s_manual_ssid) - 1] = '\0';
    strncpy(s_manual_pass, password ? password : "", sizeof(s_manual_pass) - 1);
    s_manual_pass[sizeof(s_manual_pass) - 1] = '\0';

    s_manual_done = false;
    s_manual_ok = false;
    s_manual_reason = 0;
    s_manual = true;

    wifi_config_t cfg = {0};
    snprintf((char *)cfg.sta.ssid, sizeof(cfg.sta.ssid), "%.31s", s_manual_ssid);
    snprintf((char *)cfg.sta.password, sizeof(cfg.sta.password), "%.63s", s_manual_pass);

    ESP_LOGI(TAG, "manual connect -> %s", s_manual_ssid);

    /* 先断开当前连接,再用目标 AP 的配置重连。
     * 不要 set_mode/stop,否则会触发 STA_START 走自动轮转。
     * disconnect 之后驱动需要时间落定:紧接着 connect 会返回
     * ESP_ERR_WIFI_CONN(驱动仍在处理上一条命令)或 ESP_ERR_NO_MEM。
     * 实测 200ms 不够——扫描恰好刚在末尾调过 esp_wifi_connect() 恢复轮转时
     * 尤其明显,所以这里改成"短暂等待 + 确认驱动落定"的方式。 */
    s_manual_switching = true;
    esp_wifi_disconnect();

    /* 等驱动从 connecting/associated 状态退出来。wifi 状态机在自己跑,
     * 轮询 sta_get_ap_info 失败 + 再让出几拍,比固定 sleep 可靠。 */
    for (int i = 0; i < 12; i++) {
        vTaskDelay(pdMS_TO_TICKS(60));
        wifi_ap_record_t probe;
        if (esp_wifi_sta_get_ap_info(&probe) != ESP_OK) {
            /* 已断开;再多让一拍让驱动内部清理完 */
            vTaskDelay(pdMS_TO_TICKS(80));
            break;
        }
    }

    esp_err_t err = esp_wifi_set_config(ESP_IF_WIFI_STA, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "manual: set_config failed %s", esp_err_to_name(err));
        s_manual = false;
        s_manual_switching = false;
        return err;
    }
    err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_CONN) {
        /* 驱动还在忙:再等一拍重试一次,这是实测最容易撞上的一个 */
        ESP_LOGW(TAG, "manual: connect busy, retrying once");
        vTaskDelay(pdMS_TO_TICKS(300));
        err = esp_wifi_connect();
    }
    s_manual_switching = false;   /* 之后任何断连都是真实结果 */
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "manual: esp_wifi_connect failed %s", esp_err_to_name(err));
        s_manual = false;
        return err;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */

static void wifi_hw_init_stage1(void) {
#if USE_WS2812_STATUS_LED
    /* WS2812 用 RMT 驱动,不能当普通 GPIO 配置(会破坏单总线时序)。
     * 初始化失败只是没有灯,不影响 WiFi 与命令通道。 */
    if (dap_led_init() != ESP_OK) {
        ESP_LOGW(TAG, "status LED unavailable, continuing without it");
    }
#else
    GPIO_FUNCTION_SET(PIN_LED_WIFI_STATUS);
    GPIO_SET_DIRECTION_NORMAL_OUT(PIN_LED_WIFI_STATUS);
#endif

    tcpip_adapter_init();

    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ap_ssid_build();
    if (dap_config_get_mode_pref() == 1) {
        s_mode = DAP_MODE_AP;
    }
}

/* 上游 API 保留:完成 WiFi 硬件初始化 + 按模式启动并等待(仅 STA 会阻塞到拿到 IP) */
void wifi_init(void) {
    wifi_hw_init_stage1();

#if (USE_STATIC_IP == 1)
    /* 仅 STA 模式有意义 */
#endif

    ESP_LOGI(TAG, "boot mode = %s", dap_cmd_mode_name());

    if (s_mode == DAP_MODE_AP) {
        ESP_ERROR_CHECK(apply_ap_mode());
        /* 密码不进日志。现场排障要密码时走 @AP(USB 命令通道),
         * 那是可控的本地通道;ESP_LOG 会进串口日志、用户会截图发群里。 */
        ESP_LOGI(TAG, "AP ready: ssid=%s ip=192.168.4.1 (password via @AP)",
                 s_ap_ssid);
        dap_cmd_set_wifi_ready(true);
        return;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    /* 重试定时器:单发,到点回调里 esp_wifi_connect()。断连事件按退避档位
     * xTimerReset 设定周期;连上后 GOT_IP 清零档位。 */
    s_retry_timer = xTimerCreate("sta_retry", pdMS_TO_TICKS(1000),
                                 pdFALSE, NULL, retry_timer_cb);
    /* 不在这里写 SSID:交给 STA_START 事件统一处理,保证"启动"和"切回 STA"
     * 走同一条路径。 */
    s_retry_stage = 0;
    ESP_ERROR_CHECK(esp_wifi_start());

    /* 门闩在这里就该放开:esp_wifi_start() 返回时 WiFi 驱动已经可接受
     * set_mode/stop,等 IP 只是"能不能上网",和"能不能切模式"无关。
     *
     * 早前把门闩放在"等到 IP 或超时"之后,导致开机后约 20 秒内 @MODE 一直
     * 返回 405 wifi_still_starting —— 而那恰恰是最想切 AP 的场景。实测
     * @STATUS 已经能正常返回时 @MODE 仍被拒,体验很怪。 */
    dap_cmd_set_wifi_ready(true);
    ESP_LOGI(TAG, "wifi driver ready; @MODE accepted from now on");

    ESP_LOGI(TAG, "waiting for AP connection...");
    led_indicate(DAP_LED_CONNECTING);
    /* 这里只是"顺便等一会儿再放行 @SCAN 之类",超时不影响命令通道。 */
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, IPV4_GOTIP_BIT,
                                           false, true, pdMS_TO_TICKS(STA_BOOT_WAIT_MS));
    if (bits & IPV4_GOTIP_BIT) {
        ESP_LOGI(TAG, "connected");
    } else {
        ESP_LOGW(TAG, "no AP within %d ms; keeping STA retry in background",
                 STA_BOOT_WAIT_MS);
        /* 启动阶段一直没连上:红色快闪,和"正在连接"区分开。
         * 后台仍会继续轮转,连上后 GOT_IP 会把灯改成绿色常亮。 */
        led_indicate(DAP_LED_FAULT);
    }
}
