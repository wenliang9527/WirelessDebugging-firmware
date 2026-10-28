/**
 * @file wifi_handle.c
 * @brief WiFi 初始化、STA 连接轮询、AP 模式启停
 *
 * 产品化 P1 改动:
 *   - 保留上游的 STA 多 SSID 轮转(基于编译期列表 / 本地凭据文件)
 *   - 新增 AP 模式:仅由 @MODE AP 命令触发,不自动降级(C2 约束)
 *   - 新增模式偏好 NVS 持久化,重启后保持上次模式
 *   - 模式切换用 stop -> set_mode -> set_config -> start 序列
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
#include "main/uart_bridge.h"

#include "components/DAP/include/gpio_op.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
    /* 注意:板载 RGB LED 是 WS2812 接 GPIO48,不是普通 LED(见
     * 01_hardware/开发板资料.md §3)。此处保留原值仅为不破坏上游编译,
     * 状态指示将在 P2.5 用 RMT + led_strip 重做。 */
    #define PIN_LED_WIFI_STATUS 4
#else
    #error unknown hardware
#endif

static const char *TAG = "wifi";

/* AP 参数(E3 默认值) */
#define AP_SSID_PREFIX    "DAP-"
#define AP_PASSWORD       "12345678"
#define AP_CHANNEL        6
#define AP_MAX_CONN       4

/* 启动时最长等 IP 的时间。超时不算失败,只是让命令通道先能用;
 * 之后 STA 仍会在后台按列表轮转重连。 */
#define STA_BOOT_WAIT_MS  20000

/* 单次手动连接(@CONNECT)的等待上限 */
#define MANUAL_CONNECT_TIMEOUT_MS 20000

static EventGroupHandle_t wifi_event_group;
static int ssid_index = 0;

static volatile dap_mode_t s_mode = DAP_MODE_STA;
static char s_ap_ssid[33] = AP_SSID_PREFIX "0000";

const int IPV4_GOTIP_BIT = BIT0;

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
            /* STA 刚启动:写入列表里待尝试的 SSID 再连接。
             * ssid_change() 只在断连回调与这里被调用,避免重复推进游标。 */
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
        GPIO_SET_LEVEL_HIGH(PIN_LED_WIFI_STATUS);
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
            ssid_index = 0;
        }
        break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
        GPIO_SET_LEVEL_LOW(PIN_LED_WIFI_STATUS);
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
            /* 扫描前主动断开的,不代表连接失败 —— 忽略,别推进轮转游标 */
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
            /* 手动连接期间不轮转:如实记录原因码交给 @CONNECT 上报。 */
            s_manual_reason = (int)info->disconnected.reason;
            s_manual_ok = false;
            s_manual_done = true;
            s_manual = false;
            ESP_LOGW(TAG, "manual connect to %s failed, reason=%d",
                     s_manual_ssid, s_manual_reason);
            /* 恢复自动轮转 */
            ssid_index = 0;
            if (s_mode == DAP_MODE_STA) {
                ssid_change();
                esp_wifi_connect();
            }
            break;
        }

        /* 产品化约束 C2:不自动降级到 AP。仅 STA 模式下继续轮转重连。 */
        if (s_mode == DAP_MODE_STA) {
            ssid_change();
            esp_wifi_connect();
        }
        break;

    case SYSTEM_EVENT_AP_START:
        ESP_LOGI(TAG, "AP started: ssid=%s", s_ap_ssid);
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
        /* 理论上不会发生(出厂列表至少有一条),兜底避免除零/空转 */
        ESP_LOGW(TAG, "no configured network");
        return;
    }
    if (ssid_index < 0 || ssid_index >= n) {
        ssid_index = 0;
    }

    dap_cfg_ap_t ap;
    if (!dap_config_get(ssid_index, &ap)) {
        ssid_index = 0;
        return;
    }

    wifi_config_t wifi_config = {0};
    /* 用 "%.31s"/"%.63s" 显式限定长度:目标缓冲区(32/64)可能小于源
     * 数组容量(33/65),不加精度限定会触发 -Werror=format-truncation。 */
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%.31s", ap.ssid);
    snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%.63s", ap.password);
    ESP_LOGI(TAG, "trying ssid[%d/%d]: %s", ssid_index, n, wifi_config.sta.ssid);
    ssid_index++;
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
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
    /* 从 list[0] 开始。这里只重置游标,不自己调 ssid_change():
     * esp_wifi_start() 会触发 STA_START,由事件处理统一写入首个 SSID 并连接。
     * 若在此处也调一次,首个 SSID 会被跳过(实测表现为切回 STA 后直接去试
     * 列表末位的 OTA,反复 201)。 */
    ssid_index = 0;
    return esp_wifi_start();
}

static esp_err_t apply_ap_mode(void) {
    wifi_config_t cfg = {0};
    cfg.ap.ssid_len = (uint8_t)strlen(s_ap_ssid);
    memcpy(cfg.ap.ssid, s_ap_ssid, cfg.ap.ssid_len);
    cfg.ap.ssid[cfg.ap.ssid_len] = '\0';
    strncpy((char *)cfg.ap.password, AP_PASSWORD, sizeof(cfg.ap.password) - 1);
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
    GPIO_FUNCTION_SET(PIN_LED_WIFI_STATUS);
    GPIO_SET_DIRECTION_NORMAL_OUT(PIN_LED_WIFI_STATUS);

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
        ESP_LOGI(TAG, "AP ready: ssid=%s ip=192.168.4.1", s_ap_ssid);
        dap_cmd_set_wifi_ready(true);
        return;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    /* 不在这里写 SSID:交给 STA_START 事件统一处理,保证"启动"和"切回 STA"
     * 走同一条路径,游标不会因两处各推一次而错位。 */
    ssid_index = 0;
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "waiting for AP connection...");
    /* 不能无限等:连不上路由器时设备必须仍可被 @MODE 接管,
     * 否则命令通道会被"永远卡在启动阶段"这个门闩挡死。
     * 超时后放行命令通道,wifi 轮转会继续在后台重试。 */
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, IPV4_GOTIP_BIT,
                                           false, true, pdMS_TO_TICKS(STA_BOOT_WAIT_MS));
    if (bits & IPV4_GOTIP_BIT) {
        ESP_LOGI(TAG, "connected");
    } else {
        ESP_LOGW(TAG, "no AP within %d ms; keeping STA retry in background, "
                      "command channel now accepts @MODE", STA_BOOT_WAIT_MS);
    }
    dap_cmd_set_wifi_ready(true);
}
