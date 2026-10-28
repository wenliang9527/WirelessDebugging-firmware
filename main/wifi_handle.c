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
#include <sys/param.h>

#include "main/wifi_configuration.h"
#include "main/wifi_handle.h"
#include "main/dap_cmd.h"
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

/* NVS:模式偏好 */
#define NVS_NS_CFG        "dap_cfg"
#define NVS_KEY_MODE      "mode"

/* AP 参数(E3 默认值) */
#define AP_SSID_PREFIX    "DAP-"
#define AP_PASSWORD       "12345678"
#define AP_CHANNEL        6
#define AP_MAX_CONN       4

/* 启动时最长等 IP 的时间。超时不算失败,只是让命令通道先能用;
 * 之后 STA 仍会在后台按列表轮转重连。 */
#define STA_BOOT_WAIT_MS  20000

static EventGroupHandle_t wifi_event_group;
static int ssid_index = 0;

static volatile dap_mode_t s_mode = DAP_MODE_STA;
static char s_ap_ssid[33] = AP_SSID_PREFIX "0000";

const int IPV4_GOTIP_BIT = BIT0;

static void ssid_change(void);

/* ------------------------------------------------------------------ */
/* NVS:模式偏好                                                        */
/* ------------------------------------------------------------------ */

static void mode_load(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS_CFG, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    uint8_t v = 0;
    if (nvs_get_u8(h, NVS_KEY_MODE, &v) == ESP_OK && v == DAP_MODE_AP) {
        s_mode = DAP_MODE_AP;
    }
    nvs_close(h);
}

static void mode_save(dap_mode_t m) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS_CFG, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "nvs open failed, mode not persisted");
        return;
    }
    nvs_set_u8(h, NVS_KEY_MODE, (uint8_t)m);
    nvs_commit(h);
    nvs_close(h);
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
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        GPIO_SET_LEVEL_HIGH(PIN_LED_WIFI_STATUS);
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "STA got ip: %s", ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
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
    if (ssid_index > (int)WIFI_LIST_SIZE - 1) {
        ssid_index = 0;
    }

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, wifi_list[ssid_index].ssid,
            sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, wifi_list[ssid_index].password,
            sizeof(wifi_config.sta.password) - 1);
    ESP_LOGI(TAG, "trying ssid[%d]: %s", ssid_index, wifi_config.sta.ssid);
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

    mode_save(mode);
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
    mode_load();
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
