/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdint.h>
#include <sys/param.h>

#include "sdkconfig.h"
#include "main/tcp_server.h"
#include "main/tcp_netconn.h"
#include "main/kcp_server.h"
#include "main/uart_bridge.h"
#include "main/timer.h"
#include "main/wifi_configuration.h"
#include "main/wifi_handle.h"
#include "main/dap_version.h"
#include "main/dap_cmd.h"
#include "main/dap_config.h"

#include "components/corsacOTA/src/corsacOTA.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "mdns.h"

extern void DAP_Setup(void);
extern void DAP_Thread(void *argument);
extern void SWO_Thread();

TaskHandle_t kDAPTaskHandle = NULL;

static const char *MDNS_TAG = "server_common";

#if defined(CONFIG_IDF_TARGET_ESP32S3)
#define DAP_TASK_AFFINITY 1
#else
#define DAP_TASK_AFFINITY 0
#endif

void mdns_setup() {
    // initialize mDNS
    int ret;
    ret = mdns_init();
    if (ret != ESP_OK) {
        ESP_LOGW(MDNS_TAG, "mDNS initialize failed:%d", ret);
        return;
    }

    // set mDNS hostname
    ret = mdns_hostname_set(MDNS_HOSTNAME);
    if (ret != ESP_OK) {
        ESP_LOGW(MDNS_TAG, "mDNS set hostname failed:%d", ret);
        return;
    }
    ESP_LOGI(MDNS_TAG, "mDNS hostname set to: [%s]", MDNS_HOSTNAME);

    // set default mDNS instance name
    ret = mdns_instance_name_set(MDNS_INSTANCE);
    if (ret != ESP_OK) {
        ESP_LOGW(MDNS_TAG, "mDNS set instance name failed:%d", ret);
        return;
    }
    ESP_LOGI(MDNS_TAG, "mDNS instance name set to: [%s]", MDNS_INSTANCE);
}

void app_main() {
    // struct rst_info *rtc_info = system_get_rst_info();

    // os_printf("reset reason: %x\n", rtc_info->reason);

    // if (rtc_info->reason == REASON_WDT_RST ||
    //     rtc_info->reason == REASON_EXCEPTION_RST ||
    //     rtc_info->reason == REASON_SOFT_WDT_RST)
    // {
    // if (rtc_info->reason == REASON_EXCEPTION_RST)
    // {
    //     os_printf("Fatal exception (%d):\n", rtc_info->exccause);
    // }
    // os_printf("epc1=0x%08x, epc2=0x%08x, epc3=0x%08x,excvaddr=0x%08x, depc=0x%08x\n",
    //             rtc_info->epc1, rtc_info->epc2, rtc_info->epc3,
    //             rtc_info->excvaddr, rtc_info->depc);
    // }

    ESP_ERROR_CHECK(nvs_flash_init());

    // 启动横幅:现场排障时先确认固件身份与 MAC
    // 注意:此处 WiFi 尚未初始化,不能用 esp_wifi_get_mac(),必须读 eFuse
    {
        uint8_t mac[6] = {0};
        esp_err_t mac_err = esp_efuse_mac_get_default(mac);
        if (mac_err == ESP_OK) {
            ESP_LOGI("dap", "=== %s | mac=%02X:%02X:%02X:%02X:%02X:%02X | idf=%s ===",
                     DAP_FW_ID, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                     esp_get_idf_version());
        } else {
            ESP_LOGI("dap", "=== %s | mac=unavailable(%d) | idf=%s ===",
                     DAP_FW_ID, (int)mac_err, esp_get_idf_version());
        }
        ESP_LOGI("dap", "=== wifi list: %d entries (%s) ===",
                 (int)WIFI_LIST_SIZE,
                 WIFI_LIST_IS_FACTORY_DEFAULT ? "factory default" : "local credentials");
    }

    // 配置层必须先起来:@STATUS/@SCAN/@LIST 等命令都依赖它读列表与模式偏好
    dap_config_init();

#if (USE_UART_BRIDGE == 1)
    uart_bridge_init();
#endif
    // 命令通道要尽早起来:STA 模式下 wifi_init() 会一直阻塞到拿到 IP,
    // 若在其之后启动,连不上路由器时就永远没法用 @MODE 切换模式。
    // 这里 NVS 已初始化、WiFi 基座(esp_wifi_init)尚未就绪,
    // 因此命令表里依赖 WiFi 状态的项目会在执行时自行报错。
    dap_cmd_init();

    wifi_init();
    DAP_Setup();
    timer_init();

#if (USE_MDNS == 1)
    mdns_setup();
#endif


#if (USE_OTA == 1)
    co_handle_t handle;
    co_config_t config = {
        .thread_name = "corsacOTA",
        .stack_size = 3192,
        .thread_prio = 8,
        .listen_port = 3241,
        .max_listen_num = 2,
        .wait_timeout_sec = 60,
        .wait_timeout_usec = 0,
    };

    corsacOTA_init(&handle, &config);
#endif

    // Specify the usbip server task
#if (USE_TCP_NETCONN == 1)
    xTaskCreatePinnedToCore(tcp_netconn_task, "tcp_server", 4096, NULL, 14, NULL, DAP_TASK_AFFINITY);
#else // BSD style
    xTaskCreatePinnedToCore(tcp_server_task, "tcp_server", 4096, NULL, 14, NULL,
                            DAP_TASK_AFFINITY);
#endif

    // DAP handle task
    xTaskCreatePinnedToCore(DAP_Thread, "DAP_Task", 2048, NULL, 10, &kDAPTaskHandle,
                            DAP_TASK_AFFINITY);

/* 串口桥任务栈:原 2048 太小,实测(2026-10-29)只要 client 被 accept 就爆栈:
 *   I (88121) UART: client accepted
 *   ***ERROR*** A stack overflow in task uart_server has been detected.
 *   Backtrace: panic_abort <- esp_system_abort <- vApplicationStackOverflowHook
 * 每次连 1234 都触发一次 esp_restart(RTC_SW_CPU_RST),表现为"板子反复重启、
 * 1234 时而连不上"。爆栈点是 netconn_accept/netconn_recv 那几层 lwIP 调用,
 * 它们内部要几百字节到 1KB 不等的临时栈。给到 6144(与原 FIXME 对应)。
 * ESP8266 路线维持原值 1024 不动:它的 RAM 极紧且手头无板可验。 */
#if defined CONFIG_IDF_TARGET_ESP8266
    #define UART_BRIDGE_TASK_STACK_SIZE 1024
#else
    #define UART_BRIDGE_TASK_STACK_SIZE 6144
#endif

#if (USE_UART_BRIDGE == 1)
    xTaskCreate(uart_bridge_task, "uart_server", UART_BRIDGE_TASK_STACK_SIZE, NULL, 2, NULL);
#endif
}
