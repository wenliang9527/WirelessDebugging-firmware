/*
Copyright (c) 2015, SuperHouse Automation Pty Ltd
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file uart_bridge.c
 * @author windowsair
 * @brief UART TCP bridge
 * @version 0.1
 * @date 2021-11-16
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "sdkconfig.h"

#include <string.h>
#include <stdint.h>
#include <sys/param.h>
#include <stdatomic.h>

#include "main/wifi_configuration.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netbuf.h"
#include "lwip/api.h"
#include "lwip/tcp.h"
#include <lwip/netdb.h>

#if (USE_UART_BRIDGE == 1)

#ifdef CONFIG_IDF_TARGET_ESP8266
    #define UART_BRIDGE_TX UART_NUM_0
    #define UART_BRIDGE_RX UART_NUM_1
#elif defined CONFIG_IDF_TARGET_ESP32
    #define UART_BRIDGE_TX UART_NUM_2
    #define UART_BRIDGE_RX UART_NUM_2
    #define UART_BRIDGE_TX_PIN 23
    #define UART_BRIDGE_RX_PIN 22
#elif defined CONFIG_IDF_TARGET_ESP32C3
    #define UART_BRIDGE_TX UART_NUM_1
    #define UART_BRIDGE_RX UART_NUM_1
    #define UART_BRIDGE_TX_PIN 19
    #define UART_BRIDGE_RX_PIN 18 // PIN18 has 50000ns glitch during the power-up
#elif defined CONFIG_IDF_TARGET_ESP32S3
    // S3: GPIO19/20 是 USB D-/D+(COM18 esptool 口),不能碰;SWD 占 11/12,console 占 43/44
    #define UART_BRIDGE_TX UART_NUM_1
    #define UART_BRIDGE_RX UART_NUM_1
    #define UART_BRIDGE_TX_PIN 17
    #define UART_BRIDGE_RX_PIN 18
#else
    #error unknown hardware
#endif

#define EVENTS_QUEUE_SIZE 10
#define UART_BUF_SIZE     512

#ifdef CALLBACK_DEBUG
#define debug(s, ...) os_printf("%s: " s "\n", "Cb:", ##__VA_ARGS__)
#else
#define debug(s, ...)
#endif

static const char *UART_TAG = "UART";
#define NETCONN_EVT_WIFI_DISCONNECTED (NETCONN_EVT_ERROR + 1)

static QueueHandle_t uart_server_events = NULL;
typedef struct
{
    struct netconn *nc;
    uint8_t type;
} netconn_events;

static uint8_t uart_read_buffer[UART_BUF_SIZE];
// use lwip buffer to write back
static struct netconn *uart_netconn = NULL;
static bool is_conn_valid = false; // lock free
static bool is_first_time_recv = false;

void uart_bridge_close() {
    if (uart_server_events == NULL) {
        return; // init 尚未执行(极早期 WiFi 事件),无需通知
    }
    netconn_events events;
    events.type = NETCONN_EVT_WIFI_DISCONNECTED;
    xQueueSend(uart_server_events, &events, 1000);
}

static void uart_bridge_reset() {
    uart_netconn = NULL;
    is_conn_valid = false;
}

static int num_digits(int n) {
    if (n < 10)
        return 1;
    if (n < 100)
        return 2;
    if (n < 1000)
        return 3;
    if (n < 10000)
        return 4;
    if (n < 100000)
        return 5;
    if (n < 1000000)
        return 6;

    return 7;
}

/*
 * This function will be call in Lwip in each event on netconn
 */
static void netCallback(struct netconn *conn, enum netconn_evt evt, uint16_t length) {
    // Show some callback information (debug)
    debug("sock:%u\tsta:%u\tevt:%u\tlen:%u\ttyp:%u\tfla:%02x\terr:%d",
          (uint32_t)conn, conn->state, evt, length, conn->type, conn->flags, conn->pending_err);

    netconn_events events;

    // If netconn got error, it is close or deleted, dont do treatments on it.
    if (conn->pending_err) {
        return;
    }
    // Treatments only on rcv events.
    switch (evt) {
    case NETCONN_EVT_RCVPLUS:
        events.nc = conn;
        events.type = evt;
        break;
    default:
        return;
        break;
    }

    // Send the event to the queue
    xQueueSend(uart_server_events, &events, pdMS_TO_TICKS(1000));
}

/*
 *  Initialize a server netconn and listen port
 */
static void set_tcp_server_netconn(struct netconn **nc, uint16_t port, netconn_callback callback) {
    if (nc == NULL) {
        ESP_LOGE(UART_TAG, "%s: netconn missing .\n", __FUNCTION__);
        return;
    }
    *nc = netconn_new_with_callback(NETCONN_TCP, netCallback);
    if (!*nc) {
        ESP_LOGE(UART_TAG, "Status monitor: Failed to allocate netconn.\n");
        return;
    }
    netconn_set_nonblocking(*nc, NETCONN_FLAG_NON_BLOCKING);
    // netconn_set_recvtimeout(*nc, 10);
    netconn_bind(*nc, IP_ADDR_ANY, port);
    netconn_listen(*nc);
}

/*
 *  Close and delete a socket properly
 */
static void close_tcp_netconn(struct netconn *nc) {
    nc->pending_err = ERR_CLSD; // It is hacky way to be sure than callback will don't do treatment on a netconn closed and deleted
    netconn_close(nc);
    netconn_delete(nc);
}

static void uart_bridge_setup() {
    uart_config_t uart_config = {
        .baud_rate = UART_BRIDGE_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    if (UART_BRIDGE_TX == UART_BRIDGE_RX) {
        uart_param_config(UART_BRIDGE_RX, &uart_config);
        uart_driver_install(UART_BRIDGE_RX, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0);
    } else {
        uart_param_config(UART_BRIDGE_RX, &uart_config);
        uart_param_config(UART_BRIDGE_TX, &uart_config);

        uart_driver_install(UART_BRIDGE_RX, UART_BUF_SIZE, 0, 0, NULL, 0); // RX only
        uart_driver_install(UART_BRIDGE_TX, 0, UART_BUF_SIZE, 0, NULL, 0); // TX only
    }

#if defined CONFIG_IDF_TARGET_ESP32 || defined CONFIG_IDF_TARGET_ESP32C3 || defined CONFIG_IDF_TARGET_ESP32S3
    uart_set_pin(UART_BRIDGE_TX, UART_BRIDGE_TX_PIN, UART_BRIDGE_RX_PIN, -1, -1);
#endif

}

void uart_bridge_init() {
    uart_server_events = xQueueCreate(EVENTS_QUEUE_SIZE, sizeof(netconn_events));
}

void uart_bridge_task() {
    uart_bridge_setup();
    /* 队列已在 uart_bridge_init() 建好(main 里先调 init 再建任务)。
     * 这里原本又 xQueueCreate 一次 —— 第一次的队列泄漏,而且 wifi 事件里
     * 的 uart_bridge_close() 可能往旧队列发事件,任务却读新队列。 */
    if (uart_server_events == NULL) {
        uart_server_events = xQueueCreate(EVENTS_QUEUE_SIZE, sizeof(netconn_events));
    }

    struct netconn *nc = NULL; // To create servers

    set_tcp_server_netconn(&nc, UART_BRIDGE_PORT, netCallback);

    struct netbuf *netbuf = NULL; // To store incoming Data
    struct netconn *nc_in = NULL; // To accept incoming netconn
    //
    char *buffer;
    uint16_t len_buf;
    size_t uart_buf_len;

    while (1) {
        netconn_events events;
        int ret = xQueueReceive(uart_server_events, &events, pdMS_TO_TICKS(100));

        if (ret != pdTRUE) {
            // timeout tick: drain UART into the client
            if (is_conn_valid && uart_netconn != NULL) {
                // 不要用 ESP_ERROR_CHECK:拿不到长度只该跳过这一轮,
                // 不该 abort 整机(历史上这里 abort 过一次就重启)。
                if (uart_get_buffered_data_len(UART_BRIDGE_RX, &uart_buf_len) != ESP_OK) {
                    uart_buf_len = 0;
                }
                uart_buf_len = uart_buf_len > UART_BUF_SIZE ? UART_BUF_SIZE : uart_buf_len;
                uart_buf_len = uart_read_bytes(UART_BRIDGE_RX, uart_read_buffer, uart_buf_len, pdMS_TO_TICKS(5));
                if (uart_buf_len > 0 && netconn_write(uart_netconn, uart_read_buffer, uart_buf_len, NETCONN_COPY) != ERR_OK) {
                    // peer is gone (write retries exhausted): release the slot
                    ESP_LOGE(UART_TAG, "write fail (tick), closing");
                    close_tcp_netconn(uart_netconn);
                    uart_bridge_reset();
                }
            }
        } else if (events.type == NETCONN_EVT_WIFI_DISCONNECTED) { // WIFI disconnected
            if (is_conn_valid && uart_netconn != NULL) {
                close_tcp_netconn(uart_netconn);
                uart_bridge_reset();
            }
        } else if (events.nc == nc) {
            // event on the LISTEN conn; pointer-compare dispatch: a stale event
            // of an already-deleted client conn must never be dereferenced here
            if (is_conn_valid) {
                if (netconn_accept(events.nc, &nc_in) == ERR_OK && nc_in != NULL) {
                    ESP_LOGI(UART_TAG, "busy: kicked newcomer");
                    close_tcp_netconn(nc_in);
                }
                continue;
            }

            if (netconn_accept(events.nc, &nc_in) == ERR_OK && nc_in != NULL) {
                ESP_LOGI(UART_TAG, "client accepted");
                uart_netconn = nc_in;
                is_conn_valid = true;
                is_first_time_recv = true;
            }
        } else if (events.nc == uart_netconn) {
            // if (events.nc && events.nc->pcb.tcp)
            //     tcp_nagle_disable(events.nc->pcb.tcp);

            uart_netconn = events.nc;
            // read data from UART(同 tick 分支:错误只跳过,不 abort)
            if (uart_get_buffered_data_len(UART_BRIDGE_RX, &uart_buf_len) != ESP_OK) {
                uart_buf_len = 0;
            }
            uart_buf_len = uart_buf_len > UART_BUF_SIZE ? UART_BUF_SIZE : uart_buf_len;
            uart_buf_len = uart_read_bytes(UART_BRIDGE_RX, uart_read_buffer, uart_buf_len, pdMS_TO_TICKS(5));
            // then send data
            if (netconn_write(uart_netconn, uart_read_buffer, uart_buf_len, NETCONN_COPY) != ERR_OK) {
                // peer is gone (write retries exhausted): release the slot
                ESP_LOGE(UART_TAG, "write fail, closing");
                close_tcp_netconn(uart_netconn);
                uart_bridge_reset();
                continue;
            }

            // try to get data
            err_t rerr = netconn_recv(events.nc, &netbuf);
            if (rerr == ERR_OK) // data incoming ?
            {
                uint16_t rx_len = 0;
                do {
                    netbuf_data(netbuf, (void *)&buffer, &len_buf);
                    rx_len += len_buf;
                    // write to uart
                    if (is_first_time_recv) { // change line config: "<baud>[E|O|N]", e.g. "19200E" = 19200 8E1
                        if (len_buf > 1 && len_buf < 8) {
                            char tmp_buff[8];
                            memcpy(tmp_buff, buffer, len_buf);
                            tmp_buff[len_buf] = '\0';
                            uart_parity_t parity = UART_PARITY_DISABLE;
                            int len = len_buf;
                            char pc = tmp_buff[len - 1];
                            if (pc == 'E' || pc == 'e') {
                                parity = UART_PARITY_EVEN;
                                tmp_buff[--len] = '\0';
                            } else if (pc == 'O' || pc == 'o') {
                                parity = UART_PARITY_ODD;
                                tmp_buff[--len] = '\0';
                            } else if (pc == 'N' || pc == 'n') {
                                parity = UART_PARITY_DISABLE;
                                tmp_buff[--len] = '\0';
                            }
                            int baudrate = atoi(tmp_buff);
                            if (baudrate > 0 && baudrate < 2000000 && num_digits(baudrate) == len) {
                                ESP_LOGI(UART_TAG, "change line:%d 8%s1", baudrate,
                                         parity == UART_PARITY_EVEN ? "E" : (parity == UART_PARITY_ODD ? "O" : "N"));
                                uart_set_baudrate(UART_BRIDGE_RX, baudrate);
                                uart_set_baudrate(UART_BRIDGE_TX, baudrate);
                                uart_set_parity(UART_BRIDGE_RX, parity);
                                uart_set_parity(UART_BRIDGE_TX, parity);
                                is_first_time_recv = false;
                                continue;
                            }
                        }
                        is_first_time_recv = false;
                    }
                    uart_write_bytes(UART_BRIDGE_TX, (const char *)buffer, len_buf);
                } while (netbuf_next(netbuf) >= 0);
                netbuf_delete(netbuf);
                if (rx_len == 0) {
                    // empty netbuf = peer closed gracefully (FIN): release the slot
                    ESP_LOGI(UART_TAG, "FIN rx, closing");
                    close_tcp_netconn(uart_netconn);
                    uart_bridge_reset();
                }
            } else {
                // any recv error (incl. ERR_CLSD) means the peer is gone
                ESP_LOGE(UART_TAG, "recv err=%d, closing", rerr);
                close_tcp_netconn(uart_netconn);
                uart_bridge_reset();
            }
        } else {
            ESP_LOGI(UART_TAG, "stale event dropped");
        }
    }
}

#endif // (USE_UART_BRIDGE == 1)