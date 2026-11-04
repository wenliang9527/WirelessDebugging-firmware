/**
 * @file dap_upload.c
 * @brief 离线烧录 D3:WiFi 镜像上传服务实现(TCP:1235)
 *
 * 帧格式(小端,定长头):
 *   [type u8][len u16][rfu u8] + payload[len]
 *   BEGIN  0x01  slot u8 | name[32] | size u32 | crc32 u32 | family[8] | addr u32
 *   CHUNK  0x02  seq u32 | data...(≤16KB)
 *   ACK    0x03  seq u32 | status u8      (CHUNK 必答,滑窗=1)
 *   FINISH 0x04  (空)
 *   RESULT 0x05  status u8 | msg[64]      (FINISH 的应答,或错误收场)
 *
 * status: 0=OK 1=BUSY(会话被占) 2=BAD_ARG 3=IO 4=SIZE 5=CRC 6=PROTOCOL
 *
 * 为什么用朴素 sockets 而不是 uart_bridge 那套 netconn 回调:那是给 DAP
 * 高吞吐数据面调优的;这里是低频控制流,阻塞式 socket 最直白也最好排障。
 */
#include <string.h>
#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <errno.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "main/dap_store.h"
#include "main/dap_upload.h"

#define FRAME_HDR_SIZE   4
#define MAX_CHUNK        (16 * 1024)
#define RX_BUF_SIZE      (MAX_CHUNK + 8)
#define FRAME_RX_TIMEOUT_MS 10000   /* 帧间超时:上传中断了就收场,别吊着会话 */
#define UPLOAD_TASK_STACK 4096
#define UPLOAD_TASK_PRIO  2

enum {
    FT_BEGIN  = 0x01,
    FT_CHUNK  = 0x02,
    FT_ACK    = 0x03,
    FT_FINISH = 0x04,
    FT_RESULT = 0x05,
};

enum {
    ST_OK       = 0,
    ST_BUSY     = 1,
    ST_BAD_ARG  = 2,
    ST_IO       = 3,
    ST_SIZE     = 4,
    ST_CRC      = 5,
    ST_PROTOCOL = 6,
};

static const char *TAG = "dap.upload";
static uint8_t s_rx[RX_BUF_SIZE];

/* ---------------- 基础收发 ---------------- */

static int recv_all(int fd, uint8_t *buf, size_t n) {
    size_t got = 0;
    while (got < n) {
        ssize_t r = recv(fd, buf + got, n - got, 0);
        if (r < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (r == 0) return -1; /* 对端关闭 */
        got += (size_t)r;
    }
    return 0;
}

static int send_all(int fd, const uint8_t *buf, size_t n) {
    size_t sent = 0;
    while (sent < n) {
        ssize_t r = send(fd, buf + sent, n - sent, 0);
        if (r < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        sent += (size_t)r;
    }
    return 0;
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}
static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/** 发一帧(头 + 载荷) */
static int send_frame(int fd, uint8_t type, const uint8_t *payload, uint16_t len) {
    uint8_t hdr[FRAME_HDR_SIZE] = { type, 0, 0, 0 };
    put_u16(hdr + 1, len);
    if (send_all(fd, hdr, sizeof(hdr)) != 0) return -1;
    if (len && send_all(fd, payload, len) != 0) return -1;
    return 0;
}

static int send_ack(int fd, uint32_t seq, uint8_t status) {
    uint8_t p[5];
    put_u32(p, seq);
    p[4] = status;
    return send_frame(fd, FT_ACK, p, sizeof(p));
}

static int send_result(int fd, uint8_t status, const char *msg) {
    uint8_t p[65];
    p[0] = status;
    strlcpy((char *)p + 1, msg ? msg : "", sizeof(p) - 1);
    return send_frame(fd, FT_RESULT, p, sizeof(p));
}

/* ---------------- 会话处理 ---------------- */

/**
 * 处理一个已连接客户端,直到 FINISH/出错/超时。
 * 返回前保证:若有未收尾的上传会话,已 abort(锁已还)。
 */
static void serve_client(int fd) {
    bool in_session = false;

    /* 帧间超时:挂在 socket 上,recv 返回 EAGAIN 即视为断流 */
    struct timeval tv = { .tv_sec = FRAME_RX_TIMEOUT_MS / 1000, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    for (;;) {
        uint8_t hdr[FRAME_HDR_SIZE];
        if (recv_all(fd, hdr, sizeof(hdr)) != 0) break;

        uint8_t type = hdr[0];
        uint16_t len = (uint16_t)(hdr[1] | (hdr[2] << 8));
        if ((size_t)len > sizeof(s_rx)) {           /* 头就说超长:协议错,收场 */
            send_result(fd, ST_PROTOCOL, "frame_too_long");
            break;
        }
        if (len && recv_all(fd, s_rx, len) != 0) break;

        if (type == FT_BEGIN) {
            if (len != 1 + 32 + 4 + 4 + 8 + 4) {
                send_ack(fd, 0, ST_PROTOCOL);
                break;
            }
            dap_store_meta_t m = {0};
            const uint8_t *p = s_rx;
            int slot = p[0]; p += 1;
            memcpy(m.name, p, 32);   m.name[31] = '\0';   p += 32;
            m.size   = get_u32(p);   p += 4;
            m.crc32  = get_u32(p);   p += 4;
            memcpy(m.family, p, 8);  m.family[7] = '\0';  p += 8;
            m.addr   = get_u32(p);

            dap_store_abort(); /* 残留会话(上次异常断)作废重来 */
            esp_err_t err = dap_store_begin(slot, &m);
            uint8_t st = (err == ESP_OK) ? ST_OK
                       : (err == ESP_ERR_INVALID_STATE) ? ST_BUSY
                       : ST_BAD_ARG;
            if (st == ST_OK) in_session = true;
            send_ack(fd, 0, st);
            if (st != ST_OK) break;
            ESP_LOGI(TAG, "upload begin slot=%d size=%u name=%s",
                     slot, (unsigned)m.size, m.name);
            continue;
        }

        if (type == FT_CHUNK) {
            if (len < 4) {
                send_ack(fd, 0, ST_PROTOCOL);
                break;
            }
            uint32_t seq = get_u32(s_rx);
            if (!in_session) {
                send_ack(fd, seq, ST_BUSY);
                break;
            }
            esp_err_t err = dap_store_write(s_rx + 4, (size_t)len - 4);
            uint8_t st = ST_OK;
            if (err == ESP_ERR_INVALID_SIZE) {
                st = ST_SIZE;
            } else if (err != ESP_OK) {
                st = ST_IO;
            }
            if (send_ack(fd, seq, st) != 0) break;
            if (st != ST_OK) {
                dap_store_abort();
                in_session = false;
                break;
            }
            continue;
        }

        if (type == FT_FINISH) {
            esp_err_t err = dap_store_finish();
            in_session = false;
            if (err == ESP_OK) {
                send_result(fd, ST_OK, "saved");
            } else if (err == ESP_ERR_INVALID_SIZE) {
                send_result(fd, ST_SIZE, "size_mismatch");
            } else if (err == ESP_ERR_INVALID_CRC) {
                send_result(fd, ST_CRC, "crc_mismatch");
            } else {
                send_result(fd, ST_IO, "finish_failed");
            }
            break;
        }

        /* 未知帧类型:协议错 */
        send_result(fd, ST_PROTOCOL, "unknown_frame");
        break;
    }

    if (in_session) {
        dap_store_abort(); /* 断流还占着会话锁可不行 */
    }
}

static void upload_task(void *arg) {
    int lfd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (lfd < 0) {
        ESP_LOGE(TAG, "socket failed: %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int one = 1;
    setsockopt(lfd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(DAP_UPLOAD_PORT);
    if (bind(lfd, (struct sockaddr *)&addr, sizeof(addr)) != 0 ||
        listen(lfd, 1) != 0) {
        ESP_LOGE(TAG, "bind/listen %d failed: %d", DAP_UPLOAD_PORT, errno);
        close(lfd);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "image upload service listening on tcp/%d", DAP_UPLOAD_PORT);

    for (;;) {
        struct sockaddr_in cli;
        socklen_t clen = sizeof(cli);
        int cfd = accept(lfd, (struct sockaddr *)&cli, &clen);
        if (cfd < 0) {
            if (errno == EINTR) continue;
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        serve_client(cfd);
        close(cfd);
    }
}

esp_err_t dap_upload_init(void) {
    if (xTaskCreate(upload_task, "dap_upload", UPLOAD_TASK_STACK, NULL,
                    UPLOAD_TASK_PRIO, NULL) != pdPASS) {
        return ESP_FAIL;
    }
    return ESP_OK;
}
