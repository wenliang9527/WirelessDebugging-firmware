/**
 * @file dap_cmd.c
 * @brief 产品化 P1:USB(COM18)命令通道
 *
 * 背景:
 *   IDF 的"次 console"模式只有单向输出,收不到任何输入。本项目已把主 console
 *   切到 USB-Serial/JTAG(sdkconfig: CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y),
 *   这样同一个 USB 口既能输出 IDF 日志,又能接收 PC 发来的命令。
 *
 * 线格式:
 *   PC -> 板:  "@命令 参数...\n"           (ASCII,行尾 \n,\r 被忽略)
 *   板 -> PC:  "@OK ..." / "@ERR <码> <说明>" / "@DATA ..." / "@EVT ..."
 *   板端 IDF 日志不带 '@' 前缀,PC 侧一律当噪声丢弃。
 *
 * 并发:
 *   单任务串行处理。命令处理函数尽量短;耗时操作(如 @CONNECT 扫描)由
 *   wifi 侧配合,这里只负责解析与回帧。
 */
#include "sdkconfig.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdint.h>
#include <ctype.h>
#include <sys/param.h>

#include "main/dap_cmd.h"
#include "main/dap_version.h"
#include "main/wifi_configuration.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"

#if defined CONFIG_IDF_TARGET_ESP32S3 || defined CONFIG_IDF_TARGET_ESP32C3
    #include "driver/usb_serial_jtag.h"
    #include "esp_vfs_dev.h"
    #define DAP_CMD_USE_USJ 1
#endif

static const char *TAG = "dap.cmd";

#define CMD_TASK_STACK   4096
#define CMD_TASK_PRIO    12
#define CMD_LINE_MAX     256
#define CMD_RX_BUF_SIZE  1024
#define CMD_TX_BUF_SIZE  1024

static volatile bool s_cmd_ready = false;
/* WiFi 基座是否已完成初始化。wifi_init() 在 STA 模式下会阻塞到拿到 IP,
 * 在这之前绝不能让 @MODE 改动 WiFi 状态,否则会与 wifi_init() 的启动流程
 * 竞争(实测表现为 AP 被启动两次、boot mode 日志重复)。 */
static volatile bool s_wifi_ready = false;

void dap_cmd_set_wifi_ready(bool ready) {
    s_wifi_ready = ready;
}

/* ------------------------------------------------------------------ */
/* 输出:统一走 stdout(主 console 已是 USB-Serial/JTAG)                */
/* ------------------------------------------------------------------ */

static void cmd_write(const char *buf, size_t len) {
    if (len == 0) {
        return;
    }
    /* 直接写驱动,避免 stdout 缓冲/换行转换干扰协议帧 */
#if DAP_CMD_USE_USJ
    usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(100));
#else
    fwrite(buf, 1, len, stdout);
    fflush(stdout);
#endif
}

void dap_cmd_reply(const char *fmt, ...) {
    char line[CMD_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line) - 2, fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if (n > (int)sizeof(line) - 3) {
        n = sizeof(line) - 3;
    }
    line[n++] = '\n';
    cmd_write(line, (size_t)n);
}

void dap_cmd_ok(const char *fmt, ...) {
    char body[CMD_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    if (body[0]) {
        dap_cmd_reply("@OK %s", body);
    } else {
        dap_cmd_reply("@OK");
    }
}

void dap_cmd_err(int code, const char *msg) {
    dap_cmd_reply("@ERR %d %s", code, msg ? msg : "error");
}

void dap_cmd_data(const char *fmt, ...) {
    char body[CMD_LINE_MAX];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    dap_cmd_reply("@DATA %s", body);
}

/* ------------------------------------------------------------------ */
/* 工具                                                                */
/* ------------------------------------------------------------------ */

/** 十六进制字符转数值,-1 表示非法 */
static int hexval(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/**
 * URL 百分号解码(原地)。
 * SSID/密码可能含空格、中文、'&'、'=' 等,统一由 PC 侧编码后再发。
 */
static void url_decode(char *s) {
    char *dst = s;
    for (char *src = s; *src; src++) {
        if (*src == '%' && hexval(src[1]) >= 0 && hexval(src[2]) >= 0) {
            *dst++ = (char)((hexval(src[1]) << 4) | hexval(src[2]));
            src += 2;
        } else if (*src == '+') {
            *dst++ = ' ';
        } else {
            *dst++ = *src;
        }
    }
    *dst = '\0';
}

/**
 * 从 "key=value&key=value" 里取某个 key 的值(已解码)。
 * 返回 false 表示不存在。
 */
static bool param_get(const char *kv, const char *key, char *out, size_t out_len) {
    size_t klen = strlen(key);
    const char *p = kv;
    while (p && *p) {
        const char *amp = strchr(p, '&');
        size_t seg = amp ? (size_t)(amp - p) : strlen(p);
        if (seg > klen && strncmp(p, key, klen) == 0 && p[klen] == '=') {
            size_t vlen = seg - klen - 1;
            if (vlen >= out_len) {
                vlen = out_len - 1;
            }
            memcpy(out, p + klen + 1, vlen);
            out[vlen] = '\0';
            url_decode(out);
            return true;
        }
        p = amp ? amp + 1 : NULL;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* 命令实现                                                            */
/* ------------------------------------------------------------------ */

static void cmd_ping(const char *args) {
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    dap_cmd_ok("pong fw=%s mac=%02X:%02X:%02X:%02X:%02X:%02X mode=%s",
               DAP_FW_VERSION, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
               dap_cmd_mode_name());
}

static void cmd_status(const char *args) {
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) != ESP_OK) {
        dap_cmd_err(DAP_ERR_INTERNAL, "wifi_get_mode_failed");
        return;
    }

    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA) {
        wifi_config_t cfg;
        char ssid[33] = "(none)";
        if (esp_wifi_get_config(WIFI_IF_STA, &cfg) == ESP_OK && cfg.sta.ssid[0]) {
            memcpy(ssid, cfg.sta.ssid, sizeof(cfg.sta.ssid));
            ssid[sizeof(cfg.sta.ssid)] = '\0';
        }
        tcpip_adapter_ip_info_t ip;
        char ipstr[16] = "0.0.0.0";
        if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip) == ESP_OK) {
            snprintf(ipstr, sizeof(ipstr), IPSTR, IP2STR(&ip.ip));
        }
        wifi_ap_record_t ap;
        int rssi = 0, ch = 0;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            rssi = ap.rssi;
            ch = ap.primary;
        }
        dap_cmd_ok("mode=%s ssid=%s ip=%s rssi=%d ch=%d",
                   dap_cmd_mode_name(), ssid, ipstr, rssi, ch);
    } else {
        tcpip_adapter_ip_info_t ip;
        char ipstr[16] = "0.0.0.0";
        if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip) == ESP_OK) {
            snprintf(ipstr, sizeof(ipstr), IPSTR, IP2STR(&ip.ip));
        }
        dap_cmd_ok("mode=%s ip=%s ssid=%s", dap_cmd_mode_name(), ipstr,
                   dap_cmd_ap_ssid());
    }
}

static void cmd_reboot(const char *args) {
    dap_cmd_ok("rebooting");
    vTaskDelay(pdMS_TO_TICKS(100));  /* 让响应先发出去 */
    esp_restart();
}

static void cmd_mode(const char *args) {
    char want[8] = {0};
    if (!args || sscanf(args, "%7s", want) != 1) {
        dap_cmd_err(DAP_ERR_BADARG, "usage: @MODE STA|AP");
        return;
    }
    for (char *p = want; *p; p++) {
        *p = (char)toupper((unsigned char)*p);
    }

    dap_mode_t target;
    if (strcmp(want, "STA") == 0) {
        target = DAP_MODE_STA;
    } else if (strcmp(want, "AP") == 0) {
        target = DAP_MODE_AP;
    } else {
        dap_cmd_err(DAP_ERR_BADARG, "mode_must_be_STA_or_AP");
        return;
    }

    if (!s_wifi_ready) {
        /* wifi_init() 仍在跑(STA 模式会阻塞等 IP),此时切换会与其启动
         * 流程竞争。等它落定再切。 */
        dap_cmd_err(DAP_ERR_STATE, "wifi_still_starting");
        return;
    }

    esp_err_t err = dap_cmd_set_mode(target);
    if (err != ESP_OK) {
        dap_cmd_err(DAP_ERR_INTERNAL, esp_err_to_name(err));
        return;
    }
    if (target == DAP_MODE_AP) {
        dap_cmd_ok("mode=AP ip=192.168.4.1 ssid=%s", dap_cmd_ap_ssid());
    } else {
        dap_cmd_ok("mode=STA");
    }
}

/* ------------------------------------------------------------------ */
/* 分发                                                                */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *name;
    void (*fn)(const char *args);
} cmd_entry_t;

static const cmd_entry_t s_cmds[] = {
    { "PING",    cmd_ping   },
    { "STATUS",  cmd_status },
    { "MODE",    cmd_mode   },
    { "REBOOT",  cmd_reboot },
};

static void dispatch_line(char *line) {
    /* 去掉行尾空白 */
    size_t n = strlen(line);
    while (n > 0 && isspace((unsigned char)line[n - 1])) {
        line[--n] = '\0';
    }

    /* 跳过以 '@' 开头的标记 */
    char *p = line;
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    if (*p != '@') {
        return;  /* 不是协议行,忽略(可能是终端里误敲的字符) */
    }
    p++;

    /* 取命令名 */
    char *name = p;
    while (*name && !isspace((unsigned char)*name)) {
        name++;
    }
    char saved = *name;
    *name = '\0';
    char *args = NULL;
    if (saved) {
        *name = '\0';
        args = name + 1;
        while (*args && isspace((unsigned char)*args)) {
            args++;
        }
    }
    if (!*p) {
        return;
    }

    /* 命令名大小写不敏感 */
    char upper[16];
    size_t i = 0;
    for (; p[i] && i < sizeof(upper) - 1; i++) {
        upper[i] = (char)toupper((unsigned char)p[i]);
    }
    upper[i] = '\0';

    for (size_t k = 0; k < sizeof(s_cmds) / sizeof(s_cmds[0]); k++) {
        if (strcmp(upper, s_cmds[k].name) == 0) {
            s_cmds[k].fn(args);
            return;
        }
    }
    dap_cmd_err(DAP_ERR_UNKNOWN, upper);
}

/* ------------------------------------------------------------------ */
/* 接收任务                                                            */
/* ------------------------------------------------------------------ */

static void cmd_task(void *arg) {
    char line[CMD_LINE_MAX];
    size_t len = 0;
    uint8_t chunk[64];

    while (1) {
        int n = usb_serial_jtag_read_bytes(chunk, sizeof(chunk), pdMS_TO_TICKS(50));
        if (n <= 0) {
            continue;
        }
        for (int i = 0; i < n; i++) {
            char c = (char)chunk[i];
            if (c == '\n' || c == '\r') {
                if (len > 0) {
                    line[len] = '\0';
                    dispatch_line(line);
                    len = 0;
                }
            } else if (len < CMD_LINE_MAX - 1) {
                line[len++] = c;
            } else {
                /* 超长行:丢弃并报错,避免无界增长 */
                len = 0;
                dap_cmd_err(DAP_ERR_BADARG, "line_too_long");
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* 初始化                                                              */
/* ------------------------------------------------------------------ */

esp_err_t dap_cmd_init(void) {
#if DAP_CMD_USE_USJ
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = CMD_RX_BUF_SIZE;
    cfg.tx_buffer_size = CMD_TX_BUF_SIZE;

    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        /* 已装过(理论上不该发生):继续用,只提示 */
        ESP_LOGW(TAG, "USJ driver already installed");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "USJ driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 把 vfs 的读写切到驱动路径,否则 stdout 绕过驱动、与命令写冲突 */
    esp_vfs_usb_serial_jtag_use_driver();
#else
    ESP_LOGE(TAG, "command channel requires ESP32-S3 or C3");
    return ESP_ERR_NOT_SUPPORTED;
#endif

    s_cmd_ready = true;
    BaseType_t r = xTaskCreate(cmd_task, "dap_cmd", CMD_TASK_STACK, NULL,
                               CMD_TASK_PRIO, NULL);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "command channel ready on USB-Serial/JTAG (fw=%s)", DAP_FW_VERSION);
    return ESP_OK;
}

bool dap_cmd_is_ready(void) {
    return s_cmd_ready;
}
