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
 * 命令一览(2026-10):
 *   @PING @STATUS @SCAN @CONNECT @LIST @DEL @PRIO @PIN @MODE @LED
 *   @FACTORY @REBOOT
 * 离线烧录(2026-11,第二阶段):
 *   @FLIST @FSTAT @FDEL @FSEL @FNAME @FPUT(@FLASH/@FPROG 随烧录引擎上线)
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
#include "main/dap_config.h"
#include "main/dap_led.h"
#include "main/dap_store.h"
#include "main/dap_version.h"
#include "main/wifi_configuration.h"
#include "main/wifi_handle.h"

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

/*
 * 为什么用 fwrite 而不是 usb_serial_jtag_write_bytes():
 *
 * IDF 4.4 的 USB-Serial/JTAG 有两条互不相让的 TX 路径,共用同一个硬件 FIFO:
 *   - VFS(日志,esp_rom_printf / stdout):usb_serial_jtag_tx_char() 直写 FIFO,
 *     FIFO 满 50ms 后主动丢字节,永不阻塞。
 *   - 驱动(usb_serial_jtag_write_bytes):写 ring buffer,靠 SERIAL_IN_EMPTY
 *     中断搬到 FIFO。而该死的中断只在 usb_serial_jtag_ll_txfifo_writable()
 *     为真时才搬数据。
 *
 * 日志一多就把 FIFO 占满,导致 ISR 永远拿不到可写时机,ring buffer 永远排不空。
 * xRingbufferSend 超时后静默丢数据(返回值都没检查),主机侧则看到 WriteFile
 * 永久挂起——实测表现就是"烧录后能用几秒,之后 @PING 再也发不进去"。
 *
 * 结论:命令输出必须和日志走同一条 VFS 路径,宁可像日志一样被丢弃(有 @ 前缀
 * 和 PC 侧重试兜底),也不能把整条 TX 拖死。
 */
static void cmd_write(const char *buf, size_t len) {
    if (len == 0) {
        return;
    }
    fwrite(buf, 1, len, stdout);
    fflush(stdout);
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
        /* prov=1 标记未配网态:一条已保存网络都没有。PC 工具据此把
         * "STA 未连接"的提示换成"等待配网"引导,而不是误导用户去查路由器。 */
        dap_cmd_ok("mode=%s ssid=%s ip=%s rssi=%d ch=%d prov=%d",
                   dap_cmd_mode_name(), ssid, ipstr, rssi, ch,
                   (dap_config_count() == 0) ? 1 : 0);
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

/* ------------------------------------------------------------------ */
/* P2:扫描 / 连接 / 凭据管理                                           */
/* ------------------------------------------------------------------ */

/** authmode 转可读文本,PC 侧直接显示 */
static const char *auth_str(uint8_t m) {
    switch (m) {
    case WIFI_AUTH_OPEN:            return "OPEN";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-ENT";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    default:                        return "UNKNOWN";
    }
}

static void cmd_scan(const char *args) {
    dap_scan_result_t *res = calloc(24, sizeof(dap_scan_result_t));
    if (!res) {
        dap_cmd_err(DAP_ERR_INTERNAL, "no_mem");
        return;
    }
    int found = 0;
    esp_err_t err = dap_wifi_scan(res, 24, &found);
    if (err == ESP_ERR_NOT_SUPPORTED) {
        /* 纯 AP 模式下 IDF 驱动不支持扫描,这是硬约束 */
        dap_cmd_err(DAP_ERR_MODE_AP, "scan_requires_STA_mode");
        free(res);
        return;
    }
    if (err != ESP_OK) {
        dap_cmd_err(DAP_ERR_INTERNAL, esp_err_to_name(err));
        free(res);
        return;
    }
    for (int i = 0; i < found; i++) {
        dap_cmd_data("idx=%d&ssid=%s&rssi=%d&auth=%s&ch=%d",
                     i, res[i].ssid, (int)res[i].rssi,
                     auth_str(res[i].authmode), (int)res[i].channel);
    }
    dap_cmd_ok("scan count=%d", found);
    free(res);
}

static void cmd_connect(const char *args) {
    char ssid[DAP_CFG_SSID_LEN] = {0};
    char pass[DAP_CFG_PASS_LEN] = {0};

    if (!args || !param_get(args, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
        dap_cmd_err(DAP_ERR_BADARG, "missing_ssid");
        return;
    }
    if (!param_get(args, "pass", pass, sizeof(pass))) {
        pass[0] = '\0';  /* 允许开放网络 */
    }

    if (dap_wifi_busy()) {
        dap_cmd_err(DAP_ERR_BUSY, "connect_in_progress");
        return;
    }

    esp_err_t err = dap_wifi_connect_manual(ssid, pass);
    if (err == ESP_ERR_INVALID_STATE) {
        dap_cmd_err(DAP_ERR_MODE_AP, "connect_requires_STA_mode");
        return;
    }
    if (err != ESP_OK) {
        /* 这里是"启动连接"就失败(而非连接过程失败),用 500 段错误码,
         * 避免把 ESP_ERR_xxx 的原始数字(如 8)当成 WiFi 原因码误导 PC。 */
        dap_cmd_err(DAP_ERR_INTERNAL, esp_err_to_name(err));
        return;
    }

    /* 轮询等待结果。命令处理是串行的,这里阻塞不影响其他命令(它们会排队),
     * 但会挡住 @PING —— 所以超时给足却不过长。 */
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(25000);
    while (xTaskGetTickCount() < deadline) {
        bool done = false, ok = false;
        int reason = 0;
        dap_wifi_get_manual_result(&done, &ok, &reason);
        if (done) {
            if (ok) {
                tcpip_adapter_ip_info_t ip;
                char ipstr[16] = "0.0.0.0";
                if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip) == ESP_OK) {
                    snprintf(ipstr, sizeof(ipstr), IPSTR, IP2STR(&ip.ip));
                }
                dap_cmd_ok("connected ssid=%s ip=%s", ssid, ipstr);
            } else {
                /* 原因码直接透传 IDF 值,PC 侧据此给不同提示 */
                const char *msg = "connect_failed";
                switch (reason) {
                case WIFI_REASON_NO_AP_FOUND:           msg = "NO_AP_FOUND"; break;
                case WIFI_REASON_AUTH_FAIL:             msg = "AUTH_FAIL"; break;
                case WIFI_REASON_AUTH_EXPIRE:           msg = "AUTH_EXPIRE"; break;
                case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: msg = "HANDSHAKE_TIMEOUT"; break;
                case WIFI_REASON_HANDSHAKE_TIMEOUT:     msg = "HANDSHAKE_TIMEOUT"; break;
                case WIFI_REASON_CONNECTION_FAIL:       msg = "CONNECTION_FAIL"; break;
                default: break;
                }
                dap_cmd_err(reason > 0 ? reason : DAP_ERR_TIMEOUT, msg);
            }
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    dap_cmd_err(DAP_ERR_TIMEOUT, "TIMEOUT");
}

static void cmd_list(const char *args) {
    int n = dap_config_count();
    for (int i = 0; i < n; i++) {
        dap_cfg_ap_t ap;
        if (!dap_config_get(i, &ap)) {
            continue;
        }
        /* 密码打码:只报是否存在,不泄露到日志/终端。pin= 固定优先标志。 */
        dap_cmd_data("idx=%d&ssid=%s&pass=%s&pin=%d",
                     i, ap.ssid, ap.password[0] ? "***" : "(open)", ap.pin ? 1 : 0);
    }
    dap_cmd_ok("count=%d source=%s", n,
               dap_config_using_factory() ? "factory" : "nvs");
}

static void cmd_del(const char *args) {
    char v[12] = {0};
    if (!args || !param_get(args, "idx", v, sizeof(v))) {
        dap_cmd_err(DAP_ERR_BADARG, "missing_idx");
        return;
    }
    int idx = atoi(v);
    esp_err_t err = dap_config_del(idx);
    if (err != ESP_OK) {
        dap_cmd_err(DAP_ERR_BADARG, "idx_out_of_range");
        return;
    }
    dap_cmd_ok("deleted idx=%d remaining=%d", idx, dap_config_count());
}

/**
 * @PRIO idx=N    把第 N 条移到最前(开机时最先尝试)
 * @PRIO ssid=xxx 按名字移动(中文 SSID 用这个,索引会随着增删变化)
 * 不带参数时报当前顺序。
 *
 * 为什么需要这个命令:@CONNECT 成功会自动把目标网络置顶,所以"最近连过的"
 * 永远排第一。用户想让家里某个网络固定优先,光连一次不够 —— 下次手动连
 * 别的网络又会被顶下去。这里提供显式排序,和自动置顶的语义区分开。
 */
static void cmd_prio(const char *args) {
    int n = dap_config_count();
    if (n <= 0) {
        dap_cmd_err(DAP_ERR_STATE, "no_saved_networks");
        return;
    }

    char sv[12] = {0};
    int idx = -1;
    if (args && param_get(args, "idx", sv, sizeof(sv))) {
        idx = atoi(sv);
    } else if (args) {
        /* param_get 内部已做 URL 解码,这里拿到的就是明文 SSID */
        char name[DAP_CFG_SSID_LEN * 3] = {0};
        if (param_get(args, "ssid", name, sizeof(name))) {
            idx = dap_config_index_of(name);
            if (idx < 0) {
                dap_cmd_err(DAP_ERR_UNKNOWN, "ssid_not_in_list");
                return;
            }
        }
    }

    if (idx < 0) {
        /* 无参数:回报当前顺序,顺带自报哪个是首位 */
        dap_cmd_err(DAP_ERR_BADARG, "usage: @PRIO idx=N | ssid=<enc>");
        return;
    }

    esp_err_t err = dap_config_prioritize(idx);
    if (err != ESP_OK) {
        dap_cmd_err(DAP_ERR_BADARG, "idx_out_of_range");
        return;
    }
    dap_cfg_ap_t top;
    if (dap_config_get(0, &top)) {
        dap_cmd_ok("prioritized ssid=%s idx=0", top.ssid);
    } else {
        dap_cmd_ok("prioritized idx=0");
    }
}

/**
 * @PIN idx=N on|off   给第 N 条打/摘"固定优先"标记(粘性,存 NVS)
 *
 * 与 @PRIO 的语义分界:@PRIO 是一次性物理置顶,会被下次 @CONNECT 的自动
 * 置顶顶掉;@PIN 是粘性标志,连接轮转时 pinned 条目永远排在未 pinned
 * 之前,与物理顺序无关。GUI 的 📌 勾选走这条命令。
 */
static void cmd_pin(const char *args) {
    char sv[12] = {0};
    if (!args || !param_get(args, "idx", sv, sizeof(sv))) {
        dap_cmd_err(DAP_ERR_BADARG, "missing_idx");
        return;
    }
    /* on/off 是裸 token 不是 key=value,param_get 取不到,单独扫。
     * 格式 "@PIN idx=N on|off":跳过 "idx=..." 段后剩下的就是开关词。 */
    const char *tok = strstr(args, "on");
    if (!tok || (tok > args && tok[-1] != ' ' && tok[-1] != '&')) {
        tok = strstr(args, "off");
    }
    if (!tok) {
        dap_cmd_err(DAP_ERR_BADARG, "usage: @PIN idx=N on|off");
        return;
    }
    int idx = atoi(sv);
    bool on;
    if (!strncmp(tok, "on", 2)) {
        on = true;
    } else if (!strncmp(tok, "off", 3)) {
        on = false;
    } else {
        dap_cmd_err(DAP_ERR_BADARG, "usage: @PIN idx=N on|off");
        return;
    }
    esp_err_t err = dap_config_set_pin(idx, on);
    if (err != ESP_OK) {
        dap_cmd_err(DAP_ERR_BADARG, "idx_out_of_range");
        return;
    }
    dap_cmd_ok("pin idx=%d %s", idx, on ? "on" : "off");
}

static void cmd_factory(const char *args) {
    dap_cmd_ok("rebooting");
    vTaskDelay(pdMS_TO_TICKS(100));
    if (dap_config_factory_reset() != ESP_OK) {
        ESP_LOGW(TAG, "factory reset nvs error");
    }
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

/**
 * @LED [pct]
 *
 * 不带参数:查询当前亮度。
 * 带参数:设置亮度 1..100 并写入 NVS(重启后保持)。
 *
 * 加这条命令是为了"灯太亮"这类主观偏好能当场调,不必为改一个数字重烧固件。
 */
static void cmd_led(const char *args) {
    if (!dap_led_available()) {
        dap_cmd_err(DAP_ERR_INTERNAL, "led_unavailable");
        return;
    }

    char v[8] = {0};
    if (!args || sscanf(args, "%7s", v) != 1) {
        dap_cmd_ok("led pct=%d", (int)dap_led_get_brightness());
        return;
    }

    char *end = NULL;
    long pct = strtol(v, &end, 10);
    if (end == v || (end && *end != '\0')) {
        dap_cmd_err(DAP_ERR_BADARG, "pct_must_be_number");
        return;
    }
    if (pct < DAP_CFG_LED_MIN_PCT || pct > DAP_CFG_LED_MAX_PCT) {
        dap_cmd_err(DAP_ERR_BADARG, "pct_out_of_range_1_100");
        return;
    }

    dap_led_set_brightness((uint8_t)pct);
    esp_err_t err = dap_config_set_led_brightness((uint8_t)pct);
    if (err != ESP_OK) {
        /* 亮度已经生效了,只是没存下来 —— 如实告知,别报成功 */
        dap_cmd_err(DAP_ERR_INTERNAL, "applied_but_not_saved");
        return;
    }
    dap_cmd_ok("led pct=%d saved", (int)pct);
}

/* ------------------------------------------------------------------ */
/* AP 热点信息                                                         */
/* ------------------------------------------------------------------ */
/**
 * @AP          读当前 AP 参数(SSID + 密码)
 * @AP REGEN    重新生成密码(不动 SSID),下次切模式/重启生效
 *
 * 为什么单独开一条命令而不是塞进 @STATUS:
 *   密码是敏感信息,只有**用户主动要看**时才该出现在输出里。@STATUS 是
 *   每次连上都会自动发的,把密码塞进去等于让它天天躺在日志文件里。
 *
 * 本命令只在 USB 命令通道上可达(固件不暴露 TCP 版命令通道),所以"知道
 * 密码"的门槛 = 物理接触板子 + 有 USB 线,这正是我们想要的门槛。
 */
static void cmd_ap(const char *args) {
    char sub[16] = {0};
    if (args) {
        sscanf(args, "%15s", sub);
    }

    if (sub[0] != '\0') {
        /* 项目里没有 <strings.h>,其他地方的大小写折叠都是手写 toupper
         * (见 dispatch_line)。跟着用同一套,别为一个比较引入新头文件。 */
        for (char *q = sub; *q; q++) {
            *q = (char)toupper((unsigned char)*q);
        }
        if (strcmp(sub, "REGEN") != 0) {
            dap_cmd_err(DAP_ERR_BADARG, "usage: @AP or @AP REGEN");
            return;
        }
        dap_config_regen_ap_password();
        /* 新密码要等下一次 apply_ap_mode 才生效(切模式或重启)。不在这里
         * 自动切 —— 切模式会踢掉当前连接,而用户可能只是想先把密码换好。
         * note= 字段让 PC 侧能把这句话原样转述,不必自己猜。 */
        dap_cmd_ok("ap ssid=%s pass=%s note=applies_on_next_mode_switch",
                   dap_cmd_ap_ssid(), dap_config_get_ap_password());
        return;
    }

    dap_cmd_ok("ap ssid=%s pass=%s", dap_cmd_ap_ssid(),
               dap_config_get_ap_password());
}

/* ------------------------------------------------------------------ */
/* 离线烧录:镜像文件命令(第二阶段 D2)                                 */
/*                                                                     */
/* 镜像存 storage 分区(dap_store),索引在 NVS。命令面只做管理与      */
/* USB 兜底上传;烧录触发(@FLASH/@FPROG)随 D4 引擎上线。             */
/* ------------------------------------------------------------------ */

/** URL 编码到 dst(与 param_get 的解码互逆,name 里的空格/中文/&/= 都安全) */
static void url_encode_to(char *dst, size_t dstn, const char *src) {
    size_t o = 0;
    for (const unsigned char *p = (const unsigned char *)src; *p && o + 4 <= dstn; p++) {
        unsigned char c = *p;
        if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            dst[o++] = (char)c;
        } else {
            o += (size_t)snprintf(dst + o, dstn - o, "%%%02X", c);
        }
    }
    dst[o] = '\0';
}

/** 元数据一行 @DATA(name 走 URL 编码,PC 侧解码) */
static void fmeta_row(int slot, const dap_store_meta_t *m) {
    char enc[DAP_STORE_NAME_LEN * 3];
    url_encode_to(enc, sizeof(enc), m->name);
    dap_cmd_data("slot=%d name=%s size=%u crc=%08" PRIX32 " family=%s"
                 " addr=0x%08" PRIX32 " seq=%u cur=%d",
                 slot, enc, (unsigned)m->size, m->crc32, m->family,
                 m->addr, (unsigned)m->seq, slot == dap_store_current());
}

/** 取 slot=N 参数并做范围检查 */
static bool slot_arg(const char *args, int *out) {
    char v[8] = {0};
    if (!args || !param_get(args, "slot", v, sizeof(v))) return false;
    int n = atoi(v);
    if (n < 0 || n >= DAP_STORE_MAX_SLOT) return false;
    *out = n;
    return true;
}

/** @FLIST — 列出全部镜像(每槽一行 @DATA,再一行 @OK 汇总) */
static void cmd_flist(const char *args) {
    if (!dap_store_mounted()) {
        dap_cmd_err(DAP_ERR_NOSTORE, "storage_not_mounted");
        return;
    }
    int n = 0;
    for (int i = 0; i < DAP_STORE_MAX_SLOT; i++) {
        dap_store_meta_t m;
        if (dap_store_get(i, &m)) {
            fmeta_row(i, &m);
            n++;
        }
    }
    dap_cmd_ok("files=%d cur=%d", n, dap_store_current());
}

/** @FSTAT slot=N — 单槽详情 */
static void cmd_fstat(const char *args) {
    if (!dap_store_mounted()) {
        dap_cmd_err(DAP_ERR_NOSTORE, "storage_not_mounted");
        return;
    }
    int slot;
    if (!slot_arg(args, &slot)) {
        dap_cmd_err(DAP_ERR_BADARG, "usage: @FSTAT slot=N");
        return;
    }
    dap_store_meta_t m;
    if (!dap_store_get(slot, &m)) {
        dap_cmd_err(DAP_ERR_SLOT_EMPTY, "slot_empty");
        return;
    }
    fmeta_row(slot, &m);
    dap_cmd_ok("slot=%d", slot);
}

/** @FDEL slot=N — 删除镜像(删的是当前槽时由存储层自动回落) */
static void cmd_fdel(const char *args) {
    if (!dap_store_mounted()) {
        dap_cmd_err(DAP_ERR_NOSTORE, "storage_not_mounted");
        return;
    }
    int slot;
    if (!slot_arg(args, &slot)) {
        dap_cmd_err(DAP_ERR_BADARG, "usage: @FDEL slot=N");
        return;
    }
    esp_err_t err = dap_store_del(slot);
    if (err == ESP_ERR_NOT_FOUND) {
        dap_cmd_err(DAP_ERR_SLOT_EMPTY, "slot_empty");
        return;
    }
    if (err != ESP_OK) {
        dap_cmd_err(DAP_ERR_INTERNAL, "del_failed");
        return;
    }
    dap_cmd_ok("deleted slot=%d cur=%d", slot, dap_store_current());
}

/** @FSEL slot=N — 设为当前烧录文件 */
static void cmd_fsel(const char *args) {
    if (!dap_store_mounted()) {
        dap_cmd_err(DAP_ERR_NOSTORE, "storage_not_mounted");
        return;
    }
    int slot;
    if (!slot_arg(args, &slot)) {
        dap_cmd_err(DAP_ERR_BADARG, "usage: @FSEL slot=N");
        return;
    }
    esp_err_t err = dap_store_set_current(slot);
    if (err == ESP_ERR_NOT_FOUND) {
        dap_cmd_err(DAP_ERR_SLOT_EMPTY, "slot_empty");
        return;
    }
    if (err != ESP_OK) {
        dap_cmd_err(DAP_ERR_INTERNAL, "sel_failed");
        return;
    }
    dap_cmd_ok("current slot=%d", slot);
}

/** @FNAME slot=N name=<enc> — 改显示名 */
static void cmd_fname(const char *args) {
    if (!dap_store_mounted()) {
        dap_cmd_err(DAP_ERR_NOSTORE, "storage_not_mounted");
        return;
    }
    int slot;
    char name[DAP_STORE_NAME_LEN * 3] = {0};
    if (!slot_arg(args, &slot) || !param_get(args, "name", name, sizeof(name))) {
        dap_cmd_err(DAP_ERR_BADARG, "usage: @FNAME slot=N name=<enc>");
        return;
    }
    if (name[0] == '\0' || strlen(name) >= DAP_STORE_NAME_LEN) {
        dap_cmd_err(DAP_ERR_BADARG, "name_empty_or_too_long");
        return;
    }
    if (!dap_store_get(slot, NULL)) {
        dap_cmd_err(DAP_ERR_SLOT_EMPTY, "slot_empty");
        return;
    }
    if (dap_store_rename(slot, name) != ESP_OK) {
        dap_cmd_err(DAP_ERR_INTERNAL, "rename_failed");
        return;
    }
    dap_cmd_ok("renamed slot=%d", slot);
}

/*
 * @FPUT — USB 兜底上传(无 WiFi 时的小文件通道)。
 *
 *   @FPUT begin slot=N name=<enc> size=B crc=XXXXXXXX family=PY32 addr=0x08000000
 *   @FPUT data <hex...>     (每行 ≤116 字节,受 CMD_LINE_MAX=256 限制)
 *   @FPUT end               (板端复算 CRC,过了才落 NVS)
 *   @FPUT abort             (放弃)
 *
 * 与 TCP:1235(D3)共用 dap_store 的单会话锁,两边谁先 begin 谁占住,
 * 后来的 begin 会把前一个未完成的会话作废(异常中断后 PC 重试无需重启板)。
 */
static void cmd_fput(const char *args) {
    if (!dap_store_mounted()) {
        dap_cmd_err(DAP_ERR_NOSTORE, "storage_not_mounted");
        return;
    }
    if (!args || !*args) {
        dap_cmd_err(DAP_ERR_BADARG, "usage: @FPUT begin|data|end|abort ...");
        return;
    }

    char sub[8] = {0};
    {
        size_t i = 0;
        while (args[i] && !isspace((unsigned char)args[i]) && i < sizeof(sub) - 1) {
            sub[i] = args[i];
            i++;
        }
        sub[i] = '\0';
    }

    if (strcmp(sub, "begin") == 0) {
        dap_store_meta_t m = {0};
        char v[DAP_STORE_NAME_LEN * 3] = {0};
        int slot;
        uint32_t size = 0, crc = 0;
        if (!slot_arg(args, &slot) ||
            !param_get(args, "name", v, sizeof(v)) || v[0] == '\0' ||
            strlen(v) >= DAP_STORE_NAME_LEN) {
            dap_cmd_err(DAP_ERR_BADARG, "need slot_and_name");
            return;
        }
        strlcpy(m.name, v, sizeof(m.name));
        if (param_get(args, "size", v, sizeof(v)))  size = (uint32_t)strtoul(v, NULL, 10);
        if (param_get(args, "crc", v, sizeof(v)))   crc  = (uint32_t)strtoul(v, NULL, 16);
        if (param_get(args, "family", v, sizeof(v))) strlcpy(m.family, v, sizeof(m.family));
        if (param_get(args, "addr", v, sizeof(v)))  m.addr = (uint32_t)strtoul(v, NULL, 0);
        if (size == 0 || crc == 0 || m.family[0] == '\0') {
            dap_cmd_err(DAP_ERR_BADARG, "need size crc family");
            return;
        }
        m.size = size;
        m.crc32 = crc;

        dap_store_abort(); /* 若有残留会话(上次异常断),作废重来 */
        esp_err_t err = dap_store_begin(slot, &m);
        if (err == ESP_ERR_INVALID_ARG) {
            dap_cmd_err(DAP_ERR_BADARG, "bad_meta_or_slot");
        } else if (err != ESP_OK) {
            dap_cmd_err(DAP_ERR_INTERNAL, "begin_failed");
        } else {
            dap_cmd_ok("begin slot=%d size=%u", slot, (unsigned)size);
        }
        return;
    }

    if (strcmp(sub, "data") == 0) {
        const char *hex = args + strlen(sub);
        while (*hex && isspace((unsigned char)*hex)) hex++;
        size_t hlen = strlen(hex);
        if (hlen == 0 || (hlen & 1)) {
            dap_cmd_err(DAP_ERR_BADARG, "hex_len_must_be_even");
            return;
        }
        if (hlen / 2 > 128) {
            dap_cmd_err(DAP_ERR_BADARG, "max_128_bytes_per_line");
            return;
        }
        uint8_t buf[128];
        for (size_t i = 0; i < hlen; i += 2) {
            int hi = hexval(hex[i]), lo = hexval(hex[i + 1]);
            if (hi < 0 || lo < 0) {
                dap_cmd_err(DAP_ERR_BADARG, "bad_hex_char");
                return;
            }
            buf[i / 2] = (uint8_t)((hi << 4) | lo);
        }
        esp_err_t err = dap_store_write(buf, hlen / 2);
        if (err == ESP_ERR_INVALID_STATE) {
            dap_cmd_err(DAP_ERR_STATE, "no_upload_in_progress");
        } else if (err == ESP_ERR_INVALID_SIZE) {
            dap_store_abort(); /* 超长即会话非法,立即作废 */
            dap_cmd_err(DAP_ERR_BADARG, "size_exceeds_declared");
        } else if (err != ESP_OK) {
            dap_store_abort();
            dap_cmd_err(DAP_ERR_INTERNAL, "write_failed");
        } else {
            dap_cmd_ok("written=%u", (unsigned)(hlen / 2));
        }
        return;
    }

    if (strcmp(sub, "end") == 0) {
        esp_err_t err = dap_store_finish();
        if (err == ESP_ERR_INVALID_STATE) {
            dap_cmd_err(DAP_ERR_STATE, "no_upload_in_progress");
        } else if (err == ESP_ERR_INVALID_SIZE) {
            dap_cmd_err(DAP_ERR_BADARG, "size_mismatch");
        } else if (err == ESP_ERR_INVALID_CRC) {
            dap_cmd_err(DAP_ERR_CRC, "crc_mismatch");
        } else if (err != ESP_OK) {
            dap_cmd_err(DAP_ERR_INTERNAL, "finish_failed");
        } else {
            dap_cmd_ok("saved crc_ok=1");
        }
        return;
    }

    if (strcmp(sub, "abort") == 0) {
        dap_store_abort();
        dap_cmd_ok("aborted");
        return;
    }

    dap_cmd_err(DAP_ERR_BADARG, "usage: @FPUT begin|data|end|abort ...");
}

/* ------------------------------------------------------------------ */
/* 分发                                                                */
/* ------------------------------------------------------------------ */

typedef struct {
    const char *name;
    void (*fn)(const char *args);
} cmd_entry_t;

static const cmd_entry_t s_cmds[] = {
    { "PING",    cmd_ping    },
    { "STATUS",  cmd_status  },
    { "SCAN",    cmd_scan    },
    { "CONNECT", cmd_connect },
    { "LIST",    cmd_list    },
    { "DEL",     cmd_del     },
    { "PRIO",    cmd_prio    },
    { "PIN",     cmd_pin     },
    { "MODE",    cmd_mode    },
    { "LED",     cmd_led     },
    { "AP",      cmd_ap      },
    { "FACTORY", cmd_factory },
    { "REBOOT",  cmd_reboot  },
    { "FLIST",   cmd_flist   },
    { "FSTAT",   cmd_fstat   },
    { "FDEL",    cmd_fdel    },
    { "FSEL",    cmd_fsel    },
    { "FNAME",   cmd_fname   },
    { "FPUT",    cmd_fput    },
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

    /* 把 vfs 的读写切到驱动路径,否则 stdout 绕过驱动、与命令写冲突。
     *
     * 【2026-10-29 重要修正】不能用 use_driver()!
     *
     * use_driver() 把 stdout 的 tx_func 换成 usbjtag_tx_char_via_driver(),
     * 后者是 `usb_serial_jtag_write_bytes(&ch, 1, portMAX_DELAY)` —— 每次写
     * **一个字符**且**无限等待**。tx ring buffer(1024B)写满后,只要有任何一个
     * 线程在打日志,就会永久阻塞在 xRingbufferSend 里。
     *
     * 现场表现:板子拔掉 USB 单独上电 → 白灯呼吸一直不变、WiFi 也连不上。
     * 因为 dap_cmd_init() 在 wifi_init() 之前,而 wifi_init() 里满是
     * ESP_LOGI("wifi driver ready"/"main network"/"waiting for AP"...),
     * 没有 USB 主机把这些字节读走,缓冲区一满,wifi_init 就卡死在这儿 ——
     * LED 永远停在 DAP_LED_BOOT(白色呼吸)。一插上 USB,主机开始取数据,
     * 缓冲区腾空,阻塞解除,启动流程继续,灯才变色。这就是"插 USB 就好"的原因。
     *
     * use_nonblocking() 走的是 usb_serial_jtag_tx_char():直写硬件 FIFO,
     * FIFO 满 50ms 后主动丢字节,永不阻塞。命令应答和日志都可能被丢,但都有
     * 上层兜底(命令有 @ 前缀 + PC 侧重试),而"整机启动卡死"是不可接受的。
     */
    esp_vfs_usb_serial_jtag_use_nonblocking();
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
