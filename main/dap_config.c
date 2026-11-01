/**
 * @file dap_config.c
 * @brief 产品化 P2:NVS 配置层实现
 *
 * NVS 布局(命名空间 "dap_cfg"):
 *   count    u8    用户保存的网络条数(0..DAP_CFG_MAX_AP)
 *   mode     u8    模式偏好 0=STA 1=AP
 *   led_pct  u8    状态灯亮度百分比
 *   ap_pass  str   AP 热点密码(10 字符,首启随机生成,设备身份)
 *   ap0_ssid str   第 0 条 SSID
 *   ap0_pass str   第 0 条密码
 *   ...            ap1_..ap4_
 *
 * 采用"整表重写"策略:任何增删都按当前内存镜像整体写回。
 * 条数上限只有 5,写入量极小,换来逻辑简单、不会出现索引空洞。
 */
#include "sdkconfig.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "main/dap_config.h"
#include "main/wifi_configuration.h"

#include "esp_log.h"
#include "esp_system.h"      /* esp_random() */
#include "esp_mac.h"          /* esp_efuse_mac_get_default():见下方兼容说明 */
#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "dap.cfg";

#define NVS_NS          "dap_cfg"
#define KEY_COUNT       "count"
#define KEY_MODE        "mode"
#define KEY_LED_PCT     "led_pct"
#define KEY_AP_PASS     "ap_pass"

/* NVS 键名缓冲区:足够容纳 "ap<4>_ssid" / "ap<4>_pass" */
#define KEY_BUF_LEN     24

/* 内存镜像:NVS 或出厂列表加载后的工作副本 */
static dap_cfg_ap_t s_aps[DAP_CFG_MAX_AP];
static int  s_count = 0;
static bool s_ready = false;
static bool s_factory = true;   /* true = 用的是编译期出厂列表 */
static uint8_t s_mode_pref = 0; /* 0=STA 1=AP */
static uint8_t s_led_pct = DAP_CFG_LED_DEFAULT_PCT;
static char s_ap_pass[DAP_CFG_AP_PASS_LEN] = {0};  /* 空 = 还没生成 */

/* ------------------------------------------------------------------ */
/* 并发控制(体检 A-1,2026-11-01)                                      */
/* ------------------------------------------------------------------ */
/* 内存镜像(s_aps/s_count/s_mode_pref/s_led_pct/s_ap_pass)有两个写者:
 *   - cmd_task(USB 命令:@DEL/@PIN/@LED,以及 @CONNECT 成功落盘)
 *   - WiFi 事件回调(GOT_IP -> dap_config_add() 整表重写;ssid_change() 读)
 * 两者分属不同任务,此前无任何保护:@CONNECT 拿到 IP 的瞬间与 PC 侧的
 * @LIST/@DEL 并发,会撕裂读/写。这里用一把递归互斥罩住全部公开接口 ——
 * 条目 <= 5、操作都是毫秒级 NVS 读写,锁开销可忽略。
 * 用递归锁的唯一原因:dap_config_init() 会调 dap_config_regen_ap_password()
 * (公开接口),非递归锁会自锁;除此之外没有公开 -> 公开调用。 */
static SemaphoreHandle_t s_cfg_mux = NULL;

static void cfg_lock(void) {
    xSemaphoreTakeRecursive(s_cfg_mux, portMAX_DELAY);
}

static void cfg_unlock(void) {
    xSemaphoreGiveRecursive(s_cfg_mux);
}

/* ------------------------------------------------------------------ */

static void copy_ssid(char *dst, const char *src) {
    snprintf(dst, DAP_CFG_SSID_LEN, "%s", src ? src : "");
}

static void copy_pass(char *dst, const char *src) {
    snprintf(dst, DAP_CFG_PASS_LEN, "%s", src ? src : "");
}

/** 生成 NVS 键名,避免 format-truncation 警告 */
static void key_ssid(int idx, char *buf, size_t len) {
    snprintf(buf, len, "ap%d_ssid", idx);
}

static void key_pass(int idx, char *buf, size_t len) {
    snprintf(buf, len, "ap%d_pass", idx);
}

static void key_pin(int idx, char *buf, size_t len) {
    snprintf(buf, len, "ap%d_pin", idx);
}

/** 把编译期列表复制进内存镜像(出厂默认)。
 * 产品化主网模型:出厂 = 未配网,编译期列表不再参与(凭据一律经工具配网落 NVS)。
 * 本地开发者的 wifi_credentials.h 仍可编译进固件当开发便利,但走的是
 * "用户配置"通道而非出厂通道 —— 这里保持读它,行为与老版本兼容。 */
static void load_factory_list(void) {
    s_count = 0;
    int n = (int)WIFI_LIST_SIZE;
    if (n > DAP_CFG_MAX_AP) {
        n = DAP_CFG_MAX_AP;
    }
    for (int i = 0; i < n; i++) {
        copy_ssid(s_aps[i].ssid, wifi_list[i].ssid);
        copy_pass(s_aps[i].password, wifi_list[i].password);
        s_aps[i].pin = 0;
    }
    s_count = n;
    s_factory = true;
}

/** 从 NVS 读整表。返回 true 表示读到了至少一条用户配置。 */
static bool load_from_nvs(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        return false;
    }

    /* 亮度是独立偏好:即使一条用户网络都没存,也要读出来。
     * 早退之前先处理掉,避免"只改了亮度、没配网"时设置丢失。 */
    uint8_t pct = 0;
    if (nvs_get_u8(h, KEY_LED_PCT, &pct) == ESP_OK &&
        pct >= DAP_CFG_LED_MIN_PCT && pct <= DAP_CFG_LED_MAX_PCT) {
        s_led_pct = pct;
    }

    uint8_t cnt = 0;
    if (nvs_get_u8(h, KEY_COUNT, &cnt) != ESP_OK || cnt == 0) {
        nvs_close(h);
        return false;
    }
    if (cnt > DAP_CFG_MAX_AP) {
        cnt = DAP_CFG_MAX_AP;
    }

    int loaded = 0;
    for (int i = 0; i < cnt; i++) {
        char ks[KEY_BUF_LEN], kp[KEY_BUF_LEN], kt[KEY_BUF_LEN];
        key_ssid(i, ks, sizeof(ks));
        key_pass(i, kp, sizeof(kp));
        key_pin(i, kt, sizeof(kt));

        size_t sl = DAP_CFG_SSID_LEN;
        size_t pl = DAP_CFG_PASS_LEN;
        char ssid[DAP_CFG_SSID_LEN] = {0};
        char pass[DAP_CFG_PASS_LEN] = {0};

        if (nvs_get_str(h, ks, ssid, &sl) != ESP_OK || ssid[0] == '\0') {
            continue;  /* 跳过空洞,不让一条坏记录毁掉整表 */
        }
        if (nvs_get_str(h, kp, pass, &pl) != ESP_OK) {
            pass[0] = '\0';  /* 开放网络 */
        }
        uint8_t pin = 0;
        nvs_get_u8(h, kt, &pin);   /* 缺键 = 未 pin,不必报错 */
        copy_ssid(s_aps[loaded].ssid, ssid);
        copy_pass(s_aps[loaded].password, pass);
        s_aps[loaded].pin = pin ? 1 : 0;
        loaded++;
    }

    uint8_t m = 0;
    if (nvs_get_u8(h, KEY_MODE, &m) == ESP_OK) {
        s_mode_pref = (m == 1) ? 1 : 0;
    }
    nvs_close(h);

    if (loaded == 0) {
        return false;
    }
    s_count = loaded;
    s_factory = false;
    return true;
}

/** 把内存镜像整体写回 NVS */
static esp_err_t save_all(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 先清掉可能残留的旧键(条数变少时),避免读到僵尸条目 */
    for (int i = 0; i < DAP_CFG_MAX_AP; i++) {
        char ks[KEY_BUF_LEN], kp[KEY_BUF_LEN], kt[KEY_BUF_LEN];
        key_ssid(i, ks, sizeof(ks));
        key_pass(i, kp, sizeof(kp));
        key_pin(i, kt, sizeof(kt));
        nvs_erase_key(h, ks);
        nvs_erase_key(h, kp);
        nvs_erase_key(h, kt);
    }
    nvs_erase_key(h, KEY_COUNT);

    for (int i = 0; i < s_count; i++) {
        char ks[KEY_BUF_LEN], kp[KEY_BUF_LEN], kt[KEY_BUF_LEN];
        key_ssid(i, ks, sizeof(ks));
        key_pass(i, kp, sizeof(kp));
        key_pin(i, kt, sizeof(kt));
        err = nvs_set_str(h, ks, s_aps[i].ssid);
        if (err != ESP_OK) {
            goto done;
        }
        err = nvs_set_str(h, kp, s_aps[i].password);
        if (err != ESP_OK) {
            goto done;
        }
        err = nvs_set_u8(h, kt, s_aps[i].pin ? 1 : 0);
        if (err != ESP_OK) {
            goto done;
        }
    }
    err = nvs_set_u8(h, KEY_COUNT, (uint8_t)s_count);
    if (err != ESP_OK) {
        goto done;
    }
    err = nvs_set_u8(h, KEY_MODE, s_mode_pref);
    if (err != ESP_OK) {
        goto done;
    }
    err = nvs_set_u8(h, KEY_LED_PCT, s_led_pct);
    if (err != ESP_OK) {
        goto done;
    }
    err = nvs_commit(h);

done:
    nvs_close(h);
    if (err == ESP_OK) {
        s_factory = (s_count == 0);
    }
    return err;
}

/* ------------------------------------------------------------------ */
/* 公开接口                                                            */
/* ------------------------------------------------------------------ */

esp_err_t dap_config_init(void) {
    if (s_ready) {
        return ESP_OK;
    }
    /* 先建锁:cmd_task 在 dap_cmd_init() 之后才存在、WiFi 事件循环更晚,
     * 此处是全流程唯一的无并发窗口,保证此后所有公开接口都拿得到锁。 */
    s_cfg_mux = xSemaphoreCreateRecursiveMutex();
    cfg_lock();
    s_count = 0;
    s_mode_pref = 0;
    s_factory = true;

    /* AP 密码是设备身份:先读出来(没配网也要读),读不到就现场生成。
     * 必须在 load_from_nvs 之前单独处理 —— 后者在"一条网络都没有"时会
     * 提前返回 false,不会走到后面的键。 */
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t plen = sizeof(s_ap_pass);
        if (nvs_get_str(h, KEY_AP_PASS, s_ap_pass, &plen) != ESP_OK) {
            s_ap_pass[0] = '\0';
        }
        nvs_close(h);
    }

    if (!load_from_nvs()) {
        load_factory_list();
        ESP_LOGI(TAG, "no user config, using %d factory entries", s_count);
    } else {
        ESP_LOGI(TAG, "loaded %d entries from NVS", s_count);
    }
    s_ready = true;

    /* 密码缺失/长度不对(NVS 里是别的版本写的)就重新生成一个。
     * 放在 s_ready=true 之后,保证只生成一次。 */
    if (strlen(s_ap_pass) != DAP_CFG_AP_PASS_CHARS) {
        dap_config_regen_ap_password();
    } else {
        ESP_LOGI(TAG, "AP password loaded (device identity)");
    }
    cfg_unlock();
    return ESP_OK;
}

int dap_config_count(void) {
    cfg_lock();
    int n = s_count;
    cfg_unlock();
    return n;
}

bool dap_config_get(int idx, dap_cfg_ap_t *out) {
    if (!out || idx < 0) {
        return false;
    }
    cfg_lock();
    bool ok = (idx < s_count);
    if (ok) {
        *out = s_aps[idx];
    }
    cfg_unlock();
    return ok;
}

esp_err_t dap_config_add(const char *ssid, const char *password) {
    if (!ssid || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    cfg_lock();
    /* 已存在同 SSID:更新密码并置顶,避免同一网络堆多条 */
    int found = -1;
    for (int i = 0; i < s_count; i++) {
        if (strcmp(s_aps[i].ssid, ssid) == 0) {
            found = i;
            break;
        }
    }

    if (found >= 0) {
        dap_cfg_ap_t tmp = s_aps[found];
        copy_pass(tmp.password, password);
        for (int i = found; i > 0; i--) {
            s_aps[i] = s_aps[i - 1];
        }
        s_aps[0] = tmp;
    } else {
        if (s_count < DAP_CFG_MAX_AP) {
            s_count++;
        }
        /* 整体后移,新的放最前(优先级最高) */
        for (int i = s_count - 1; i > 0; i--) {
            s_aps[i] = s_aps[i - 1];
        }
        copy_ssid(s_aps[0].ssid, ssid);
        copy_pass(s_aps[0].password, password);
        /* 整体后移让 s_aps[0] 仍带着旧首条的 pin,新条目必须显式清零,
         * 否则"新连的网络"会继承别人的固定优先。 */
        s_aps[0].pin = 0;
    }

    esp_err_t err = save_all();
    cfg_unlock();
    return err;
}

esp_err_t dap_config_del(int idx) {
    cfg_lock();
    if (idx < 0 || idx >= s_count) {
        cfg_unlock();
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = idx; i < s_count - 1; i++) {
        s_aps[i] = s_aps[i + 1];
    }
    s_count--;
    memset(&s_aps[s_count], 0, sizeof(s_aps[0]));
    esp_err_t err = save_all();
    cfg_unlock();
    return err;
}

esp_err_t dap_config_prioritize(int idx) {
    cfg_lock();
    if (idx < 0 || idx >= s_count) {
        cfg_unlock();
        return ESP_ERR_INVALID_ARG;
    }
    if (idx == 0) {
        cfg_unlock();
        return ESP_OK;  /* 已经在最前,不必写 NVS */
    }
    /* 抽出来再前插,其余元素的相对顺序保持不变。
     * 注意是"稳定"移动:别用整体排序,否则同优先级的相对次序会被打乱,
     * 用户看到的列表会莫名其妙地跳动。 */
    dap_cfg_ap_t tmp = s_aps[idx];
    for (int i = idx; i > 0; i--) {
        s_aps[i] = s_aps[i - 1];
    }
    s_aps[0] = tmp;
    esp_err_t err = save_all();
    cfg_unlock();
    return err;
}

esp_err_t dap_config_set_pin(int idx, bool pin) {
    cfg_lock();
    if (idx < 0 || idx >= s_count) {
        cfg_unlock();
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t v = pin ? 1 : 0;
    if (s_aps[idx].pin == v) {
        cfg_unlock();
        return ESP_OK;  /* 幂等:状态没变就不写 NVS */
    }
    s_aps[idx].pin = v;
    esp_err_t err = save_all();
    cfg_unlock();
    return err;
}

bool dap_config_pin_at(int idx) {
    cfg_lock();
    bool ok = (idx >= 0 && idx < s_count) && (s_aps[idx].pin != 0);
    cfg_unlock();
    return ok;
}

int dap_config_conn_physical(int logical) {
    cfg_lock();
    if (logical < 0 || logical >= s_count) {
        cfg_unlock();
        return -1;
    }
    /* 计算序 = pin 过的在前(组内保持物理序),未 pin 的跟随其后(即 LRU:
     * @CONNECT 自动置顶让"最近连过的"排物理最前,这里自然继承)。
     * 轮转游标 ssid_index 从此是"逻辑序号",看不懂这层映射的调用方
     * (dap_config_get 直取)不受影响。
     * 加锁改写:结果统一用 phys 变量带出,避免中途 return 漏解锁。 */
    int phys = -1;
    int pinned_seen = 0;
    for (int i = 0; i < s_count; i++) {
        if (s_aps[i].pin) {
            if (logical == pinned_seen) {
                phys = i;
                break;
            }
            pinned_seen++;
        }
    }
    if (phys < 0) {
        int unpinned_seen = 0;
        for (int i = 0; i < s_count; i++) {
            if (!s_aps[i].pin) {
                if (logical - pinned_seen == unpinned_seen) {
                    phys = i;
                    break;
                }
                unpinned_seen++;
            }
        }
    }
    cfg_unlock();
    return phys;
}

int dap_config_index_of(const char *ssid) {
    if (!ssid || ssid[0] == '\0') {
        return -1;
    }
    cfg_lock();
    int found = -1;
    for (int i = 0; i < s_count; i++) {
        if (strcmp(s_aps[i].ssid, ssid) == 0) {
            found = i;
            break;
        }
    }
    cfg_unlock();
    return found;
}

bool dap_config_using_factory(void) {
    cfg_lock();
    bool f = s_factory;
    cfg_unlock();
    return f;
}

esp_err_t dap_config_factory_reset(void) {
    cfg_lock();
    /* AP 密码是**设备身份**,不是"用户配置" —— 恢复出厂不该把它换掉。
     * 理由:
     *   1. 用户可能已经把密码抄在板子背面的标签上;悄悄换掉会让人彻底
     *      连不上,而 AP 恰好是"连不上路由器时的救命通道"。
     *   2. 恢复出厂后本来就断网了,此时再换密码 = 双重打击。
     * 所以先备份、erase_all 之后再写回。想主动换密码用 @AP REGEN。 */
    char saved_pass[DAP_CFG_AP_PASS_LEN];
    snprintf(saved_pass, sizeof(saved_pass), "%s",
             dap_config_get_ap_password());   /* 保证非空 */

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_erase_all(h);
        if (err == ESP_OK) {
            err = nvs_set_str(h, KEY_AP_PASS, saved_pass);
        }
        if (err == ESP_OK) {
            err = nvs_commit(h);
        }
        nvs_close(h);
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        err = ESP_OK;
    }

    s_count = 0;
    s_mode_pref = 0;
    s_led_pct = DAP_CFG_LED_DEFAULT_PCT;   /* 亮度也回默认 */
    snprintf(s_ap_pass, sizeof(s_ap_pass), "%s", saved_pass);
    load_factory_list();
    cfg_unlock();
    return err;
}

esp_err_t dap_config_set_mode_pref(uint8_t mode) {
    cfg_lock();
    s_mode_pref = (mode == 1) ? 1 : 0;
    /* 只有用户配置存在时才写盘;否则只改内存,避免为一条偏好创建命名空间 */
    if (!s_factory) {
        esp_err_t err = save_all();
        cfg_unlock();
        return err;
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        cfg_unlock();
        return err;
    }
    err = nvs_set_u8(h, KEY_MODE, s_mode_pref);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    cfg_unlock();
    return err;
}

uint8_t dap_config_get_mode_pref(void) {
    cfg_lock();
    uint8_t m = s_mode_pref;
    cfg_unlock();
    return m;
}

/* ------------------------------------------------------------------ */
/* 状态灯亮度                                                          */
/* ------------------------------------------------------------------ */

esp_err_t dap_config_set_led_brightness(uint8_t pct) {
    if (pct < DAP_CFG_LED_MIN_PCT || pct > DAP_CFG_LED_MAX_PCT) {
        return ESP_ERR_INVALID_ARG;
    }
    cfg_lock();
    s_led_pct = pct;

    /* 亮度是独立键,单独写一个就够了 —— 不必整表重写。
     * 也刻意不受 s_factory 限制:哪怕没配过网,亮度也该能存下来。 */
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        cfg_unlock();
        return err;
    }
    err = nvs_set_u8(h, KEY_LED_PCT, s_led_pct);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    cfg_unlock();
    return err;
}

uint8_t dap_config_get_led_brightness(void) {
    cfg_lock();
    uint8_t pct = s_led_pct;
    cfg_unlock();
    return pct;
}

/* ------------------------------------------------------------------ */
/* AP 热点密码                                                         */
/* ------------------------------------------------------------------ */

/* 字符表:去掉容易看错的 0/O、1/I/l、2/Z、5/S。
 * 用户多半是在手机小键盘上照着板子背面的贴纸敲,少一个歧义字符就少一次
 * "连不上热点"的客服。剩 31 个字符,10 位 ≈ 49.5 bit 熵,远超 WPA2 需要。 */
static const char s_ap_pass_chars[] = "346789ABCDEFGHJKMNPQRTUVWXY";

/**
 * 生成一个密码写进 s_ap_pass 并落盘。
 *
 * 熵源:esp_random()(硬件 RNG)+ eFuse MAC 混入。加 MAC 是为了同批次板子
 * 在 RNG 尚未充分抖动的极早期也不会生成相同密码 —— 它们通常在同一秒内
 * 被烧录,如果只靠 RNG,起步状态相近的芯片有撞车风险。
 *
 * 刻意避开 arc4random 之类需要额外初始化的源:这里在 dap_config_init()
 * 里调用,WiFi 还没起来,能用且只用得起的只有 esp_random()。
 */
static void ap_pass_generate(void) {
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);

    uint32_t seed = esp_random();
    for (int i = 0; i < 6; i++) {
        seed = seed * 16777619u ^ mac[i];       /* FNV-1a 风格混合 */
    }

    const size_t n = sizeof(s_ap_pass_chars) - 1;   /* 不含结尾 0 */
    for (int i = 0; i < DAP_CFG_AP_PASS_CHARS; i++) {
        /* 每轮再取一次 esp_random(),避免只靠一个 32 位种子循环取模 */
        seed = seed * 1103515245u + 12345u + esp_random();
        s_ap_pass[i] = s_ap_pass_chars[(seed >> 8) % n];
    }
    s_ap_pass[DAP_CFG_AP_PASS_CHARS] = '\0';
}

static esp_err_t ap_pass_save(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_str(h, KEY_AP_PASS, s_ap_pass);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

const char *dap_config_get_ap_password(void) {
    cfg_lock();
    if (s_ap_pass[0] == '\0') {
        /* 理论上 init 里已经生成过;这里兜底,防止有人绕过 init 直接调 */
        ap_pass_generate();
        ap_pass_save();
    }
    cfg_unlock();
    /* 返回内部缓冲指针是既有 API(调用方都是立刻拷走);REGEN 与读取
     * 实际都由 cmd_task 串行发起,不存在并发改写同一缓冲的窗口。 */
    return s_ap_pass;
}

const char *dap_config_regen_ap_password(void) {
    cfg_lock();
    ap_pass_generate();
    esp_err_t err = ap_pass_save();
    if (err != ESP_OK) {
        /* 存不下来也要能用(本次开机有效),但必须吼一声 —— 否则用户会
         * 在下次开机时发现密码变了,而没有任何线索。 */
        ESP_LOGE(TAG, "AP password NOT persisted (%s), it will change on reboot",
                 esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "AP password regenerated and saved");
    }
    cfg_unlock();
    return s_ap_pass;
}
