/**
 * @file dap_config.c
 * @brief 产品化 P2:NVS 配置层实现
 *
 * NVS 布局(命名空间 "dap_cfg"):
 *   count    u8    用户保存的网络条数(0..DAP_CFG_MAX_AP)
 *   mode     u8    模式偏好 0=STA 1=AP
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
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "dap.cfg";

#define NVS_NS          "dap_cfg"
#define KEY_COUNT       "count"
#define KEY_MODE        "mode"
#define KEY_LED_PCT     "led_pct"

/* NVS 键名缓冲区:足够容纳 "ap<4>_ssid" / "ap<4>_pass" */
#define KEY_BUF_LEN     24

/* 内存镜像:NVS 或出厂列表加载后的工作副本 */
static dap_cfg_ap_t s_aps[DAP_CFG_MAX_AP];
static int  s_count = 0;
static bool s_ready = false;
static bool s_factory = true;   /* true = 用的是编译期出厂列表 */
static uint8_t s_mode_pref = 0; /* 0=STA 1=AP */
static uint8_t s_led_pct = DAP_CFG_LED_DEFAULT_PCT;

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

/** 把编译期列表复制进内存镜像(出厂默认) */
static void load_factory_list(void) {
    s_count = 0;
    int n = (int)WIFI_LIST_SIZE;
    if (n > DAP_CFG_MAX_AP) {
        n = DAP_CFG_MAX_AP;
    }
    for (int i = 0; i < n; i++) {
        copy_ssid(s_aps[i].ssid, wifi_list[i].ssid);
        copy_pass(s_aps[i].password, wifi_list[i].password);
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
        char ks[KEY_BUF_LEN], kp[KEY_BUF_LEN];
        key_ssid(i, ks, sizeof(ks));
        key_pass(i, kp, sizeof(kp));

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
        copy_ssid(s_aps[loaded].ssid, ssid);
        copy_pass(s_aps[loaded].password, pass);
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
        char ks[KEY_BUF_LEN], kp[KEY_BUF_LEN];
        key_ssid(i, ks, sizeof(ks));
        key_pass(i, kp, sizeof(kp));
        nvs_erase_key(h, ks);
        nvs_erase_key(h, kp);
    }
    nvs_erase_key(h, KEY_COUNT);

    for (int i = 0; i < s_count; i++) {
        char ks[KEY_BUF_LEN], kp[KEY_BUF_LEN];
        key_ssid(i, ks, sizeof(ks));
        key_pass(i, kp, sizeof(kp));
        err = nvs_set_str(h, ks, s_aps[i].ssid);
        if (err != ESP_OK) {
            goto done;
        }
        err = nvs_set_str(h, kp, s_aps[i].password);
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
    s_count = 0;
    s_mode_pref = 0;
    s_factory = true;

    if (!load_from_nvs()) {
        load_factory_list();
        ESP_LOGI(TAG, "no user config, using %d factory entries", s_count);
    } else {
        ESP_LOGI(TAG, "loaded %d entries from NVS", s_count);
    }
    s_ready = true;
    return ESP_OK;
}

int dap_config_count(void) {
    return s_count;
}

bool dap_config_get(int idx, dap_cfg_ap_t *out) {
    if (!out || idx < 0 || idx >= s_count) {
        return false;
    }
    *out = s_aps[idx];
    return true;
}

esp_err_t dap_config_add(const char *ssid, const char *password) {
    if (!ssid || ssid[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

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
    }

    return save_all();
}

esp_err_t dap_config_del(int idx) {
    if (idx < 0 || idx >= s_count) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int i = idx; i < s_count - 1; i++) {
        s_aps[i] = s_aps[i + 1];
    }
    s_count--;
    memset(&s_aps[s_count], 0, sizeof(s_aps[0]));
    return save_all();
}

bool dap_config_using_factory(void) {
    return s_factory;
}

esp_err_t dap_config_factory_reset(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_erase_all(h);
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
    load_factory_list();
    return err;
}

esp_err_t dap_config_set_mode_pref(uint8_t mode) {
    s_mode_pref = (mode == 1) ? 1 : 0;
    /* 只有用户配置存在时才写盘;否则只改内存,避免为一条偏好创建命名空间 */
    if (!s_factory) {
        return save_all();
    }
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(h, KEY_MODE, s_mode_pref);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

uint8_t dap_config_get_mode_pref(void) {
    return s_mode_pref;
}

/* ------------------------------------------------------------------ */
/* 状态灯亮度                                                          */
/* ------------------------------------------------------------------ */

esp_err_t dap_config_set_led_brightness(uint8_t pct) {
    if (pct < DAP_CFG_LED_MIN_PCT || pct > DAP_CFG_LED_MAX_PCT) {
        return ESP_ERR_INVALID_ARG;
    }
    s_led_pct = pct;

    /* 亮度是独立键,单独写一个就够了 —— 不必整表重写。
     * 也刻意不受 s_factory 限制:哪怕没配过网,亮度也该能存下来。 */
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(h, KEY_LED_PCT, s_led_pct);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

uint8_t dap_config_get_led_brightness(void) {
    return s_led_pct;
}
