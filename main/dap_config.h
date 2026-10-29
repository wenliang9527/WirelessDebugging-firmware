/**
 * @file dap_config.h
 * @brief 产品化 P2:NVS 配置层(WiFi 凭据列表 + 模式偏好)
 *
 * 设计要点:
 *   - 最多保存 DAP_CFG_MAX_AP 条已配网络,按优先级排序(索引 0 最先尝试)
 *   - 没有 NVS 配置时回落到编译期列表(wifi_credentials.h 或出厂默认),
 *     因此老固件升级上来的板子不会突然失去联网能力
 *   - @FACTORY 只清本项目的命名空间,不动 WiFi 驱动自带的数据
 */
#ifndef __DAP_CONFIG_H__
#define __DAP_CONFIG_H__

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DAP_CFG_MAX_AP      5
#define DAP_CFG_SSID_LEN    33
#define DAP_CFG_PASS_LEN    65

/** 一条已保存的网络 */
typedef struct {
    char ssid[DAP_CFG_SSID_LEN];
    char password[DAP_CFG_PASS_LEN];
    uint8_t pin;    /* 1 = 固定优先:连接轮转永远排在未 pin 条目之前 */
} dap_cfg_ap_t;

/** 初始化(读 NVS,失败时标记为出厂默认状态) */
esp_err_t dap_config_init(void);

/** 当前生效的网络条数 */
int  dap_config_count(void);

/** 取第 idx 条(0 起)。返回 false 表示越界 */
bool dap_config_get(int idx, dap_cfg_ap_t *out);

/** 追加一条。已存在同 SSID 则更新密码并置顶。列表满时挤掉最后一条。 */
esp_err_t dap_config_add(const char *ssid, const char *password);

/** 删除第 idx 条(0 起) */
esp_err_t dap_config_del(int idx);

/**
 * 把第 idx 条移到最前(优先级最高),其余顺序不变。
 *
 * 为什么需要它:@CONNECT 成功会自动把目标网络置顶,所以"最近连过的"
 * 永远排第一。想让某条固定优先,光调一次顺序不够 —— 下次手动连别的
 * 网络又会被顶下去。这个接口配合 @PRIORITIZE 提供显式排序能力。
 */
esp_err_t dap_config_prioritize(int idx);

/**
 * pin 标记:给第 idx 条打上/摘掉"固定优先"。
 *
 * 与 @PRIO(物理置顶)的区别:@CONNECT 成功会自动把目标网络移到物理最前,
 * 一次性置顶会被下次手动连接顶掉;pin 是粘性标志,连接轮转时 pinned 条目
 * 永远排在未 pinned 条目之前,与物理顺序无关。物理顺序仍然表达"最近用过的
 * 未 pin 网络优先"。
 */
esp_err_t dap_config_set_pin(int idx, bool pin);

/** 第 idx 条是否被 pin(越界返回 false) */
bool dap_config_pin_at(int idx);

/**
 * 排名查询:返回 ssid 在列表中的位置,未找到返回 -1。
 * 供 @STATUS/@LIST 之外的诊断使用。
 */
int dap_config_index_of(const char *ssid);

/**
 * 连接顺序映射:第 logical 个尝试的网络在物理列表里的下标。
 *
 * 连接顺序 = pin 过的在前 + 未 pin 的在后(组内保持物理序)。
 * STA 轮转(wifi_handle.c 的 ssid_index)用这个游标遍历;@CONNECT 成功的
 * 自动置顶只动物理顺序,推不倒 pin —— 这正是 pin 与 @PRIO 的语义分界。
 * 越界返回 -1。
 */
int dap_config_conn_physical(int logical);

/** 是否正在使用编译期出厂列表(即 NVS 里没有任何用户配置) */
bool dap_config_using_factory(void);

/** 清空本项目命名空间并回到出厂默认 */
esp_err_t dap_config_factory_reset(void);

/** AP 模式偏好 */
esp_err_t dap_config_set_mode_pref(uint8_t mode);   /* 0=STA 1=AP */
uint8_t   dap_config_get_mode_pref(void);

/**
 * 状态灯亮度(百分比 1..100)。
 * 存在 NVS 里,重启后保持 —— 免得为"调暗一点"每次重烧固件。
 * 默认 DAP_CFG_LED_DEFAULT_PCT。
 */
#define DAP_CFG_LED_DEFAULT_PCT  4
#define DAP_CFG_LED_MIN_PCT      1
#define DAP_CFG_LED_MAX_PCT      100

esp_err_t dap_config_set_led_brightness(uint8_t pct);
uint8_t   dap_config_get_led_brightness(void);

#ifdef __cplusplus
}
#endif

#endif
