/**
 * @file dap_led.c
 * @brief P2.5:板载 WS2812 状态指示灯(实现)
 *
 * 硬件事实(原理图 V1.4 核对):
 *   U1 = XL-5050RGBC-WS2812B,DIN(3 脚)经 0Ω 电阻接 GPIO48,DO 悬空。
 *   单颗灯,不需要级联;用 RMT 通道产生 800kHz 单总线时序。
 */
#include "sdkconfig.h"

#include "main/dap_led.h"
#include "main/dap_config.h"

#if defined CONFIG_IDF_TARGET_ESP32S3

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/rmt.h"
#include "led_strip.h"

static const char *TAG = "dap.led";

/* 原理图确认的引脚 */
#define LED_GPIO            48
#define LED_COUNT           1
#define LED_RMT_CHANNEL     RMT_CHANNEL_0

/* 亮度:运行时从配置层取,可用 @LED 命令调,存 NVS 重启后保持。
 *
 * 为什么默认值这么低(4%):WS2812 是高亮 LED,人眼对亮度是非线性感知的,
 * 15% 的绿色(=38/255 PWM)在室内已经明显刺眼。要拿它当"状态指示"而不是
 * 照明,4% 左右刚好——暗处能看清,又不至于盯着难受。 */
static volatile uint8_t s_pct = DAP_CFG_LED_DEFAULT_PCT;

#define LED_TASK_STACK      2560
#define LED_TASK_PRIO       3
#define LED_TICK_MS         60      /* 动画节拍 */

static led_strip_t *s_strip = NULL;
static volatile dap_led_state_t s_state = DAP_LED_BOOT;
static volatile bool s_run = false;

/** 按当前亮度百分比缩放单通道 */
static inline uint32_t scale(uint32_t v)
{
    return (v * (uint32_t)s_pct) / 100u;
}

/**
 * 写一个颜色。
 * 注意 WS2812 的 GRB 顺序由 led_strip 组件内部处理,这里按 RGB 传即可。
 */
static void led_write(uint32_t r, uint32_t g, uint32_t b)
{
    if (!s_strip) {
        return;
    }
    s_strip->set_pixel(s_strip, 0, scale(r), scale(g), scale(b));
    s_strip->refresh(s_strip, 100);
}

/** 逻辑像素缩放:给呼吸/闪烁用的乘法 */
static void led_write_level(uint32_t r, uint32_t g, uint32_t b, uint32_t pct)
{
    if (!s_strip) {
        return;
    }
    s_strip->set_pixel(s_strip, 0,
                       scale(r * pct / 100u),
                       scale(g * pct / 100u),
                       scale(b * pct / 100u));
    s_strip->refresh(s_strip, 100);
}

static void led_task(void *arg)
{
    uint32_t tick = 0;
    while (s_run) {
        dap_led_state_t st = s_state;

        switch (st) {
        case DAP_LED_BOOT: {
            /* 白色呼吸:三角波 5%..100%,周期约 2s */
            uint32_t phase = tick % 33;
            uint32_t lvl = (phase <= 16) ? (5 + phase * 95 / 16)
                                         : (5 + (32 - phase) * 95 / 16);
            led_write_level(255, 255, 255, lvl);
            break;
        }
        case DAP_LED_CONNECTING: {
            /* 黄色慢闪:1.2s 周期,亮 40% */
            uint32_t phase = tick % 20;
            led_write_level(255, 180, 0, phase < 8 ? 100 : 0);
            break;
        }
        case DAP_LED_STA_OK:
            /* 绿色常亮:只在状态刚变化时写一次,避免无谓刷 RMT */
            led_write_level(0, 255, 60, 100);
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        case DAP_LED_AP_MODE: {
            /* 蓝色慢闪:1.2s 周期 */
            uint32_t phase = tick % 20;
            led_write_level(0, 90, 255, phase < 8 ? 100 : 0);
            break;
        }
        case DAP_LED_FAULT: {
            /* 红色快闪:0.36s 周期,明显区别于"连接中" */
            uint32_t phase = tick % 6;
            led_write_level(255, 0, 0, phase < 3 ? 100 : 0);
            break;
        }
        default:
            led_write(0, 0, 0);
            break;
        }

        tick++;
        vTaskDelay(pdMS_TO_TICKS(LED_TICK_MS));
    }

    led_write(0, 0, 0);
    vTaskDelete(NULL);
}

esp_err_t dap_led_init(void)
{
    if (s_strip) {
        return ESP_OK;
    }

    /* 亮度来自配置层(NVS),没配过就用默认值 */
    uint8_t pct = dap_config_get_led_brightness();
    if (pct < DAP_CFG_LED_MIN_PCT || pct > DAP_CFG_LED_MAX_PCT) {
        pct = DAP_CFG_LED_DEFAULT_PCT;
    }
    s_pct = pct;

    /* 用带 gpio 参数的这个入口:同版本的 led_strip_new_rmt_ws2812() 只收
     * channel/max_leds,不知道怎么指定数据脚,别用错。 */
    s_strip = led_strip_init((uint8_t)LED_RMT_CHANNEL, LED_GPIO, LED_COUNT);
    if (!s_strip) {
        ESP_LOGW(TAG, "WS2812 init failed (GPIO%d); status LED disabled",
                 LED_GPIO);
        return ESP_FAIL;
    }

    /* 有些版本的组件需要显式 clear 一次,把上电残留颜色清掉 */
    s_strip->clear(s_strip, 50);

    s_run = true;
    s_state = DAP_LED_BOOT;
    if (xTaskCreate(led_task, "dap_led", LED_TASK_STACK, NULL,
                    LED_TASK_PRIO, NULL) != pdPASS) {
        ESP_LOGW(TAG, "LED task create failed; status LED disabled");
        s_run = false;
        s_strip->del(s_strip);
        s_strip = NULL;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "WS2812 status LED on GPIO%d (%d%%)",
             LED_GPIO, (int)s_pct);
    return ESP_OK;
}

void dap_led_set_brightness(uint8_t pct)
{
    if (pct < DAP_CFG_LED_MIN_PCT) {
        pct = DAP_CFG_LED_MIN_PCT;
    } else if (pct > DAP_CFG_LED_MAX_PCT) {
        pct = DAP_CFG_LED_MAX_PCT;
    }
    /* 只改这个变量:动画任务下一拍就会用新亮度重画,不必重建 RMT。 */
    s_pct = pct;
}

uint8_t dap_led_get_brightness(void)
{
    return s_pct;
}

void dap_led_set(dap_led_state_t state)
{
    s_state = state;
}

void dap_led_off(void)
{
    if (s_strip) {
        s_strip->clear(s_strip, 50);
    }
}

bool dap_led_available(void)
{
    return s_strip != NULL;
}

#else  /* 非 S3:保持空实现,不影响其他目标编译 */

esp_err_t dap_led_init(void) { return ESP_ERR_NOT_SUPPORTED; }
void dap_led_set(dap_led_state_t state) { (void)state; }
void dap_led_off(void) { }
bool dap_led_available(void) { return false; }
void dap_led_set_brightness(uint8_t pct) { (void)pct; }
uint8_t dap_led_get_brightness(void) { return 0; }

#endif
