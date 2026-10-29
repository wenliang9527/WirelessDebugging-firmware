/**
 * @file wifi_configuration.h
 * @brief Fill in your wifi configuration information here.
 * @version 0.1
 * @date 2020-01-22
 *
 * @copyright Copyright (c) 2020
 *
 */
#ifndef __WIFI_CONFIGURATION__
#define __WIFI_CONFIGURATION__

/**
 * WiFi 凭据来源:
 *   - 若存在 main/wifi_credentials.h(本地私有,已 .gitignore),用里面的列表
 *   - 否则回落到下面的出厂默认列表(DAP / OTA)
 *
 * 这样真实 WiFi 密码不会进入版本历史;开源发布出去的是出厂默认列表。
 */
#if defined(__has_include)
#  if __has_include("main/wifi_credentials.h")
#    include "main/wifi_credentials.h"
#    define WIFI_LIST_IS_FACTORY_DEFAULT 0
#  endif
#endif

#ifndef WIFI_LIST_IS_FACTORY_DEFAULT
#define WIFI_LIST_IS_FACTORY_DEFAULT 1

/* 产品化主网模型(2026-10-29):出厂 = 未配网。
 * 凭据一律由 PC 工具经 USB 配网(@CONNECT)写入 NVS;固件不内置任何
 * 默认 WiFi —— 消掉"出厂默认密码 12345678"这个已知凭据隐患。
 * 本地开发者仍可建 main/wifi_credentials.h(已 gitignore)做开发便利。 */
static struct {
    const char *ssid;
    const char *password;
} wifi_list[] __attribute__((unused)) = {
    {0, 0},
};

#define WIFI_LIST_SIZE 0

#endif // WIFI_LIST_IS_FACTORY_DEFAULT

#define USE_MDNS       1
// Use the address "dap.local" to access the device
#define MDNS_HOSTNAME "dap"
#define MDNS_INSTANCE "DAP mDNS"
//

#define USE_STATIC_IP 0
// If you don't want to specify the ip configuration, then ignore the following items.
#define DAP_IP_ADDRESS 192, 168, 137, 123
#define DAP_IP_GATEWAY 192, 168, 137, 1
#define DAP_IP_NETMASK 255, 255, 255, 0
//

#define USE_OTA              0

#define USE_UART_BRIDGE      1
#define UART_BRIDGE_PORT     1234
#define UART_BRIDGE_BAUDRATE 115200
// 串口桥 TCP:1234 <-> S3 UART1(GPIO17=TX / GPIO18=RX,见 uart_bridge.c 自补的 S3 分支)
// 目标串口 3.3V 直连共地;单客户端;连接后首包 2-7 位纯数字会被当作波特率命令
//

// DO NOT CHANGE
#define USE_TCP_NETCONN 0

#define PORT                3240
#define CONFIG_EXAMPLE_IPV4 1
#define USE_KCP             0
#define MTU_SIZE            1500
//

#if (USE_TCP_NETCONN == 1 && USE_KCP == 1)
#error Can not use KCP and TCP at the same time!
#endif

#if (USE_KCP == 1)
#warning KCP is a very experimental feature, and it should not be used under any circumstances. Please make sure what you are doing. Related usbip version: https://github.com/windowsair/usbip-win
#endif


extern int printf(const char *, ...);
inline int os_printf(const char *__restrict __fmt, ...)  {
    return printf(__fmt, __builtin_va_arg_pack());
}

#endif