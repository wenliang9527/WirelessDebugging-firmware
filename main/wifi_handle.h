#ifndef _WIFI_HANDLE_H_
#define _WIFI_HANDLE_H_

/**
 * WiFi 硬件初始化 + 按 NVS 记录的偏好模式启动。
 * STA 模式下会阻塞到拿到 IP;AP 模式下立即返回。
 */
void wifi_init(void);

#endif