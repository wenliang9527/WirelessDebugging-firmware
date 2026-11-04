/**
 * @file dap_upload.h
 * @brief 离线烧录 D3:WiFi 镜像上传服务(TCP:1235)
 *
 * 与 USB 兜底通道(@FPUT)共用 dap_store 的单会话锁:两边谁先 begin
 * 谁占住,后到者被拒。一次只服务一个客户端(GUI 串行上传)。
 */
#ifndef __DAP_UPLOAD_H__
#define __DAP_UPLOAD_H__

#include "esp_err.h"

#define DAP_UPLOAD_PORT 1235

/** 起上传服务任务(监听 DAP_UPLOAD_PORT) */
esp_err_t dap_upload_init(void);

#endif /* __DAP_UPLOAD_H__ */
