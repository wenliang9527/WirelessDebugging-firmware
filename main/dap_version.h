/**
 * @file dap_version.h
 * @brief 固件版本号(单一来源)
 *
 * 版本规则:
 *   - 0.4.x  上游 windowsair v0.4.0 基座
 *   - 0.5.0  本项目 v3(串口桥 + S3 UART 分支 + 槽位释放修复),无 AP
 *   - 0.6.0  产品化第一版(USB 命令通道 + 双模式切换)
 *   - 0.7.0  产品化第二版(配网命令 @SCAN/@CONNECT + NVS 配置)
 *
 * 现场排障时先看启动日志的 fw= 字段确认固件身份。
 */
#ifndef __DAP_VERSION__
#define __DAP_VERSION__

#define DAP_FW_VERSION_MAJOR 0
#define DAP_FW_VERSION_MINOR 5
#define DAP_FW_VERSION_PATCH 0

#define DAP_STR_(x) #x
#define DAP_STR(x) DAP_STR_(x)
#define DAP_FW_VERSION \
    DAP_STR(DAP_FW_VERSION_MAJOR) "." DAP_STR(DAP_FW_VERSION_MINOR) "." DAP_STR(DAP_FW_VERSION_PATCH)

/** 固件标识:出现在启动日志与 @PING 响应里 */
#define DAP_FW_ID "dap-" DAP_FW_VERSION

#endif
