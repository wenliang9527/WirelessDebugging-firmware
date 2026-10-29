/**
 * @file dap_version.h
 * @brief 固件版本号(单一来源)
 *
 * 版本规则:
 *   - 0.4.x  上游 windowsair v0.4.0 基座
 *   - 0.5.0  本项目 v3(串口桥 + S3 UART 分支 + 槽位释放修复),无 AP
 *   - 0.6.0  产品化第一版(USB 命令通道 + 双模式切换)
 *   - 0.7.0  产品化第二版(配网命令 @SCAN/@CONNECT + NVS 配置)
 *   - 0.7.1  修 uart_server 任务栈溢出:连 1234 必触发 esp_restart,
 *           表现为"板子反复重启、串口桥时而连不上"(commit ce993c5)
 *   - 0.7.2  修 use_driver() 阻塞:拔掉 USB 单独上电时 stdout 阻塞在
 *           tx ring buffer 上,wifi_init 永远跑不完,灯停在白色呼吸;
 *           插上 USB 才继续启动(commit d4bce21)
 *   - 0.8.0  AP 热点密码不再全局硬编码 12345678:首启随机生成 10 位、
 *           存 NVS、经 @AP 命令读回。新增 @AP / @AP REGEN。
 *
 * 0.7.1 与 0.7.2 是同一天连着改的两处启动流程缺陷,没有单独出过 0.7.1
 * 的包 —— 现场能拿到的第一个同时含两处修复的版本就是 0.7.2。
 * 这两个版本都只动启动与运行时行为,命令集与 NVS 布局不变,
 * 所以配网工具的版本门槛仍是 0.6(见 tools/dap_config.py 的 version_tuple)。
 *
 * 现场排障时先看启动日志的 fw= 字段确认固件身份:
 *   白灯呼吸不动、插 USB 才启动  -> fw < 0.7.2 的老固件
 *   连 1234 就重启              -> fw < 0.7.1 的老固件
 */
#ifndef __DAP_VERSION__
#define __DAP_VERSION__

#define DAP_FW_VERSION_MAJOR 0
#define DAP_FW_VERSION_MINOR 8
#define DAP_FW_VERSION_PATCH 0

#define DAP_STR_(x) #x
#define DAP_STR(x) DAP_STR_(x)
#define DAP_FW_VERSION \
    DAP_STR(DAP_FW_VERSION_MAJOR) "." DAP_STR(DAP_FW_VERSION_MINOR) "." DAP_STR(DAP_FW_VERSION_PATCH)

/** 固件标识:出现在启动日志与 @PING 响应里 */
#define DAP_FW_ID "dap-" DAP_FW_VERSION

#endif
