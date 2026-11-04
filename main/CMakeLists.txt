set(COMPONENT_ADD_INCLUDEDIRS "${PROJECT_PATH}")
set(COMPONENT_SRCS
    main.c timer.c tcp_server.c usbip_server.c DAP_handle.c
    uart_bridge.c wifi_handle.c dap_cmd.c dap_config.c dap_led.c
    dap_store.c)

# WS2812 状态灯用 led_strip 组件(已复制到 <project>/components/led_strip)。
# 这里【不】设置 COMPONENT_REQUIRES —— 本项目是 register_component() 老风格,
# 默认靠全量依赖;一旦显式写 requires 就会替换掉默认列表,导致 nvs.h /
# gpio_struct.h / esp_ota_ops.h 等一堆头文件找不到。组件目录本身在
# components/ 下,能被自动发现。

if(CONFIG_USE_WEBSOCKET_DAP)
    list(APPEND COMPONENT_SRCS "websocket_server.c")
endif()

register_component()
