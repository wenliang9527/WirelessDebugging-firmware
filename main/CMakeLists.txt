set(COMPONENT_ADD_INCLUDEDIRS "${PROJECT_PATH}")
set(COMPONENT_SRCS
    main.c timer.c tcp_server.c usbip_server.c DAP_handle.c
    uart_bridge.c wifi_handle.c)

if(CONFIG_USE_WEBSOCKET_DAP)
    list(APPEND COMPONENT_SRCS "websocket_server.c")
endif()

# wifi_credentials.h 是可选文件(本地私有,已 .gitignore)。它由
# wifi_configuration.h 用 __has_include 引入,编译器不会把"文件不存在"
# 记成依赖,于是增删该文件后增量编译不会重编 main.c —— 表现为改了 WiFi
# 配置却毫无变化。把它显式列为依赖(存在时),让 ninja 能感知其变化。
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/wifi_credentials.h")
    set_source_files_properties(
        main.c wifi_handle.c
        PROPERTIES OBJECT_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/wifi_credentials.h")
endif()

register_component()
