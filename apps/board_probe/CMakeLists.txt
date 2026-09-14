# ============================================================================
# board_probe —— 板端环境探针（MMZ/堆、CPU 频率、RVV、设备节点、PM 档位）
#
# 编译: bash scripts/build.sh board_probe
# 产物: build/board_probe/board_probe
# 运行: msh> /sdcard/app/board_probe   （前台，几十秒内结束）
# 日志: /sdcard/app/logs/board_probe.log
# ============================================================================

set(APP_NAME board_probe)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_executable(${APP_NAME} ${APP_SOURCES})

target_compile_options(${APP_NAME} PRIVATE
    -O2 -Wall -Wextra
    -ffunction-sections -fdata-sections)
target_link_options(${APP_NAME} PRIVATE -s)

target_link_directories(${APP_NAME} PRIVATE
    "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/lib")

target_link_libraries(${APP_NAME} PRIVATE
    k230_rtos_app
    -Wl,--start-group sys pm -Wl,--end-group m)

set_target_properties(${APP_NAME} PROPERTIES
    OUTPUT_NAME ${APP_NAME}
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${APP_NAME}")
