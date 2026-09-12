# ============================================================================
# usb_cam_stream —— 通过 Type-C USB(CDC ACM) 把相机画面发给主机
#
# 数据通路:
#   VICAP 采集(NV12) -> VENC JPEG 编码 -> 写 /dev/ttyGS0 (USB 虚拟串口)
#   主机端: 脚本 scripts/usb_cam_viewer.py 读串口解帧并显示
#
# 协议（每帧 16 字节定长头 + JPEG 数据）:
#   [0..3]   magic 'L','C','J','P'
#   [4..7]   JPEG 长度 (uint32, little-endian)
#   [8..11]  图像宽 (uint32, little-endian)
#   [12..15] 图像高 (uint32, little-endian)
#   [16..]   JPEG 数据
#
# 编译: bash scripts/build.sh usb_cam_stream
# 产物: build/usb_cam_stream/usb_cam_stream
# 部署: bash scripts/deploy.sh usb_cam_stream <板子IP> /sdcard/app
# 运行(msh): /sdcard/app/usb_cam_stream -w 640 -h 480
# 由 launcher 开机启动: 见 apps/launcher/startup.list
# ============================================================================

set(APP_NAME usb_cam_stream)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_executable(${APP_NAME} ${APP_SOURCES})

target_compile_options(${APP_NAME} PRIVATE
    -Wall -Wextra
    -ffunction-sections -fdata-sections)

# 与 green_led_rtos 相同的 MPP 用户态库目录（+venc 用于 JPEG 编码）
target_link_directories(${APP_NAME} PRIVATE
    "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/lib")

target_link_libraries(${APP_NAME} PRIVATE
    k230_rtos_app
    -Wl,--start-group vicap vb sys sensor isp vo connector framework venc
    -Wl,--end-group)

set_target_properties(${APP_NAME} PROPERTIES
    OUTPUT_NAME ${APP_NAME}
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${APP_NAME}")
