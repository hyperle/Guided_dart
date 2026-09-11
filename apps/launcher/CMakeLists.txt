# ============================================================================
# launcher —— 开机自启动器（读 startup_final.list，逐个拉起其他用户程序）
#
# 设计思路：
#   RT-Smart 里每个用户 ELF 是一个独立进程（内核 lwp，用户态 musl libc），
#   所以本启动器用标准 POSIX 的 fork()/execv()/waitpid() 把列表里的每个
#   程序拉成子进程（对应内核 sys_fork/sys_execve/sys_waitpid，K230 SDK 已实现）。
#
# 开机注册方式（不烧进镜像、直接替换 SD 卡文件即可更新）：
#   make menuconfig
#     RT-Smart Configuration
#       Rtsmart auto execute command string  = /sdcard/app/launcher
#   （注意：不要用 @preload/fastboot 快起模式，否则跑的是镜像里的备份，
#     替换 SD 卡文件不生效）
#
# 被启动程序清单（板端默认路径）：/sdcard/app/startup_final.list
#   （launcher 源码 DEFAULT_LIST；也可用 argv[1] 指定其它清单）
#   源码母版在 apps/launcher/startup.list，改完拷到板子并命名 startup_final.list
#   即可，无需重编 launcher。
#
# 构建：bash scripts/build.sh launcher
# 产物：build/launcher/launcher
# 部署：bash scripts/deploy.sh launcher <板子IP> /sdcard/app
# ============================================================================

set(APP_NAME launcher)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_executable(${APP_NAME} ${APP_SOURCES})

target_compile_options(${APP_NAME} PRIVATE
    -Wall -Wextra
    -ffunction-sections -fdata-sections)

# 启动器只需公共配置（SDK 头文件 + link.lds 静态链接），不需要外设 HAL
target_link_libraries(${APP_NAME} PRIVATE k230_rtos_app)

# 产物路径固定为 build/<APP_NAME>/<APP_NAME>，便于脚本定位
set_target_properties(${APP_NAME} PROPERTIES
    OUTPUT_NAME ${APP_NAME}
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${APP_NAME}")
