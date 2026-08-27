# ============================================================================
# 业务应用骨架示例（最小范式，无业务代码）
#
# ★ 你的业务代码放在这里：apps/<你的应用名>/  目录下 ★
#
# 新增业务应用的步骤：
#   1. 拷贝本目录：cp -r apps/app_template apps/my_app
#   2. 修改下方 APP_NAME 为你的应用名（建议小写、下划线分隔，如 my_app）
#   3. 在 apps/my_app/ 下编写业务代码（main.c / 任意 .c .cpp）
#   4. 编译：bash scripts/build.sh          （产物 build/my_app/my_app）
#   5. 部署：bash scripts/deploy.sh my_app <板子IP>
#   6. 板端运行：msh> /tmp/my_app
#
# 说明：本目录下所有 *.c / *.cpp 都会自动参与编译；公共头文件路径、
#       RT-Smart 链接脚本等由顶层 CMakeLists 的 k230_rtos_app 接口目标提供。
# ============================================================================

set(APP_NAME app_template)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

add_executable(${APP_NAME} ${APP_SOURCES})

# 与 SDK 一致的告警级别（必要参数 -march/-mabi/-mcmodel 已在工具链文件中设置）
target_compile_options(${APP_NAME} PRIVATE
    -Wall -Wextra
    -ffunction-sections -fdata-sections)

# 链接公共配置（SDK 头文件 + link.lds 静态链接）
target_link_libraries(${APP_NAME} PRIVATE k230_rtos_app)

# 输出 ELF 文件名即 APP_NAME（与 SDK 生成的板端可执行文件同名）
# 产物路径固定为 build/<APP_NAME>/<APP_NAME>，便于脚本定位
set_target_properties(${APP_NAME} PROPERTIES
    OUTPUT_NAME ${APP_NAME}
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${APP_NAME}")
