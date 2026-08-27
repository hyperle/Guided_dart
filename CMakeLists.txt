cmake_minimum_required(VERSION 3.20)

# ============================================================================
# K230D / K230 RT-Smart 用户态业务应用仓库（与固件无关）
#
# 业务代码统一放在 apps/<应用名>/ 目录下，每个子目录编译为一个
# 可在板端 RT-Smart 中直接运行的可执行程序（ELF）。
#
# 构建方式（交叉编译，目标 rv64imafdcv / lp64d，musl 静态链接）：
#   bash scripts/build.sh
# 或手动：
#   cmake -S . -B build \
#         -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/toolchain-riscv64-musl.cmake \
#         -DK230_SDK_ROOT=/mnt/mydata/kRTOSSDK \
#         -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#   cmake --build build
# ============================================================================

project(k230d_rtos_apps LANGUAGES C)

# ---------------------------------------------------------------------------
# K230 RTOS SDK 根目录（本机默认；若 SDK 在其他路径，用 -DK230_SDK_ROOT=... 覆盖）
# 作用：提供 SDK 头文件（include/、mpp/、rtsmart_hal 等）供业务代码调用。
# 注意：本仓库只“借用”SDK 的头文件/链接脚本，不参与固件编译。
# ---------------------------------------------------------------------------
set(K230_SDK_ROOT "/mnt/mydata/kRTOSSDK" CACHE PATH
    "K230 RTOS SDK 根目录（用于获取 SDK 头文件与链接脚本）")

# ---------------------------------------------------------------------------
# 公共配置：所有业务应用共享的编译/链接参数（对应 SDK helloworld 的写法）
# ---------------------------------------------------------------------------
add_library(k230_rtos_app INTERFACE)

# 1) SDK 根 include（含 generated/autoconf.h，与 SDK 应用编译一致）
if(EXISTS "${K230_SDK_ROOT}/include")
  target_include_directories(k230_rtos_app INTERFACE "${K230_SDK_ROOT}/include")
  if(EXISTS "${K230_SDK_ROOT}/include/generated/autoconf.h")
    target_compile_options(k230_rtos_app INTERFACE
        "-include${K230_SDK_ROOT}/include/generated/autoconf.h")
  endif()
endif()

# 2) MPP（多媒体处理平台）头文件 —— 用不到多媒体 API 时可忽略
foreach(_inc
    "${K230_SDK_ROOT}/src/rtsmart/mpp/include"
    "${K230_SDK_ROOT}/src/rtsmart/mpp/include/comm"
    "${K230_SDK_ROOT}/src/rtsmart/mpp/include/ioctl"
    "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/api"
    "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/api/framework")
  if(EXISTS "${_inc}")
    target_include_directories(k230_rtos_app INTERFACE "${_inc}")
  endif()
endforeach()

# 3) rtsmart_hal（外设 HAL 驱动头文件，随板卡配置输出）
file(GLOB _hal_incs "${K230_SDK_ROOT}/output/*/rtsmart/libs/rtsmart_hal/include")
foreach(_inc IN LISTS _hal_incs)
  target_include_directories(k230_rtos_app INTERFACE "${_inc}")
endforeach()

# 4) 链接参数：与 SDK 一致 —— 使用 RT-Smart 链接脚本 + 静态链接
target_link_options(k230_rtos_app INTERFACE
    "-T${CMAKE_CURRENT_SOURCE_DIR}/cmake/link.lds"
    --static
    -Wl,--gc-sections)

# ---------------------------------------------------------------------------
# 收集 apps/ 下所有业务应用（每个子目录 = 一个板端可执行程序）
# ---------------------------------------------------------------------------
file(GLOB _app_dirs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/apps/*")
foreach(_dir IN LISTS _app_dirs)
  if(IS_DIRECTORY "${_dir}" AND EXISTS "${_dir}/CMakeLists.txt")
    add_subdirectory("${_dir}")
  endif()
endforeach()

message(STATUS "K230_SDK_ROOT        = ${K230_SDK_ROOT}")
message(STATUS "K230_TOOLCHAIN_DIR   = ${K230_TOOLCHAIN_DIR}")
