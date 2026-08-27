# ============================================================================
# K230D / K230 RT-Smart 用户态应用交叉编译工具链
#
# 工具链：riscv64-unknown-linux-musl-（RT-Smart 用户态 musl 工具链）
#   安装方式：bash scripts/setup-toolchain.sh
#   默认位置：${HOME}/.kendryte/k230_toolchains/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin
#   或 SDK 内：${K230_SDK_ROOT}/tools/toolchain/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin
#
# 目标架构：-march=rv64imafdcv -mabi=lp64d -mcmodel=medany
#   （与 SDK src/applications/helloworld 的编译参数完全一致）
# ============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv64)

# 避免交叉编译时执行目标机程序（try_run），统一改为静态库试编译
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(K230_SDK_ROOT "" CACHE PATH "K230 RTOS SDK 根目录（用于定位工具链）")
set(K230_TOOLCHAIN_DIR "" CACHE PATH "riscv64 musl 工具链 bin 目录（一般无需手动指定）")

# ---- 自动探测工具链位置 ----
# 优先级：仓库自带(third_party) > SDK 内置 > SDK 默认安装目录(~/.kendryte)
if(NOT K230_TOOLCHAIN_DIR)
  foreach(_cand
      "${CMAKE_CURRENT_LIST_DIR}/../third_party/toolchain/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin"
      "${K230_SDK_ROOT}/tools/toolchain/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin"
      "$ENV{HOME}/.kendryte/k230_toolchains/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin")
    if(EXISTS "${_cand}/riscv64-unknown-linux-musl-gcc")
      set(K230_TOOLCHAIN_DIR "${_cand}")
      break()
    endif()
  endforeach()
endif()

if(NOT EXISTS "${K230_TOOLCHAIN_DIR}/riscv64-unknown-linux-musl-gcc")
  message(FATAL_ERROR
    "未找到 riscv64 musl 工具链：${K230_TOOLCHAIN_DIR}\n"
    "请先运行 bash scripts/setup-toolchain.sh 安装工具链，"
    "或通过 -DK230_TOOLCHAIN_DIR=... 指定工具链 bin 目录。")
endif()

set(_TC_PREFIX "${K230_TOOLCHAIN_DIR}/riscv64-unknown-linux-musl-")
set(CMAKE_C_COMPILER   "${_TC_PREFIX}gcc")
if(EXISTS "${_TC_PREFIX}g++")
  set(CMAKE_CXX_COMPILER "${_TC_PREFIX}g++")
endif()
set(CMAKE_AR            "${_TC_PREFIX}ar")
set(CMAKE_RANLIB        "${_TC_PREFIX}ranlib")
set(CMAKE_STRIP         "${_TC_PREFIX}strip")
set(CMAKE_OBJCOPY       "${_TC_PREFIX}objcopy")
set(CMAKE_OBJDUMP       "${_TC_PREFIX}objdump")
set(CMAKE_NM            "${_TC_PREFIX}nm")

# 只允许在工具链目录及其 sysroot 中查找头文件/库，避免误用宿主环境
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# ---- 目标架构编译参数（与 SDK 一致）----
set(CMAKE_C_FLAGS_INIT   "-march=rv64imafdcv -mabi=lp64d -mcmodel=medany -Os")
set(CMAKE_CXX_FLAGS_INIT "-march=rv64imafdcv -mabi=lp64d -mcmodel=medany -Os")

# 关闭 CMake 对交叉编译器的默认试链接干扰
set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_CXX_COMPILER_WORKS TRUE)
