cmake_minimum_required(VERSION 3.28)

# ============================================================================
# K230D / K230 RT-Smart 用户态业务应用仓库（与固件无关）
#
# src/ 按功能放 L4 调用与策略；include/ 声明下层接口和实现库，
# runtime/ 保留启动器和独立 IMU 服务。各目录分别声明本目录目标。
# 业务应用 dart_app 与进程启动器 launcher 分别生成 RT-Smart 可执行程序（ELF）。
#
# 构建方式（交叉编译，目标 rv64imafdcv / lp64d，musl 静态链接）：
#   bash .script/build-rtos
# 或手动：
#   cmake -S . -B build \
#         -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/toolchain-riscv64-musl.cmake \
#         -DK230_SDK_ROOT=/mnt/mydata/kRTOSSDK \
#         -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#   cmake --build build
# ============================================================================

project(k230d_rtos_apps LANGUAGES C CXX)

# C data-plane modules use a strict C11 ABI; the orchestration/control plane
# uses C++23 (notably std::expected) with ordinary headers and sources.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include(cmake/targets.cmake)

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
set(K230_SDK_OUTPUT_DIR "" CACHE PATH "SDK output directory for the firmware board")
set(_sdk_output "${K230_SDK_OUTPUT_DIR}")
if(NOT _sdk_output AND EXISTS "${K230_SDK_ROOT}/.config")
  file(STRINGS "${K230_SDK_ROOT}/.config" _board_config
      REGEX "^CONFIG_BOARD_CONFIG_NAME=\"[^\"]+\"$")
  if(_board_config)
    string(REGEX REPLACE "^CONFIG_BOARD_CONFIG_NAME=\"([^\"]+)\"$" "\\1"
        _board_config "${_board_config}")
    set(_sdk_output "${K230_SDK_ROOT}/output/${_board_config}")
  endif()
endif()
if(NOT _sdk_output)
  file(GLOB _hal_roots "${K230_SDK_ROOT}/output/*/rtsmart/libs/rtsmart_hal")
  list(LENGTH _hal_roots _hal_count)
  if(_hal_count GREATER 1)
    message(FATAL_ERROR "Multiple SDK boards found; set K230_SDK_OUTPUT_DIR to the firmware output directory")
  elseif(_hal_count EQUAL 1)
    list(GET _hal_roots 0 _hal_root)
    get_filename_component(_sdk_output "${_hal_root}/../../.." ABSOLUTE)
  endif()
endif()
set(_hal_root "${_sdk_output}/rtsmart/libs/rtsmart_hal")
if(CMAKE_CROSSCOMPILING AND (NOT EXISTS "${_hal_root}/include" OR NOT EXISTS "${_hal_root}/lib"))
  message(FATAL_ERROR "HAL missing for selected SDK board: ${_sdk_output}; check K230_SDK_OUTPUT_DIR")
endif()
if(EXISTS "${_hal_root}/include" AND EXISTS "${_hal_root}/lib")
  target_include_directories(k230_rtos_app INTERFACE "${_hal_root}/include")
  target_link_directories(k230_rtos_app INTERFACE "${_hal_root}/lib")
  # fpioa (also used by SPI) calls utils_map_memory from libutils.
  # Keep the mutually dependent HAL archives in one linker argument. CMake can
  # deduplicate separate group delimiters against the MPP group after target
  # dependencies are flattened, leaving SPI's GPIO/FPIOA references unresolved.
  target_link_libraries(k230_rtos_app INTERFACE
      "-Wl,--start-group,-lpwm,-li2c,-lgpio,-lfpioa,-lspi,-lutils,--end-group")
  if(CMAKE_CROSSCOMPILING)
    # The toolchain's statfs64 bridge returns an empty result on this RT-Smart
    # firmware. Use the SDK's native syscall adapters for statvfs and mutexes.
    if(NOT EXISTS "${_hal_root}/lib/libk230_syscalls.a")
      message(FATAL_ERROR "RT-Smart syscall adapters missing: ${_hal_root}/lib/libk230_syscalls.a")
    endif()
    target_link_libraries(k230_rtos_app INTERFACE k230_syscalls)
  endif()
endif()

# 4) 链接参数：与 SDK 一致 —— 使用 RT-Smart 链接脚本 + 静态链接
set(K230_LINK_SCRIPT "${K230_SDK_ROOT}/src/rtsmart/libs/mk/link.lds" CACHE FILEPATH
    "RT-Smart user application linker script")
target_link_options(k230_rtos_app INTERFACE
    "-T${K230_LINK_SCRIPT}"
    --static
    -Wl,--gc-sections
    # Clang's default build-id note would precede .text and move _start by 0x40.
    -Wl,--build-id=none)

# ---------------------------------------------------------------------------
# 声明公共抽象/设备库、运行时服务，再声明业务入口、编排与领域目标
# ---------------------------------------------------------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/components/peripherals/CMakeLists.txt")
  add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/components/peripherals")
endif()
# include/ 下集中声明全部固件侧库目标（camera / spi / imu …）；
# 模块子目录只放源码，不再各自持有 CMakeLists.txt。
find_package(Threads REQUIRED)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/include")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/runtime")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src")

if(CMAKE_CROSSCOMPILING)
  add_subdirectory(tools/pwm_swing EXCLUDE_FROM_ALL)
endif()

include(CTest)
if(BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
  add_subdirectory(tests)
endif()

message(STATUS "K230_SDK_ROOT        = ${K230_SDK_ROOT}")
message(STATUS "K230 SDK output      = ${_sdk_output}")
message(STATUS "K230_TOOLCHAIN_DIR   = ${K230_TOOLCHAIN_DIR}")
