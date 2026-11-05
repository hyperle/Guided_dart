# ===========================================================================
# include/ —— 当前 L3/L2 接口和实现库目标
#
# 当前 camera/、spi/、imu/ 等下层库目标集中在本文件声明。
# 各目录是否需要说明文档，按实际使用需要决定。
# 本文件里的路径一律以 include/ 为基准，通过下面的模块目录变量拼接。
#
# 节的先后顺序有意义：spi/ 声明 DART_IMU_K230 并提供 dart_iim42652，imu/ 的
# C++ 适配器链接它，因此 spi 一节必须排在 imu 之前。
# ===========================================================================

set(_dart_include "${CMAKE_CURRENT_SOURCE_DIR}/firmware/include")
set(_dart_src "${CMAKE_CURRENT_SOURCE_DIR}/firmware/src")
set(_camera_dir "${_dart_src}/camera")
set(_spi_dir "${_dart_src}/spi")
set(_imu_dir "${_dart_src}/imu")

# ---------------------------------------------------------------------------
# camera —— C 数据面（C ABI 状态机）+ K230 数据面 + C++ 适配器（ICamera）
# ---------------------------------------------------------------------------
option(DART_CAMERA_K230 "Build the K230 camera data plane and C++ adapter" OFF)

# Read-only frame types are header-only; no device/SDK implementation.
add_library(dart_camera_types INTERFACE)
target_include_directories(dart_camera_types INTERFACE "${PROJECT_SOURCE_DIR}" "${_dart_src}")
target_compile_features(dart_camera_types INTERFACE cxx_std_23)

# Every enabled C backend contains real sources; a missing file fails configure.
function(_dart_camera_add_library target)
    set(_sources)
    foreach(_source IN LISTS ARGN)
        list(APPEND _sources "${_camera_dir}/${_source}")
    endforeach()
    add_library(${target} STATIC ${_sources})
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    target_include_directories(${target} PUBLIC "${_dart_include}" "${_dart_src}")
endfunction()

_dart_camera_add_library(dart_camera_core c_api/config.c c_api/lease.c c_api/raw10.c)
target_compile_features(dart_camera_core PUBLIC c_std_11)

# Editor support: clangd can only use sources that have a compile command, and a
# source without one gets a flag-less fallback command in which every vendor
# header looks missing. The K230 data plane is only compiled by the cross
# preset, so when it is off the SDK include paths would never reach
# compile_commands.json. This target exists purely to emit those commands: it is
# EXCLUDE_FROM_ALL (never built, never linked, no artifact) and carries exactly
# the same include set as dart_camera_k230. The SDK paths are resolved by
# k230_rtos_app, so nothing is hardcoded here.
option(DART_CAMERA_CLANGD_COMMANDS
    "Emit compile commands for SDK-dependent sources so editors can parse them" ON)
if(DART_CAMERA_CLANGD_COMMANDS AND NOT DART_CAMERA_K230)
    add_library(dart_camera_k230_compile_commands OBJECT EXCLUDE_FROM_ALL
        "${_camera_dir}/k230/lifecycle.c"
        "${_camera_dir}/k230/media.c"
        "${_camera_dir}/k230/frame.c"
        "${_camera_dir}/k230/mcm.c"
        "${_camera_dir}/k230/exposure.c")
    target_link_libraries(dart_camera_k230_compile_commands PRIVATE k230_rtos_app)
    target_compile_features(dart_camera_k230_compile_commands PRIVATE c_std_11)
    target_compile_options(dart_camera_k230_compile_commands PRIVATE -Wall -Wextra -Wpedantic)
    target_include_directories(dart_camera_k230_compile_commands PRIVATE
        "${_dart_include}"
        "${_dart_src}")
    # A header has no compile command of its own, so clangd borrows one from a
    # neighbouring file. Marking k230/internal.h as a compilable source gives it
    # the SDK include paths deterministically, instead of depending on which
    # neighbour clangd happens to pick.
    set_source_files_properties("${_camera_dir}/k230/internal.h" PROPERTIES LANGUAGE C)
    target_sources(dart_camera_k230_compile_commands PRIVATE
        "${_camera_dir}/k230/internal.h")
endif()

if(DART_CAMERA_K230)
    if(NOT CMAKE_CROSSCOMPILING OR NOT CMAKE_SYSTEM_PROCESSOR MATCHES "riscv")
        message(FATAL_ERROR "DART_CAMERA_K230 requires a RISC-V cross toolchain")
    endif()
    set(K230_MPP_LIB_DIR "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/lib"
        CACHE PATH "K230 MPP static library directory")
    foreach(_library vicap vb sensor isp sys framework)
        if(NOT EXISTS "${K230_MPP_LIB_DIR}/lib${_library}.a")
            message(FATAL_ERROR "Missing K230 MPP library: lib${_library}.a")
        endif()
    endforeach()
    # The backend owns the audited private ABI. The standalone probe reads the
    # same manifest; changing either SDK library must re-run this check.
    set(_mcm_abi_path "${_camera_dir}/k230/mcm_abi.json")
    file(READ "${_mcm_abi_path}" _mcm_abi)
    string(JSON _checked_vicap_user GET "${_mcm_abi}" "userapps/lib/libvicap.a")
    string(JSON _checked_vicap_kernel GET "${_mcm_abi}" "kernel/lib/libvicap.a")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
        "${_mcm_abi_path}"
        "${K230_MPP_LIB_DIR}/libvicap.a"
        "${K230_SDK_ROOT}/src/rtsmart/mpp/kernel/lib/libvicap.a")
    file(SHA256 "${K230_MPP_LIB_DIR}/libvicap.a" _vicap_user_hash)
    file(SHA256 "${K230_SDK_ROOT}/src/rtsmart/mpp/kernel/lib/libvicap.a" _vicap_kernel_hash)
    if(NOT _vicap_user_hash STREQUAL _checked_vicap_user
       OR NOT _vicap_kernel_hash STREQUAL _checked_vicap_kernel)
        message(FATAL_ERROR "VICAP libraries changed; review the RAW10 MCM ioctl ABI before updating ${_mcm_abi_path}")
    endif()
    if(NOT EXISTS "${K230_LINK_SCRIPT}")
        message(FATAL_ERROR "Missing RT-Smart linker script: ${K230_LINK_SCRIPT}")
    endif()
    _dart_camera_add_library(dart_camera_k230
        k230/lifecycle.c k230/media.c k230/frame.c k230/exposure.c k230/mcm.c)
    target_link_options(dart_camera_k230 INTERFACE -Wl,--wrap=ioctl)
    target_link_libraries(dart_camera_k230 PUBLIC dart_camera_core)
    target_link_libraries(dart_camera_k230 PRIVATE k230_rtos_app
        -Wl,--start-group
        "${K230_MPP_LIB_DIR}/libvicap.a"
        "${K230_MPP_LIB_DIR}/libvb.a"
        "${K230_MPP_LIB_DIR}/libsensor.a"
        "${K230_MPP_LIB_DIR}/libisp.a"
        "${K230_MPP_LIB_DIR}/libsys.a"
        "${K230_MPP_LIB_DIR}/libframework.a"
        -Wl,--end-group pthread m)
endif()

dart_add_cpp_library(dart_camera_acl
    "${_camera_dir}/error.cpp" "${_camera_dir}/frame_lease.cpp"
    "${_camera_dir}/interface.hpp" "${_camera_dir}/k230/camera.hpp")
target_include_directories(dart_camera_acl PUBLIC "${_dart_include}")
target_link_libraries(dart_camera_acl PUBLIC dart_camera_types dart_camera_core)
if(DART_CAMERA_K230)
    # The template adapter is compiled by its consumers. Propagate the
    # firmware routing configuration together with its C backend dependency.
    target_link_libraries(dart_camera_acl PUBLIC dart_camera_k230 k230_rtos_app)
endif()

# ---------------------------------------------------------------------------
# spi —— 底层 C 封装：IIM-42652 SPI 硬件驱动
#
# iim42652.h 是稳定 C ABI（只有固定宽度整数和 POD），iim42652.c 是本仓库唯一
# 包含 K230 SDK 厂商头（drv_spi）的翻译单元，负责 SPI 总线
# 句柄、寄存器配置、原始值换算和厂商状态码翻译。本模块不引入 C++。
# ---------------------------------------------------------------------------
option(DART_IMU_K230 "Build the K230 IIM-42652 SPI backend" OFF)

if(DART_IMU_K230)
    if(NOT CMAKE_CROSSCOMPILING OR NOT CMAKE_SYSTEM_PROCESSOR MATCHES "riscv")
        message(FATAL_ERROR "DART_IMU_K230 requires a RISC-V cross toolchain")
    endif()
    add_library(dart_iim42652 STATIC "${_spi_dir}/iim42652.c")
    # 驱动按 C 数据面约束编译：显式 gnu11，严格告警。
    target_compile_options(dart_iim42652 PRIVATE -Wall -Wextra -Wpedantic -std=gnu11)
    target_include_directories(dart_iim42652 PUBLIC "${_dart_src}")
    target_link_libraries(dart_iim42652 PRIVATE k230_rtos_app m)
endif()

# ---------------------------------------------------------------------------
# imu —— 设备抽象（IImu）、估计器与 K230 适配器
#
# 纯 C++，不含厂商头文件：
#   sample.hpp / interface.hpp   IMU 数据类与抽象接口
#   estimator.cpp               姿态/速度估计器，纯计算、无分配
#   k230/iim42652.cpp            C 驱动到 IImu 的适配器
# ---------------------------------------------------------------------------
dart_add_cpp_library(dart_imu
    "${_imu_dir}/math_types.hpp" "${_imu_dir}/sample.hpp"
    "${_imu_dir}/interface.hpp" "${_imu_dir}/estimator.cpp")
if(DART_IMU_K230)
    dart_add_cpp_library(dart_imu_k230 "${_imu_dir}/k230/iim42652.cpp")
    target_link_libraries(dart_imu_k230 PUBLIC dart_imu dart_iim42652)
endif()

# Editor support: spi/iim42652.c is only built by the cross preset, so with
# DART_IMU_K230 off it needs an editor-only compile command. C++ implementations already
# receive compile commands from their libraries. A borrowed command lacks the SDK
# include paths nor -std=gnu11, so drv_spi.h, drv_spi_inst_t, SPI_HAL_MODE_3 and
# the bool literals all look undeclared in the editor while the real build is
# clean. This target exists purely to emit those commands: it is
# EXCLUDE_FROM_ALL (never built, never linked, no artifact) and carries the same
# include set and flags as dart_iim42652 / dart_imu_k230. SDK paths are resolved
# by k230_rtos_app, so nothing is hardcoded here.
option(DART_IMU_CLANGD_COMMANDS
    "Emit compile commands for SDK-dependent IMU sources so editors can parse them" ON)
if(DART_IMU_CLANGD_COMMANDS AND NOT DART_IMU_K230)
    add_library(dart_imu_k230_compile_commands OBJECT EXCLUDE_FROM_ALL
        "${_spi_dir}/iim42652.c")
    target_link_libraries(dart_imu_k230_compile_commands PRIVATE k230_rtos_app)
    target_compile_features(dart_imu_k230_compile_commands PRIVATE cxx_std_23)
    target_compile_options(dart_imu_k230_compile_commands PRIVATE
        -Wall -Wextra -Wpedantic
        $<$<COMPILE_LANGUAGE:C>:-std=gnu11>
        $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti>)
    target_include_directories(dart_imu_k230_compile_commands PRIVATE "${_dart_src}")
    # A header has no compile command of its own, so clangd borrows one from a
    # neighbouring file. Declaring the C driver header as a source gives it this
    # target's include set deterministically instead of depending on which
    # neighbour clangd happens to pick.
    set_source_files_properties("${_spi_dir}/iim42652.h" PROPERTIES LANGUAGE C)
    target_sources(dart_imu_k230_compile_commands PRIVATE
        "${_spi_dir}/iim42652.h")
endif()

# ---------------------------------------------------------------------------
# PWM: platform-neutral device interface and independent output task.
# Vendor headers stay in the board backend, below L4.
# ---------------------------------------------------------------------------
option(DART_PWM_K230 "Build the K230 four-channel servo PWM backend" ${DART_CAMERA_K230})

# Process, filesystem and logging modules do not select any device backend.
file(GLOB_RECURSE _platform_sources CONFIGURE_DEPENDS
    "${_dart_src}/system/*.cpp" "${_dart_src}/logging/*.cpp" "${_dart_src}/storage/*.cpp")
dart_add_cpp_library(dart_platform ${_platform_sources})
target_link_libraries(dart_platform PUBLIC Threads::Threads)

dart_add_cpp_library(dart_pwm "${_dart_src}/pwm/interface.cpp" "${_dart_src}/pwm/session.cpp")
target_link_libraries(dart_pwm PUBLIC dart_platform Threads::Threads)
if(DART_PWM_K230)
    if(NOT CMAKE_CROSSCOMPILING OR NOT CMAKE_SYSTEM_PROCESSOR MATCHES "riscv")
        message(FATAL_ERROR "DART_PWM_K230 requires a RISC-V cross toolchain")
    endif()
    dart_add_cpp_library(dart_pwm_k230 "${_dart_src}/pwm/k230/output.cpp")
    target_link_libraries(dart_pwm_k230 PUBLIC dart_pwm PRIVATE k230_rtos_app)
endif()

# Device callback contracts depend on interfaces, keeping PWM/platform acyclic.
add_library(dart_device_services INTERFACE)
target_include_directories(dart_device_services INTERFACE "${PROJECT_SOURCE_DIR}" "${_dart_src}")
target_link_libraries(dart_device_services INTERFACE dart_camera_acl dart_imu dart_pwm dart_platform)

file(GLOB_RECURSE _recording_sources CONFIGURE_DEPENDS "${_dart_src}/recorder/*.cpp")
dart_add_cpp_library(dart_recording ${_recording_sources})
add_library(dart_recorder ALIAS dart_recording)
if(CMAKE_CROSSCOMPILING)
    set(DART_ZLIB_INCLUDE_DIR "${K230_SDK_ROOT}/src/rtsmart/libs/3rd-party/libwebsockets/libwebsockets/win32port/zlib"
        CACHE PATH "SDK zlib headers (stable zlib 1.x ABI)")
    set(DART_ZLIB_LIBRARY "${K230_SDK_ROOT}/src/rtsmart/libs/opencv/lib/opencv4/3rdparty/libzlib.a"
        CACHE FILEPATH "RT-Smart zlib static library")
    if(NOT EXISTS "${DART_ZLIB_INCLUDE_DIR}/zlib.h" OR NOT EXISTS "${DART_ZLIB_LIBRARY}")
        message(FATAL_ERROR "PNG recording needs DART_ZLIB_INCLUDE_DIR and DART_ZLIB_LIBRARY")
    endif()
    add_library(dart_zlib INTERFACE)
    target_include_directories(dart_zlib SYSTEM INTERFACE "${DART_ZLIB_INCLUDE_DIR}")
    target_link_libraries(dart_zlib INTERFACE "${DART_ZLIB_LIBRARY}")
else()
    find_package(ZLIB REQUIRED)
    add_library(dart_zlib ALIAS ZLIB::ZLIB)
endif()
target_link_libraries(dart_recording PUBLIC dart_camera_acl dart_platform Threads::Threads dart_zlib)

# JPEG CPU data plane and bounded streaming workers. Device selection is injected.
add_library(dart_streaming_data STATIC
    "${_dart_src}/streaming/jpeg.c" "${_dart_src}/streaming/http.c")
target_include_directories(dart_streaming_data PUBLIC "${_dart_include}" "${_dart_src}")
target_compile_features(dart_streaming_data PUBLIC c_std_11)
target_compile_options(dart_streaming_data PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(dart_streaming_data PUBLIC m)
dart_add_cpp_library(dart_streaming "${_dart_src}/streaming/interface.cpp"
    "${_dart_src}/streaming/session.cpp")
target_link_libraries(dart_streaming PUBLIC dart_camera_types dart_streaming_data Threads::Threads)
option(DART_WIFI_K230 "Build the SDIO Wi-Fi streaming transport" ${DART_CAMERA_K230})
if(DART_WIFI_K230)
    if(NOT CMAKE_CROSSCOMPILING OR NOT CMAKE_SYSTEM_PROCESSOR MATCHES "riscv")
        message(FATAL_ERROR "DART_WIFI_K230 requires a RISC-V cross toolchain")
    endif()
    add_library(dart_wifi_radio STATIC "${_dart_src}/streaming/k230/radio.c")
    target_link_libraries(dart_wifi_radio PRIVATE k230_rtos_app)
    target_include_directories(dart_wifi_radio PRIVATE "${_dart_src}")
    target_include_directories(dart_wifi_radio SYSTEM PRIVATE
        "${_hal_root}/include"
        "${K230_SDK_ROOT}/src/rtsmart/libs/rtsmart_hal/components/netmgmt")
    target_compile_options(dart_wifi_radio PRIVATE -Wall -Wextra -Wpedantic)
    dart_add_cpp_library(dart_streaming_k230 "${_dart_src}/streaming/k230/transport.cpp")
    target_link_libraries(dart_streaming_k230 PUBLIC dart_streaming
        PRIVATE dart_wifi_radio k230_rtos_app netmgmt)
endif()

# The composition root selects actual backends only when they are enabled.
dart_add_cpp_library(dart_board "${_dart_src}/board/k230.cpp")
target_link_libraries(dart_board PUBLIC dart_device_services dart_imu_logging dart_streaming)
if(DART_WIFI_K230)
    target_compile_definitions(dart_board PRIVATE DART_APP_WIFI_K230=1)
    target_link_libraries(dart_board PUBLIC dart_streaming_k230)
endif()
if(DART_PWM_K230)
    target_compile_definitions(dart_board PRIVATE DART_APP_PWM_K230=1)
    target_link_libraries(dart_board PUBLIC dart_pwm_k230)
endif()
if(DART_IMU_K230)
    target_compile_definitions(dart_board PRIVATE DART_APP_IMU_K230=1)
    target_link_libraries(dart_board PUBLIC dart_imu_k230)
endif()
