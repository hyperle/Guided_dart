# Concrete runtime services and process supervision, outside the L4 domain.
# Module subdirectories contain sources and README files; targets live here.

# IMU acquisition, shared estimator state and asynchronous raw/estimate log.
dart_add_cpp_library(dart_imu_logging imu/session.cpp)
target_link_libraries(dart_imu_logging PUBLIC dart_imu dart_platform Threads::Threads)

# Application assembly and concrete entry are runtime concerns.
dart_add_cpp_library(dart_application application.cpp argument_parser.cpp recording_trigger.cpp)
target_link_libraries(dart_application PUBLIC dart_detection dart_control dart_recording
    dart_imu_logging dart_pwm dart_platform dart_device_services dart_streaming Threads::Threads)

# The process entry parses hardware options. Neither the application nor its
# business components depend on this board-specific argument bundle.
dart_add_cpp_library(dart_board_arguments
    "${PROJECT_SOURCE_DIR}/include/firmware/src/board/k230_arguments.cpp")
target_link_libraries(dart_board_arguments PUBLIC dart_application dart_camera_k230_adapter)

if(DART_CAMERA_K230)
    add_executable(dart_app dart_entry.cpp)
    target_compile_options(dart_app PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(dart_app PRIVATE dart_application dart_board dart_board_arguments Threads::Threads)
    set_target_properties(dart_app PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dart_app")

    # Finite live-camera timing comparison. Run alone before dart_app.
    add_executable(vision_probe vision_probe.cpp)
    target_compile_options(vision_probe PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_link_libraries(vision_probe PRIVATE dart_detection dart_board dart_platform Threads::Threads)
    set_target_properties(vision_probe PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/vision_probe")

    # Exclusive sensor/receiver timing probe; restores documented HTS/VTS.
    add_executable(camera_timing_probe camera_timing_probe.c
        "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/src/sensor/mpi_sensor.c")
    # The shared SDK library can be rebuilt for another board. Compile its
    # original sensor MPI with this diagnostic's verified CSI2/SC038HGS mode.
    set_source_files_properties(
        "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/src/sensor/mpi_sensor.c"
        PROPERTIES COMPILE_OPTIONS
        "-include${CMAKE_CURRENT_SOURCE_DIR}/camera_timing_config.h;-Wno-error=overflow;-Wno-error=unused-variable")
    target_compile_options(camera_timing_probe PRIVATE -Wall -Wextra -Wpedantic -Werror)
    target_link_libraries(camera_timing_probe PRIVATE dart_camera_k230 k230_rtos_app)
    set_target_properties(camera_timing_probe PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/camera_timing_probe")
endif()

# Finite board diagnostic for initialization timing comparisons. Run alone.
if(DART_IMU_K230)
    add_executable(imu_init_probe imu/init_probe.c)
    target_compile_options(imu_init_probe PRIVATE -Wall -Wextra -Werror)
    target_link_libraries(imu_init_probe PRIVATE dart_iim42652 k230_rtos_app m)
    set_target_properties(imu_init_probe PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/imu_init_probe")

    # Continuous SPI scope test. Deliberately does not link the IMU driver.
    add_executable(imu_raw_probe imu/raw_probe.cpp)
    target_compile_options(imu_raw_probe PRIVATE
        -Wall -Wextra -Wpedantic -Werror -fno-exceptions -fno-rtti)
    target_link_libraries(imu_raw_probe PRIVATE dart_platform k230_rtos_app)
    set_target_properties(imu_raw_probe PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/imu_raw_probe")
endif()

# ---------------------------------------------------------------------------
# launcher：开机自启动器（平台无关）
#
# 只做"读清单 → fork/execv 拉起 → 监督回收"，不认识相机、录像或任何 SDK。
# host 也能构建与运行，便于在电脑上用假清单验证解析、前后台与子日志行为。
# ---------------------------------------------------------------------------
add_executable(launcher launcher/main.c)
target_include_directories(launcher PRIVATE "${PROJECT_SOURCE_DIR}/include/firmware/src")
target_compile_features(launcher PRIVATE c_std_11)
target_compile_options(launcher PRIVATE -Wall -Wextra -Wpedantic
    -ffunction-sections -fdata-sections)
if(CMAKE_CROSSCOMPILING)
    # 板端 ELF 必须链接 k230_rtos_app：它提供 RT-Smart 链接脚本（-T link.lds）
    # 与 -static。少了它就会链成动态 ELF（interpreter /lib/ld-musl-riscv64.so.1、
    # 入口不是 0x200000000），RT-Smart 用户态加载不了。host 构建不需要。
    target_link_libraries(launcher PRIVATE k230_rtos_app)
endif()
set_target_properties(launcher PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/launcher")

