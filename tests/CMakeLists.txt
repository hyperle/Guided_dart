if(CMAKE_CROSSCOMPILING)
    return()
endif()

add_executable(camera_raw10_test camera_raw10_test.c)
target_link_libraries(camera_raw10_test PRIVATE dart_camera_core)
target_compile_options(camera_raw10_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME camera_raw10 COMMAND camera_raw10_test)

add_executable(canmv_detector_bridge_test canmv_detector_bridge_test.cpp)
target_link_libraries(canmv_detector_bridge_test PRIVATE dart_canmv_detector)
target_compile_options(canmv_detector_bridge_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
add_test(NAME canmv_detector_bridge COMMAND canmv_detector_bridge_test)

foreach(_test recorder_session recorder_binary detector_behavior detector_tracking camera_contract camera_adapter imu_estimator imu_session argument_parser service_isolation binary_pipeline write_control)
    add_executable(${_test}_test "${_test}_test.cpp")
    target_link_libraries(${_test}_test PRIVATE dart_detection dart_recording dart_imu dart_imu_logging dart_platform dart_application)
    target_compile_options(${_test}_test PRIVATE -Werror -UNDEBUG)
    add_test(NAME ${_test} COMMAND ${_test}_test)
endforeach()

target_link_options(imu_session_test PRIVATE -Wl,--wrap=fwrite)

add_executable(servo_mapping_test servo_mapping_test.cpp)
target_link_libraries(servo_mapping_test PRIVATE dart_control)
target_compile_options(servo_mapping_test PRIVATE -Werror -UNDEBUG)
add_test(NAME servo_mapping COMMAND servo_mapping_test)

add_executable(visual_servo_test visual_servo_test.cpp)
target_link_libraries(visual_servo_test PRIVATE dart_application)
target_compile_options(visual_servo_test PRIVATE -Werror -UNDEBUG)
add_test(NAME visual_servo COMMAND visual_servo_test)

add_executable(recorder_storage_test recorder_storage_test.cpp)
target_link_libraries(recorder_storage_test PRIVATE dart_recording)
target_compile_options(recorder_storage_test PRIVATE -Werror -UNDEBUG)
target_link_options(recorder_storage_test PRIVATE -Wl,--wrap=statvfs -Wl,--wrap=fsync)
add_test(NAME recorder_storage COMMAND recorder_storage_test)

add_executable(recorder_timeout_test recorder_timeout_test.cpp)
target_link_libraries(recorder_timeout_test PRIVATE dart_recording)
target_compile_options(recorder_timeout_test PRIVATE -Werror -UNDEBUG)
target_link_options(recorder_timeout_test PRIVATE -Wl,--wrap=fsync)
add_test(NAME recorder_timeout COMMAND recorder_timeout_test)
set_tests_properties(recorder_timeout PROPERTIES TIMEOUT 20)

find_package(PNG REQUIRED)
add_executable(recorder_png_test recorder_png_test.cpp)
target_link_libraries(recorder_png_test PRIVATE dart_recording PNG::PNG)
target_compile_options(recorder_png_test PRIVATE -Werror -UNDEBUG)
target_link_options(recorder_png_test PRIVATE -Wl,--wrap=fsync)
add_test(NAME recorder_png COMMAND recorder_png_test)

set(_spi_hal_include "${K230_SDK_ROOT}/src/rtsmart/libs/rtsmart_hal/drivers/spi")
if(EXISTS "${_spi_hal_include}/drv_spi.h")
    add_executable(imu_driver_test imu_driver_test.c
        "${PROJECT_SOURCE_DIR}/include/firmware/src/spi/iim42652.c")
    target_include_directories(imu_driver_test PRIVATE
        "${PROJECT_SOURCE_DIR}/include/firmware/src" "${_spi_hal_include}")
    target_compile_options(imu_driver_test PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    target_link_libraries(imu_driver_test PRIVATE m)
    target_link_options(imu_driver_test PRIVATE -Wl,--wrap=nanosleep -Wl,--wrap=clock_gettime)
    add_test(NAME imu_driver COMMAND imu_driver_test)

    add_library(imu_raw_probe_under_test OBJECT "${PROJECT_SOURCE_DIR}/runtime/imu/raw_probe.cpp")
    target_compile_definitions(imu_raw_probe_under_test PRIVATE main=imu_raw_probe_main)
    target_link_libraries(imu_raw_probe_under_test PRIVATE dart_platform)
    target_include_directories(imu_raw_probe_under_test PRIVATE "${_spi_hal_include}")
    target_compile_options(imu_raw_probe_under_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_executable(imu_raw_probe_test imu_raw_probe_test.cpp)
    target_link_libraries(imu_raw_probe_test PRIVATE imu_raw_probe_under_test)
    target_include_directories(imu_raw_probe_test PRIVATE "${_spi_hal_include}")
    target_compile_options(imu_raw_probe_test PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
    target_link_options(imu_raw_probe_test PRIVATE -Wl,--wrap=nanosleep)
    foreach(_scenario success read_failure bus_failure sleep_failure
            bank_failure wake_failure wake_short_write wake_sleep_failure
            wake_sleep_interrupt wake_stop wake_read_failure wake_mismatch
            power_save_read_failure power_save_mismatch)
        add_test(NAME imu_raw_probe_${_scenario} COMMAND imu_raw_probe_test ${_scenario})
    endforeach()

    add_executable(imu_gyro_probe_test imu_gyro_probe_test.cpp)
    target_link_libraries(imu_gyro_probe_test PRIVATE imu_raw_probe_under_test)
    target_include_directories(imu_gyro_probe_test PRIVATE "${_spi_hal_include}")
    target_compile_options(imu_gyro_probe_test PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
    target_link_options(imu_gyro_probe_test PRIVATE -Wl,--wrap=nanosleep)
    foreach(_scenario invalid ready reset_ready power_save_mismatch rc_ready warm_on fifo_stream
            axes_disabled bad_id bad_format bank_mismatch restore_failure off_failure interrupt_wait sample_io_failure stop
            external_ready external_auto_ready external_int1_ready external_vendor_ready external_warm_on external_invalid external_pin_mismatch
            external_rtc_mismatch external_restore_failure external_pin_write_failure external_rtc_write_failure
            external_int1_write_failure external_int1_mismatch external_vendor_mismatch
            external_interrupt_off external_stop_off
            selftest_ready selftest_invalid selftest_only_stimulus_ready selftest_baseline_ready
            selftest_spi_failure selftest_stimulus_write_failure selftest_stimulus_mismatch selftest_stop_stimulus)
        add_test(NAME imu_gyro_probe_${_scenario} COMMAND imu_gyro_probe_test ${_scenario})
    endforeach()
endif()

set(_pwm_hal_include "${K230_SDK_ROOT}/src/rtsmart/libs/rtsmart_hal/drivers/pwm")
set(_fpioa_hal_include "${K230_SDK_ROOT}/src/rtsmart/libs/rtsmart_hal/drivers/fpioa")
if(EXISTS "${_pwm_hal_include}/drv_pwm.h" AND EXISTS "${_fpioa_hal_include}/drv_fpioa.h")
    dart_add_modules(dart_pwm_k230_host "${PROJECT_SOURCE_DIR}/include/firmware/src/pwm/k230/output.cpp")
    target_include_directories(dart_pwm_k230_host PRIVATE "${_pwm_hal_include}" "${_fpioa_hal_include}")
    target_link_libraries(dart_pwm_k230_host PUBLIC dart_pwm)
    add_executable(pwm_output_test pwm_output_test.cpp)
    target_link_libraries(pwm_output_test PRIVATE dart_pwm_k230_host)
    target_include_directories(pwm_output_test PRIVATE "${_pwm_hal_include}" "${_fpioa_hal_include}")
    target_compile_options(pwm_output_test PRIVATE -Werror -UNDEBUG)
    add_test(NAME pwm_output COMMAND pwm_output_test)

    add_library(pwm_swing_under_test OBJECT "${PROJECT_SOURCE_DIR}/tools/pwm_swing/main.c")
    target_compile_definitions(pwm_swing_under_test PRIVATE main=pwm_swing_main)
    target_include_directories(pwm_swing_under_test PRIVATE "${_pwm_hal_include}" "${_fpioa_hal_include}")
    target_compile_options(pwm_swing_under_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
    add_executable(pwm_swing_test pwm_swing_test.c $<TARGET_OBJECTS:pwm_swing_under_test>)
    target_include_directories(pwm_swing_test PRIVATE "${_pwm_hal_include}" "${_fpioa_hal_include}")
    target_compile_options(pwm_swing_test PRIVATE -Wall -Wextra -Wpedantic -Werror -UNDEBUG)
    target_link_libraries(pwm_swing_test PRIVATE m)
    target_link_options(pwm_swing_test PRIVATE -Wl,--wrap=clock_gettime -Wl,--wrap=nanosleep)
    foreach(_scenario success pinmux_failure frequency_failure enable_failure pulse_failure readback_failure interrupt disable_failure)
        add_test(NAME pwm_swing_${_scenario} COMMAND pwm_swing_test ${_scenario})
    endforeach()
endif()

# Independent importers must resolve to the same compiled definitions at link time.
dart_add_modules(dart_odr_consumer odr_consumer.cpp)
target_link_libraries(dart_odr_consumer PUBLIC dart_detection dart_recording dart_imu dart_control dart_pwm)
target_link_libraries(camera_contract_test PRIVATE dart_odr_consumer)
target_link_libraries(servo_mapping_test PRIVATE dart_odr_consumer)

find_package(Python3 COMPONENTS Interpreter REQUIRED)
add_test(NAME canmv_build_config
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_canmv_build_config.py")
add_test(NAME rtos_scripts
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_rtos_scripts.py")
add_test(NAME cpp_structure_rules
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test_cpp_structure.py")
add_test(NAME cpp_structure
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/check_cpp_structure.py")
add_test(NAME launcher_restart
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/launcher_restart_test.sh" $<TARGET_FILE:launcher>)
add_test(NAME launcher_write_control
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/launcher_write_control_test.py" $<TARGET_FILE:launcher>)
