# L4 algorithms include only frame contracts; device selection stays in board/.
file(GLOB _detection_sources CONFIGURE_DEPENDS "detector/*.cpp")
dart_add_cpp_library(dart_detection ${_detection_sources})
target_link_libraries(dart_detection PUBLIC dart_camera_types)

dart_add_cpp_library(dart_control control/servo.cpp)
target_link_libraries(dart_control PUBLIC dart_detection dart_imu)

file(GLOB_RECURSE _application_sources CONFIGURE_DEPENDS
    "startup/*.cpp" "vision/*.cpp" "recording/*.cpp" "diagnostics/*.cpp")
dart_add_cpp_library(dart_application ${_application_sources})
target_link_libraries(dart_application PUBLIC dart_detection dart_control dart_recording
    dart_imu_logging dart_pwm dart_platform dart_device_services dart_streaming)

if(DART_CAMERA_K230)
    add_executable(dart_app dart_entry.cpp)
    target_compile_options(dart_app PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(dart_app PRIVATE dart_application dart_board Threads::Threads)
    set_target_properties(dart_app PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dart_app")
endif()

# C ABI for CanMV: link the same detector implementation into the interpreter.
add_library(dart_canmv_detector STATIC "${PROJECT_SOURCE_DIR}/tools/canmv_tuning/detector_bridge.cpp")
target_link_libraries(dart_canmv_detector PUBLIC dart_detection)
target_compile_options(dart_canmv_detector PRIVATE -Wall -Wextra -Wpedantic)
