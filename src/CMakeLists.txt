# Detector is the only execution component; calculations live in its headers.
dart_add_cpp_library(dart_detection detector/detector.cpp)
target_link_libraries(dart_detection PUBLIC dart_camera_acl Threads::Threads)

dart_add_cpp_library(dart_recording recorder/recorder.cpp recorder/buffered_storage.cpp)
add_library(dart_recorder ALIAS dart_recording)
target_link_libraries(dart_recording PUBLIC dart_recording_formats dart_camera_acl dart_platform Threads::Threads)

dart_add_cpp_library(dart_control control/servo.cpp)
target_link_libraries(dart_control PUBLIC dart_detection dart_imu_logging dart_pwm Threads::Threads)

# C ABI for CanMV: link the same detector implementation into the interpreter.
if(DART_BUILD_TOOLS)
    add_library(dart_canmv_detector STATIC "${PROJECT_SOURCE_DIR}/tools/canmv_tuning/detector_bridge.cpp")
    target_link_libraries(dart_canmv_detector PUBLIC dart_detection)
    target_compile_options(dart_canmv_detector PRIVATE -Wall -Wextra -Wpedantic)
endif()
