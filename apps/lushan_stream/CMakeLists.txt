set(APP_NAME lushan_stream)

add_executable(${APP_NAME} main.cpp)
target_compile_options(${APP_NAME} PRIVATE -Wall -Wextra -ffunction-sections -fdata-sections)
target_link_libraries(${APP_NAME} PRIVATE k230_rtos_app k230_peripherals netmgmt pthread)
set_target_properties(${APP_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${APP_NAME}")
