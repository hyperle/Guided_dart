set(APP_NAME green_led_rtos)
add_executable(${APP_NAME} main.c recorder.c)
target_compile_options(${APP_NAME} PRIVATE -Wall -Wextra -ffunction-sections -fdata-sections)
# RT-Smart rejects oversized user ELFs on some CanMV images; no debug/symbol data is needed on-board.
target_link_options(${APP_NAME} PRIVATE -s)
target_link_libraries(${APP_NAME} PRIVATE k230_rtos_app k230_peripherals)
target_link_directories(${APP_NAME} PRIVATE
    "${K230_SDK_ROOT}/src/rtsmart/mpp/userapps/lib")
target_link_libraries(${APP_NAME} PRIVATE
    -Wl,--start-group vicap vb sys sensor isp venc
    -Wl,--end-group pthread)
set_target_properties(${APP_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${APP_NAME}")
