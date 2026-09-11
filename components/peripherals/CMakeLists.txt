add_library(k230_peripherals STATIC pwm_motor.c servo.c mpu6050.c)
target_include_directories(k230_peripherals PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(k230_peripherals PUBLIC k230_rtos_app)
