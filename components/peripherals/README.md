# Peripherals API

This directory wraps the SDK RT-Smart HAL for application code.

`pwm_motor` exposes percentage output with polarity handling. `servo` converts an angle to a pulse width. `mpu6050` is an I2C MPU6050 driver returning converted units (`g`, degrees/second, Celsius).

Example configuration:

```c
pwm_motor_config_t motor = { .channel = 0, .pin = 42, .fpioa_func = PWM0,
    .frequency_hz = 20000, .active_high = 1 };
pwm_motor_init(&motor);
pwm_motor_set_percent(35);

mpu6050_config_t imu = { .bus = 2, .address = 0x68, .bus_hz = 400000,
    .timeout_ms = 1000, .scl_pin = 11, .sda_pin = 12,
    .accel_fs_g = 2.0f, .gyro_fs_dps = 250.0f };
mpu6050_init(&imu);
mpu6050_sample_t sample;
mpu6050_read(&sample);
```

The matching SDK HAL libraries (`libpwm.a`, `libi2c.a`, `libgpio.a`, `libfpioa.a`) are discovered automatically from `K230_SDK_ROOT/output/*`.
