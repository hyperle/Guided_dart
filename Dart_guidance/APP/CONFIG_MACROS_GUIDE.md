# Dart_Guidance 配置宏速查指南

本文档汇总项目中所有可调的功能开关宏、参数宏，按职责分类。修改宏后需重新编译烧录。

---

## 1. 控制模式开关

| 宏 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `GUIDANCE_DEFAULT_CONTROL_MODE` | `guidance_controller.h:24` | `GUIDANCE_CONTROL_MODE_HORIZONTAL_PWM_PID` | 上电默认控制模式。可选: `FIXED_PWM`=1 固定零点, `HORIZONTAL_PWM_PID`=2 水平闭环 |
| `GUIDANCE_HORIZONTAL_PID_INVERT_OUTPUT` | `guidance_controller.h:48` | `false` | PID 输出方向翻转。`true` 反转 delta_x→PWM 正负号 |
| `GREEN_LIGHT_TASK_SETPOINT_USE_IMAGE_CENTER` | `green_light_task.hpp:25` | `0U` | setpoint 使用策略：`0`=固定坐标, `1`=画面中心自动 |
| `GUIDANCE_PLANNER_LAUNCH_SAMPLE_WAIT_TICKS` | `guidance_planner.h` | `5U` | IMU 全阈值触发后，planner 输出 base PWM 并锁存三轴角速度前等待的 tick 数 |
| `GUIDANCE_PLANNER_FLIGHT_END_ACCEL_THRESHOLD_RATIO` | `guidance_planner.h` | `0.5f` | 飞行结束边沿阈值比例，乘以 IMU 发射阈值使用 |
| `GUIDANCE_TARGET_SMOOTHER_DEFAULT_ENABLE` | `target_smoother.h:16` | `1U` | 测量值 alpha-beta 平滑器开关。`0`=直通原始测量值 |

---

## 2. 水平 / 竖直 PID 参数

当前生效路径：`PixelDeltaPwmPidAction`，参数在 `guidance_controller.h` 定义，`main.c` 注入。
水平 PID 消费 `delta_x`，竖直 PID 消费 `delta_y`，两轴同时闭环。

| 宏 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `GUIDANCE_HORIZONTAL_PID_KP` | `guidance_controller.h:33` | `1.0f` | 水平比例增益。delta_x 像素误差 → PWM us 修正量 |
| `GUIDANCE_HORIZONTAL_PID_KI` | `guidance_controller.h:36` | `0.05f` | 水平积分增益。消除稳态静差 |
| `GUIDANCE_HORIZONTAL_PID_KD` | `guidance_controller.h:39` | `0.01f` | 水平微分增益。抑制突变，当前接近关闭 |
| `GUIDANCE_HORIZONTAL_PID_INTEGRAL_LIMIT` | `guidance_controller.h:42` | `1000.0f` | 水平积分项绝对值上限。防止丢靶后积分饱和冲击 |
| `GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US` | `guidance_controller.h:45` | `100.0f` | 水平 PID 输出幅值上限 (us)。限制零点附近的 PWM 加减幅度 |
| `GUIDANCE_HORIZONTAL_PID_INVERT_OUTPUT` | `guidance_controller.h:48` | `false` | 水平输出方向翻转。`true` 反转 delta_x→PWM 正负号 |
| `GUIDANCE_VERTICAL_PID_KP` | `guidance_controller.h:51` | `1.0f` | 竖直比例增益。delta_y 像素误差 → PWM us 修正量 |
| `GUIDANCE_VERTICAL_PID_KI` | `guidance_controller.h:54` | `0.05f` | 竖直积分增益。消除稳态静差 |
| `GUIDANCE_VERTICAL_PID_KD` | `guidance_controller.h:57` | `0.01f` | 竖直微分增益。抑制突变 |
| `GUIDANCE_VERTICAL_PID_INTEGRAL_LIMIT` | `guidance_controller.h:60` | `1000.0f` | 竖直积分项绝对值上限。防止丢靶后积分饱和冲击 |
| `GUIDANCE_VERTICAL_PID_OUTPUT_LIMIT_US` | `guidance_controller.h:63` | `100.0f` | 竖直 PID 输出幅值上限 (us)。限制零点附近的 PWM 加减幅度 |
| `GUIDANCE_VERTICAL_PID_INVERT_OUTPUT` | `guidance_controller.h:66` | `false` | 竖直输出方向翻转。`true` 反转 delta_y→PWM 正负号 |

> 竖直方向软关闭：设 `GUIDANCE_VERTICAL_PID_KP=0` 即可只保留水平闭环，不做竖直修正。

---

## 3. 测量值平滑器参数

平滑器在 `GreenLightTask` acquisition 阶段对上游圆心做 alpha-beta 滤波，输出供 `compute_delta` 使用。

| 宏 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `GUIDANCE_TARGET_SMOOTHER_DEFAULT_ALPHA_Q15` | `target_smoother.h:19` | `14746` | 位置滤波增益，Q15 格式。≈0.45，越小越平滑 |
| `GUIDANCE_TARGET_SMOOTHER_DEFAULT_BETA_Q15` | `target_smoother.h:22` | `3932` | 速度滤波增益，Q15 格式。≈0.12，越小响应越慢 |
| `GUIDANCE_TARGET_SMOOTHER_DEFAULT_MAX_VELOCITY_PX_PER_FRAME` | `target_smoother.h:25` | `64U` | 像素/帧速度上限。防止单帧坏值拉偏预测 |
| `GUIDANCE_TARGET_SMOOTHER_GAIN_Q15_ONE` | `target_smoother.h:13` | `32768U` | Q15=1.0 基准值。仅供内部运算，一般不改 |

---

## 4. IMU / 姿态 / 发射检测

| 宏 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `GUIDANCE_IMU_MAHONY_KP` | `imu.h:29` | `1.0f` | Mahony AHRS 比例增益。越大越信任加速度计纠偏 |
| `GUIDANCE_IMU_MAHONY_KI` | `imu.h:30` | `0.0f` | Mahony AHRS 积分增益。默认关闭 |
| `IMU_REFERENCE_WINDOW_SIZE` | `imu.h:17` | `10U` | 稳定姿态滚动窗口长度。发射脉冲时锁存该窗口均值作为参考 |
| `IMU_ACCEL_NORM_TARGET_G` | `imu.h:19` | `1.0f` | 静态加速度模目标 (g) |
| `IMU_ACCEL_FULL_SCALE_G` | `imu.h:21` | `24.0f` | BMI088 加速度计量程。须与 `ACC_RANGE_24` 一致 |
| `IMU_DEFAULT_ACCEL_TRUST_DELTA_G` | `imu.h:23` | `0.35f` | 加速度可信半宽 (g)。\|norm-1g\| 超过此值则切换为纯陀螺积分 |
| `IMU_DEFAULT_ACCEL_PULSE_THRESHOLD_G` | `imu.h:25` | `2.0f` | 发射/撞击脉冲检测阈值 (g)。触发参考姿态锁存 |
| `IMU_DEFAULT_ACCEL_AXIS_SATURATION_RATIO` | `imu.h:27` | `0.98f` | 单轴近饱和比例。超出量程×此值则标记加速度不可信 |
| `IMU_DEFAULT_RECOVERY_DELAY_MS` | `imu.h:28` | `150U` | 高动态后恢复延时 (ms)。加速度恢复可信后等待此时间再重新启用 accel+gyro |
| `IMU_DEFAULT_DART_LAUNCH_ACCEL_AXIS` | `imu.h:33` | `IMU_ACCEL_AXIS_X` | 发射检测轴。0=X, 1=Y, 2=Z |
| `IMU_DEFAULT_DART_LAUNCH_ACCEL_THRESHOLD_MPS2` | `imu.h:34` | `16.0f` | 发射检测轴绝对加速度阈值 (m/s²)。超过则翻转 `DartLaunched` |
| `IMU_DART_LAUNCH_COUNTER_LIMIT` | `imu.h:37` | `1000U` | 发射计数器上限。到达后归零 |
| `IMU_DART_LAUNCH_VELOCITY_SAMPLE_TICKS` | `imu.h:39` | `10U` | 发射后第 N tick 锁存三轴角速度供速度恢复使用 |

---

## 5. 图像 / Setpoint 尺寸

| 宏 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `GREEN_LIGHT_TASK_DEFAULT_IMAGE_WIDTH` | `green_light_task.hpp:20` | `320U` | 默认图像宽度。OpenMV 首帧到达前使用此值 |
| `GREEN_LIGHT_TASK_DEFAULT_IMAGE_HEIGHT` | `green_light_task.hpp:22` | `240U` | 默认图像高度 |
| `GREEN_LIGHT_TASK_SETPOINT_X` | `green_light_task.hpp:28` | `160U` | 固定 setpoint X 坐标 (仅 `SETPOINT_USE_IMAGE_CENTER=0` 生效) |
| `GREEN_LIGHT_TASK_SETPOINT_Y` | `green_light_task.hpp:30` | `120U` | 固定 setpoint Y 坐标 (仅 `SETPOINT_USE_IMAGE_CENTER=0` 生效) |
| `GUIDANCE_CONTROLLER_DEFAULT_IMAGE_WIDTH` | `guidance_controller.h:17` | `320U` | 旧控制器路径默认图像宽度 |
| `GUIDANCE_CONTROLLER_DEFAULT_IMAGE_HEIGHT` | `guidance_controller.h:18` | `240U` | 旧控制器路径默认图像高度 |

---

## 6. 舵机零点 / PWM 限幅

| 宏 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `GUIDANCE_SERVO_INITIAL_PWM_US_0` | `guidance_controller.h:27` | `2060U` | Servo 2 (按硬件标注) 零点脉宽 |
| `GUIDANCE_SERVO_INITIAL_PWM_US_1` | `guidance_controller.h:28` | `1970U` | Servo 4 零点脉宽 |
| `GUIDANCE_SERVO_INITIAL_PWM_US_2` | `guidance_controller.h:29` | `2025U` | Servo 3 零点脉宽 |
| `GUIDANCE_SERVO_INITIAL_PWM_US_3` | `guidance_controller.h:30` | `2000U` | Servo 1 零点脉宽 |
| `SERVO_PULSE_MIN_US` | `servo.h:31` | `0U` | 物理输出脉宽下界 (us) |
| `SERVO_PULSE_MAX_US` | `servo.h:34` | `3000U` | 物理输出脉宽上界 (us)。须 < `SERVO_PWM_PERIOD_US` |
| `SERVO_PULSE_CENTER_US` | `servo.h:41` | `2000U` | 上电/丢靶默认脉宽 (us) |
| `SERVO_PWM_FREQUENCY_HZ` | `servo.h:27` | `330U` | PWM 频率。须与 CubeMX 定时器周期配置一致 |

> 零点修改：改 `guidance_controller.h:27-30` 的四路 `GUIDANCE_SERVO_INITIAL_PWM_US_x`。
> 物理上下限修改：改 `servo.h:31,34` 的 `SERVO_PULSE_MIN_US` / `SERVO_PULSE_MAX_US`。
> MIXER 方向修改：改 `control_mixer.c:74-87` 的水平/竖直混控加减符号矩阵。

---

## 7. IMU 速度恢复 Action / 水平 yaw recovery

| 宏 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `IMU_VELOCITY_RECOVERY_DELTA_TO_TICK_SCALE` | `imu_velocity_recovery_action.hpp` | `0.10f` | `abs(delta_x)` 到 sweep tick 周期的缩放 |
| `IMU_VELOCITY_RECOVERY_MIN_TICK_PERIOD` | `imu_velocity_recovery_action.hpp` | `2U` | 非零 delta 时 sweep 最小 tick 周期 |
| `IMU_VELOCITY_RECOVERY_MAX_TICK_PERIOD` | `imu_velocity_recovery_action.hpp` | `60U` | sweep 最大 tick 周期 |
| `IMU_VELOCITY_RECOVERY_SWEEP_START_PWM_US` | `imu_velocity_recovery_action.hpp` | `120.0f` | 正向语义下 sweep 起始水平增量 |
| `IMU_VELOCITY_RECOVERY_SWEEP_END_PWM_US` | `imu_velocity_recovery_action.hpp` | `-120.0f` | 正向语义下 sweep 结束水平增量 |
| `IMU_VELOCITY_RECOVERY_PID_KP` | `imu_velocity_recovery_action.hpp` | `2.0f` | yaw 速度恢复 PID 比例增益 |
| `IMU_VELOCITY_RECOVERY_PID_KI` | `imu_velocity_recovery_action.hpp` | `0.0f` | yaw 速度恢复 PID 积分增益 |
| `IMU_VELOCITY_RECOVERY_PID_KD` | `imu_velocity_recovery_action.hpp` | `0.0f` | yaw 速度恢复 PID 微分增益 |
| `IMU_VELOCITY_RECOVERY_PID_INTEGRAL_LIMIT` | `imu_velocity_recovery_action.hpp` | `100.0f` | yaw 速度恢复 PID 积分限幅 |
| `IMU_VELOCITY_RECOVERY_PID_OUTPUT_LIMIT_US` | `imu_velocity_recovery_action.hpp` | `300.0f` | yaw 速度恢复 PID 输出限幅 (us) |
| `IMU_VELOCITY_RECOVERY_SUCCESS_BAND_DPS` | `imu_velocity_recovery_action.hpp` | `5.0f` | yaw 角速度误差进入此范围视为恢复成功 (deg/s) |
| `IMU_VELOCITY_RECOVERY_SUCCESS_HOLD_TICKS` | `imu_velocity_recovery_action.hpp` | `3U` | 连续满足 recovery band 的 tick 数后返回 SUCCESS |
| `IMU_VELOCITY_RECOVERY_VELOCITY_SAMPLE_WAIT_TICKS` | `imu_velocity_recovery_action.hpp` | `20U` | planner 未提供锁存速度时的保护等待上限 |

---

## 8. 定时脉冲 Turn Action / Planner 触发

| 宏 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `GUIDANCE_PLANNER_LAUNCH_SAMPLE_WAIT_TICKS` | `guidance_planner.h` | `5U` | IMU 全阈值触发后，planner 输出 base PWM 并等待锁存三轴角速度的 tick 数 |
| `GUIDANCE_PLANNER_FLIGHT_END_ACCEL_THRESHOLD_RATIO` | `guidance_planner.h` | `0.5f` | 飞行结束边沿使用的阈值比例，实际阈值为配置发射阈值乘该比例 |
| `TIMED_TURN_PULSE_ACTION_MAX_SEGMENTS` | `timed_turn_pulse_action.hpp` | `3U` | 竖直 turn 脉冲段表最大段数 |
| `TIMED_TURN_PULSE_ACTION_DEFAULT_START_TICK` | `timed_turn_pulse_action.hpp` | `0U` | 竖直 turn action 内部第一段起始 tick；初始等待由 planner 执行 |
| `TIMED_TURN_PULSE_ACTION_STAGE0_PERIOD_TICKS` | `timed_turn_pulse_action.hpp` | `70U` | 第 0 段下降斜坡 tick 间隔数 |
| `TIMED_TURN_PULSE_ACTION_STAGE1_PERIOD_TICKS` | `timed_turn_pulse_action.hpp` | `15U` | 第 1 段 0 输出等待 tick 数 |
| `TIMED_TURN_PULSE_ACTION_STAGE2_PERIOD_TICKS` | `timed_turn_pulse_action.hpp` | `30U` | 第 2 段上升斜坡 tick 间隔数 |
| `TIMED_TURN_PULSE_ACTION_STAGE0_START_PWM_US` | `timed_turn_pulse_action.hpp` | `120.0f` | 第 0 段起始竖直增量 |
| `TIMED_TURN_PULSE_ACTION_STAGE0_STEP_PWM_US` | `timed_turn_pulse_action.hpp` | `-4.0f` | 第 0 段每 tick 增量变化 |
| `TIMED_TURN_PULSE_ACTION_STAGE2_STEP_PWM_US` | `timed_turn_pulse_action.hpp` | `4.0f` | 第 2 段每 tick 增量变化 |

---

## 9. 时序 / 超时 / 波特率

| 宏 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `GUIDANCE_CONTROLLER_LOOP_PERIOD_MS` | `guidance_controller.h:14` | `10U` | 控制计算周期 (ms)。PID dt 据此计算 |
| `GREEN_LIGHT_TASK_LOOP_PERIOD_MS` | `green_light_task.hpp:33` | `10U` | 主循环调度周期 (ms)。IMU/控制/遥测均以此为基准 |
| `GREEN_LIGHT_TASK_IDLE_POLL_DELAY_MS` | `green_light_task.hpp:35` | `1U` | 无控制 tick 时的空闲延时 (ms) |
| `GREEN_LIGHT_TASK_STARTUP_IMAGE_SIZE_TIMEOUT_MS` | `green_light_task.hpp:37` | `8000U` | 等待 OpenMV 首帧图像尺寸的最大时间 (ms) |
| `ESP32_LINK_TX_TIMEOUT_MS` | `esp32_link.h:22` | `5U` | UART TX 超时 (ms)。过短可能导致遥测丢帧 |
| `ESP32_LINK_IMU_MOTION_PUBLISH_DIVIDER` | `esp32_link.h:24` | `4U` | 0x05 IMU_MOTION 帧发布分频。每 N 个主循环 tick 发布一次 (=40ms) |

---

## 10. Task 框架

| 宏 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `TASK_INTERRUPT_STACK_CAPACITY` | `task.hpp:20` | `4U` | 串行 action 中断栈最大深度 |

---

## 11. 数据类型 / 哨兵值

| 宏 | 文件 | 默认值 | 说明 |
|---|---|---|---|
| `GUIDANCE_NO_TARGET_COORDINATE` | `guidance_types.h:20` | `0xFFFFU` | 无目标标识。measurement 坐标为此值时表示丢靶 |
| `GUIDANCE_SERVO_COUNT` | `guidance_types.h:33` | `4U` | 舵机通道数 |
| `GUIDANCE_AIM_AUTO_SETPOINT_COORDINATE` | `guidance_controller.h:16` | `0xFFFFU` | 自动居中标记。setpoint 为此值时初始化为图像中心 |

---

## 12. BMI088 硬件配置 (Drv/Inc/bmi088.h)

| 宏 | 默认值 | 说明 |
|---|---|---|
| `ACC_RANGE_24` (0x03) | **当前使用** | 加速度计量程 ±24g |
| `ACC_BW_NORMAL` (0x02) | **当前使用** | 加速度计带宽：Normal 模式 |
| `ACC_ODR_1600HZ` (0x0C) | **当前使用** | 加速度计输出速率 1600Hz |
| `GYRO_RANGE_2000` (0x00) | **当前使用** | 陀螺仪量程 ±2000 deg/s |
| `GYRO_BW_532` (0x00) | **当前使用** | 陀螺仪带宽 532Hz / ODR 2000Hz |

> 加速度计量程切换后须同步修改 `imu.h` 中的 `IMU_ACCEL_FULL_SCALE_G`。

---

## 13. 协议帧常量 (Drv/Inc/guidance_protocol_generated.h)

| 宏 | 值 | 说明 |
|---|---|---|
| `GUIDANCE_PROTOCOL_FRAME_HEADER_0` | `0xA5U` | 帧头同步字节 0 |
| `GUIDANCE_PROTOCOL_FRAME_HEADER_1` | `0x5AU` | 帧头同步字节 1 |
| `ESP32_LINK_MESSAGE_TYPE_GUIDANCE_TELEMETRY` | `0x01U` | 遥测帧 |
| `ESP32_LINK_MESSAGE_TYPE_IMU_ACCEL` | `0x04U` | IMU 加速度帧 |
| `ESP32_LINK_MESSAGE_TYPE_IMU_MOTION` | `0x05U` | IMU 运动帧 (陀螺+加速度，兼容 24-byte bridge) |
| `ESP32_LINK_MESSAGE_TYPE_DART_LAUNCH_SAMPLE` | `0x06U` | 发射 tick 与 tick=10 锁存速度帧 |

---

## 快速索引

| 想做什么 | 改哪个宏 | 位置 |
|---|---|---|
| 启用/禁用平滑器 | `GUIDANCE_TARGET_SMOOTHER_DEFAULT_ENABLE` | `target_smoother.h:16` |
| 调节平滑力度 | `GUIDANCE_TARGET_SMOOTHER_DEFAULT_ALPHA_Q15` / `BETA_Q15` | `target_smoother.h:19,22` |
| 调水平 PID 强弱 | `GUIDANCE_HORIZONTAL_PID_KP` / `KI` / `KD` | `guidance_controller.h:33-39` |
| 调竖直 PID 强弱 | `GUIDANCE_VERTICAL_PID_KP` / `KI` / `KD` | `guidance_controller.h:51-57` |
| 限制水平 PWM 摆动幅度 | `GUIDANCE_HORIZONTAL_PID_OUTPUT_LIMIT_US` | `guidance_controller.h:45` |
| 限制竖直 PWM 摆动幅度 | `GUIDANCE_VERTICAL_PID_OUTPUT_LIMIT_US` | `guidance_controller.h:63` |
| 翻转水平 PID 输出方向 | `GUIDANCE_HORIZONTAL_PID_INVERT_OUTPUT` | `guidance_controller.h:48` |
| 翻转竖直 PID 输出方向 | `GUIDANCE_VERTICAL_PID_INVERT_OUTPUT` | `guidance_controller.h:66` |
| 软关闭竖直控制 | `GUIDANCE_VERTICAL_PID_KP = 0` | `guidance_controller.h:51` |
| 改 setpoint 坐标 | `GREEN_LIGHT_TASK_SETPOINT_X` / `Y` | `green_light_task.hpp:28,30` |
| setpoint 改为画面中心 | `GREEN_LIGHT_TASK_SETPOINT_USE_IMAGE_CENTER` | `green_light_task.hpp:25` |
| 改舵机零点 | `GUIDANCE_SERVO_INITIAL_PWM_US_0~3` | `guidance_controller.h:27-30` |
| 改舵机物理限幅 | `SERVO_PULSE_MIN_US` / `MAX_US` | `servo.h:31,34` |
| 改 MIXER 舵面方向 | `control_mixer.c:74-77` 加减符号 | `control_mixer.c` |
| 改发射检测轴 | `IMU_DEFAULT_DART_LAUNCH_ACCEL_AXIS` | `imu.h:33` |
| 改发射加速度阈值 | `IMU_DEFAULT_DART_LAUNCH_ACCEL_THRESHOLD_MPS2` | `imu.h:34` |
| 改发射脉冲检测灵敏度 | `IMU_DEFAULT_ACCEL_PULSE_THRESHOLD_G` | `imu.h:25` |
| 改加速度可信窗口 | `IMU_DEFAULT_ACCEL_TRUST_DELTA_G` | `imu.h:23` |
| 改主循环频率 | `GREEN_LIGHT_TASK_LOOP_PERIOD_MS` | `green_light_task.hpp:33` |
| 改遥测发布频率 | `ESP32_LINK_IMU_MOTION_PUBLISH_DIVIDER` | `esp32_link.h:24` |
| 改 planner 启动后的速度锁存等待 | `GUIDANCE_PLANNER_LAUNCH_SAMPLE_WAIT_TICKS` | `guidance_planner.h` |
| 改飞行结束加速度比例 | `GUIDANCE_PLANNER_FLIGHT_END_ACCEL_THRESHOLD_RATIO` | `guidance_planner.h` |
