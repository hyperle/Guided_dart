# Control Action 说明

本文总结当前两个控制 action 以及 `GuidancePlanner` 的接入关系：

- `TimedTurnPulseAction`：第一段竖直动作，由 planner 以四路 PWM override 方式输出。
- `ImuVelocityRecoveryAction`：第二段水平 delta_x sweep + yaw 速度恢复动作，由 planner 接入后输出 contribution 给 mixer。
- 默认 PID 混控：planner 无覆盖任务时放行 `PixelDeltaPwmPidAction`。

两个 action 本身仍不直接调用 `Servo_SetPulseUsBatch`；硬件输出集中在 `main.c` / `ControlMixer` 路径。

## 共同边界

两个 action 都遵循 `TaskAction_t` 生命周期接口：

- `enter`：锁存本次 action 需要的状态或直接转入 `running`。
- `running`：根据 profile 的 input/params 计算 output。
- `exit`：清空本 action 的输出或回到 base PWM 预览。

profile 仍保持三段结构：

- `input`：当前 tick、目标误差、IMU 速度等运行时输入。
- `params`：base PWM、PID 参数、时序段表、输出限幅等可配置参数。
- `output`：action 的阶段、PWM contribution 或四路 PWM 预览。

## ImuVelocityRecoveryAction

文件：

- `APP/Inc/imu_velocity_recovery_action.hpp`
- `APP/Src/imu_velocity_recovery_action.c`

### 目标

该 action 表达第二段“水平 delta_x 脉冲 + yaw 角速度恢复”的控制方式。它不直接写舵机，只输出 `GuidanceControlContribution_t` 给 `ControlMixer`。

当前接入方式由 `GuidancePlanner` 负责：竖直 `TimedTurnPulseAction` 完成后，planner 用控制侧当前目标 delta 启动该 action；若本周期没有新目标，则用 `GreenLightTask.last_valid_measurement` 按当前 setpoint 回推 delta。连续识别时该 delta 等价于实时/平滑后的目标坐标，短时丢帧时仍能使用上一次有效目标。

执行流程：

1. `enter` 阶段锁存 `delta_x` 符号和 `abs(delta_x) * delta_to_tick_scale` 得到的 sweep tick 数。
2. sweep 阶段按 delta 符号输出水平 PWM contribution：正向语义为 `+120us -> -120us`，反向语义为 `-120us -> +120us`。
3. sweep 的周期数由 delta 映射得到，并限制在 `min_tick_period / max_tick_period` 内；每 tick 增量变化量由起止 PWM 和周期数计算。
4. sweep 完成后，以 planner 在全阈值触发后等待 5 个 10ms 周期锁存的 Z 轴角速度作为 yaw 目标速度。
5. 使用 `launch_yaw_velocity_dps - imu_yaw_velocity_dps` 做 PID，输出水平 PWM contribution。
6. yaw 速度误差连续进入 `success_band_dps` 达到 `success_hold_ticks` 后，输出增量清零并返回 `TASK_ACTION_SUCCESS`。

### 默认参数

```text
delta_to_tick_scale = 0.10
min_tick_period = 2
max_tick_period = 60
sweep_start_pwm_us = +120
sweep_end_pwm_us = -120
PID Kp/Ki/Kd = 2.0 / 0.0 / 0.0
PID output_limit = 300us
success_band = 5dps
success_hold_ticks = 3
velocity_sample_wait_ticks = 20
```

### 输入

核心输入字段：

- `target_delta.delta_x`：启动 action 时锁存的横向像素误差。
- `target_detected`：启动时必须为 true；启动后 sweep 和 yaw PID 不再依赖持续识别。
- `now_tick`：当前 action 调度 tick。
- `imu_yaw_velocity_dps`：当前 Z 轴角速度，作为 yaw 速度反馈。
- `launch_yaw_velocity_dps`：planner 在启动后等待 5 tick 锁存的 Z 轴角速度。
- `launch_velocity_ready`：planner 锁存速度是否有效。

### 输出

核心输出字段：

- `contribution.horizontal_pwm_us`：供 `ControlMixer` 使用的水平 PWM contribution。
- `preview_pulse_us`：按当前水平 mixer 矩阵生成的四路 PWM 预览，仅用于检查。
- `stage`：`IDLE / SWEEP / WAIT_VELOCITY_SAMPLE / PID_RECOVERY / DONE`。
- `tick_period`：本次锁存的 sweep tick 数。
- `active_pwm_us`：当前 sweep 或 PID 输出的水平 PWM 增量。
- `target_yaw_velocity_dps`：已锁存的 yaw 速度目标。
- `velocity_error_yaw_dps`：当前 yaw 速度误差。

### PWM 分配

该 action 只写水平 contribution，最终舵机分配由 `ControlMixer_Solve` 统一处理。action 内部的 `preview_pulse_us` 也按同一水平矩阵计算，便于调试。

## TimedTurnPulseAction

文件：

- `APP/Inc/timed_turn_pulse_action.hpp`
- `APP/Src/timed_turn_pulse_action.c`

### 目标

该 action 把固定时序竖直 turn 抽象成“段表 + 舵机方向矩阵”的纯计算函数。它不读取 IMU、不判断阈值、不直接调用舵机驱动；planner 传入从竖直动作开始计数的 `now_tick`，action 输出四路 PWM 预览。

初始 5 tick 等待已从该 action 拆到 `GuidancePlanner`。因此该 action 的第一段从 `now_tick = 0` 开始。

### 默认参数

默认复用 `Controller_Vertical_Turn` 的四舵机方向矩阵：

```text
servo_direction = [-1, +1, +1, +1]
```

默认时序：

```text
tick 0-70    : +120us 起，每 tick -4us，到 -160us
tick 71-85   : 0us
tick 86-116  : 0us 起，每 tick +4us，到 +120us
```

段表由宏生成：

```text
segments[0] = { start_tick=0,  duration_ticks=STAGE0_PERIOD + 1, pwm_us=+120, step=-4 }
segments[1] = { start_tick=71, duration_ticks=STAGE1_PERIOD,     pwm_us=0,    step=0 }
segments[2] = { start_tick=86, duration_ticks=STAGE2_PERIOD + 1, pwm_us=0,    step=+4 }
```

第三段结束后的下一 tick 返回 `TASK_ACTION_SUCCESS`。

## GuidancePlanner 当前接入

文件：

- `APP/Inc/guidance_planner.h`
- `APP/Src/guidance_planner.c`

`main.c` 每个 10ms 控制 tick 仍先更新 IMU、消费 UART measurement、tick `GreenLightTask` 生成当前 delta。planner 只负责控制输出调度，不干涉相机采样。

当前调度链：

1. 无 IMU 全阈值上升沿时，默认放行 `PixelDeltaPwmPidAction`，保持原 PID 混控路径。
2. 配置轴加速度绝对值大于 `dart_launch_accel_threshold_mps2` 的上升沿触发一次飞行任务。
3. planner 先输出 base PWM 等待 `GUIDANCE_PLANNER_LAUNCH_SAMPLE_WAIT_TICKS` 个控制 tick，并在等待结束时锁存当前三轴角速度。
4. 执行 `TimedTurnPulseAction`，期间禁用 PID 输出，由 planner 将 action 的四路 PWM override 写入 mixer 输出。
5. 执行 `ImuVelocityRecoveryAction`，其 delta 来自当前目标状态，若当前帧无目标则使用 `last_valid_measurement` 回推 delta；输出 contribution 交给 `ControlMixer`。
6. yaw recovery 成功后回到默认 PID 混控，继续使用每周期实时/平滑后的 delta。
7. 在上述任务链进入默认 PID 后，下一次配置轴加速度大于 `0.5 * dart_launch_accel_threshold_mps2` 的上升沿视为飞行结束，本周期强制输出 base PWM，然后允许下一次全阈值上升沿重新排任务。

因此触发节奏是“全阈值上升沿排一次任务链，半阈值上升沿结束本次飞行并清零增量，再等待下一次全阈值上升沿”。

## 后续接入建议

- 后续串行/并行/中断任务可以在 `GuidancePlanner` 当前 phase 机上继续收敛，不必先改相机消费链路。
- `TimedTurnPulseAction` 当前是四路 PWM override；`ImuVelocityRecoveryAction` 和默认 PID 都走 contribution/mixer。若之后要统一所有动作输出，可以给 `ControlMixer` 增加明确的 override slot。
- action 失败或退出时都应清空 contribution，避免 PWM 残留。

## 维护点

- 改 sweep 策略：优先调整 `ImuVelocityRecoveryActionParams_t`，不要改运行流程。
- 改固定时序：优先调整 `TimedTurnPulseActionParams_t.segments`，不要复制新的分段代码。
- 改舵机方向：调整 `servo_direction[4]` 即可复用同一分配函数。
- 改接入顺序：调整 `GuidancePlannerPhase_t` 推进和 `GuidancePlanner_Tick` 中的 phase 转移；硬件输出仍保持在 `main.c` / `ControlMixer` 路径。
