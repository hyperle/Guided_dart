# Control Action 说明

本文总结当前新增的两个控制 action 原型：

- `ImuVelocityRecoveryAction`
- `TimedTurnPulseAction`

二者当前都属于未接入主循环的计算型 action。它们不调用 `Servo_SetPulseUsBatch`，不直接写硬件 PWM；默认接入宏均为 `0`，后续需要接入时应先经过顶层调度和 `ControlMixer` 或明确的 PWM 输出策略。

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

默认接入开关：

```c
#define IMU_VELOCITY_RECOVERY_ACTION_ENABLE_IN_MAIN 0
```

### 目标

该 action 表达一种“先按像素误差方向给固定 PWM kick，再用 IMU X 轴角速度闭环恢复”的控制方式：

1. 收到 `delta_x` 后，在 `enter` 阶段锁存 `delta_x` 的正负号。
2. 用 `abs(delta_x) * delta_to_tick_scale` 计算 kick 持续 tick 数，并限制在 `min_tick_period` 到 `max_tick_period` 内。
3. kick 阶段输出 `sign(delta_x) * kick_pwm_us` 的水平 PWM contribution。
4. kick 结束后等待 tick=10 锁存速度输入就绪。
5. 使用有符号速度误差 `launch_velocity_x_dps - imu_velocity_x_dps` 运行 PID。
6. 速度误差连续落入 `success_band_dps` 达到 `success_hold_ticks` 后返回 `TASK_ACTION_SUCCESS`。

### 默认参数

```text
delta_to_tick_scale = 0.10
min_tick_period = 1
max_tick_period = 60
kick_pwm_us = 200
PID Kp/Ki/Kd = 2.0 / 0.0 / 0.0
PID output_limit = 300us
success_band = 5dps
success_hold_ticks = 3
velocity_sample_wait_ticks = 20
```

### 输入

核心输入字段：

- `target_delta.delta_x`：像素横向误差，只在 `enter` 阶段锁存方向和 kick 时间。
- `target_detected`：无目标时 action 失败。
- `now_tick`：当前调度 tick。
- `imu_velocity_x_dps`：当前有符号 X 轴角速度。
- `launch_velocity_x_dps`：tick=10 锁存的有符号 X 轴角速度。
- `launch_velocity_ready`：锁存速度是否有效。

### 输出

核心输出字段：

- `contribution.horizontal_pwm_us`：供后续 mixer 使用的水平 PWM contribution。
- `preview_pulse_us`：按当前水平矩阵生成的四路 PWM 预览，仅用于检查，不直接输出硬件。
- `stage`：当前阶段，便于 telemetry 或调试。
- `tick_period`：本次锁存的 kick tick 数。
- `target_velocity_x_dps`：已锁存的速度闭环目标。
- `velocity_error_x_dps`：当前有符号速度误差。

### 阶段

```text
IDLE
KICK
WAIT_VELOCITY_SAMPLE
PID_RECOVERY
DONE
```

### PWM 分配

kick 和 PID 输出都先写入水平 contribution，再由 action 内部生成四路 PWM 预览：

```text
servo0 -= horizontal_us
servo1 += horizontal_us
servo2 += horizontal_us
servo3 -= horizontal_us
```

这和当前 `ControlMixer` 的水平 PWM 矩阵一致。真正接入硬件时，优先把 `contribution` 交给 mixer，而不是使用 `preview_pulse_us` 直接写舵机。

## TimedTurnPulseAction

文件：

- `APP/Inc/timed_turn_pulse_action.hpp`
- `APP/Src/timed_turn_pulse_action.c`

默认接入开关：

```c
#define TIMED_TURN_PULSE_ACTION_ENABLE_IN_MAIN 0
```

### 目标

该 action 把固定时序脉冲抽象成“段表 + 舵机方向矩阵”的纯计算函数，避免为每一段重复写 PWM 分配代码。

核心纯计算入口：

```c
TaskActionResult_t TimedTurnPulseAction_Evaluate(
    const TimedTurnPulseActionInput_t *input,
    const TimedTurnPulseActionParams_t *params,
    TimedTurnPulseActionOutput_t *output);
```

`TaskAction_t` 包装层只复用这个函数，不增加硬件副作用。

### 默认参数

默认复用 `Controller_Vertical_Turn` 的四舵机方向矩阵：

```text
servo_direction = [-1, +1, +1, +1]
```

默认三段 period 由独立宏配置：

```c
#define TIMED_TURN_PULSE_ACTION_STAGE0_PERIOD_TICKS 50U
#define TIMED_TURN_PULSE_ACTION_STAGE1_PERIOD_TICKS 50U
#define TIMED_TURN_PULSE_ACTION_STAGE2_PERIOD_TICKS 50U
```

默认三段时序仍等价于：

```text
tick 10-60   : +200us
tick 60-110  : -200us
tick 110-160 : +200us
```

等价默认段表由宏累加生成：

```text
segments[0] = { start_tick=10, duration_ticks=STAGE0_PERIOD, pwm_us=+200 }
segments[1] = { start_tick=10 + STAGE0_PERIOD, duration_ticks=STAGE1_PERIOD, pwm_us=-200 }
segments[2] = { start_tick=10 + STAGE0_PERIOD + STAGE1_PERIOD, duration_ticks=STAGE2_PERIOD, pwm_us=+200 }
```

### 输入

核心输入字段：

- `now_tick`：当前 tick。
- `enabled`：为 `false` 时返回 `TASK_ACTION_FAILURE`。

### 输出

核心输出字段：

- `pulse_us`：按 base PWM、方向矩阵和当前段 PWM 计算出的四路 PWM 预览。
- `stage`：`WAIT / ACTIVE / DONE`。
- `active_segment_index`：当前命中的段号。
- `active_pwm_us`：当前段 PWM 值。
- `pwm_active`：当前 tick 是否处于某个有效脉冲段。

### 返回值

- 当前 tick 小于第一段开始 tick：返回 `TASK_ACTION_RUNNING`，输出 base PWM 预览。
- 当前 tick 落在某段内：返回 `TASK_ACTION_RUNNING`，输出该段 PWM 预览。
- 当前 tick 大于等于最后一个有效段结束 tick：返回 `TASK_ACTION_SUCCESS`。
- input/params/output 为空、未使能、段数为 0：返回 `TASK_ACTION_FAILURE`。

### PWM 分配

公共分配公式：

```text
pulse_us[i] = base_pulse_us[i] + servo_direction[i] * active_pwm_us
```

计算后会执行 `SERVO_PULSE_MIN_US / SERVO_PULSE_MAX_US` 物理限幅。当前 action 只输出 `pulse_us` 预览，不调用舵机驱动。

## 后续接入建议

如果接入 `ImuVelocityRecoveryAction`：

- 在 `GuidanceOrchestrator` 或当前 `main.c` 顶层确认目标有效和 IMU 更新完成后 tick。
- 优先把 `output.contribution` 接给 `ControlMixer_SetContribution` 或后续多 slot mixer。
- action 失败或退出时清空 contribution，避免 PWM 残留。

如果接入 `TimedTurnPulseAction`：

- 它当前输出的是四路 PWM 预览，不是 `GuidanceControlContribution_t`。
- 若要保持统一 mixer 架构，应给 `ControlMixer` 增加明确的 timed pulse/override slot，再由 mixer 做最终限幅和输出。
- 不建议在 action 内直接调用 `Servo_SetPulseUsBatch`。

## 维护点

- 改 kick 策略：优先调整 `ImuVelocityRecoveryActionParams_t`，不要改运行流程。
- 改固定时序：优先调整 `TimedTurnPulseActionParams_t.segments`，不要复制新的分段代码。
- 改舵机方向：调整 `servo_direction[4]` 即可复用同一分配函数。
- 接入主循环：先把对应 `*_ENABLE_IN_MAIN` 宏从 `0` 改为 `1`，再补齐顶层调度和 mixer 接线。
