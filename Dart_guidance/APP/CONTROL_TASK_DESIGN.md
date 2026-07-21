# 控制任务职责规划

本文记录 `Dart_guidance/APP` 后续控制架构的职责边界。目标是在单片机资源约束下，把目标几何、IMU 状态、控制算法、PWM 合成和顶层调度拆开为后续多种控制方式的串行、并行和输出叠加留出清晰接口。

## 设计目标

- `setpoint / measurement / delta` 只在一个位置归一化和计算，避免主循环与控制器重复接线。
- 三种控制方式都具备 task/action 生命周期，可按任务计划启停、串行、并行、阻断。
- 多个控制方式可以输出独立 PWM contribution，最终由统一 mixer 做叠加、限幅和舵机分配。
- IMU 姿态估计持续更新，但使用 IMU 的控制 action 可以按生命周期启停。
- `main.c` 只做硬件初始化、主循环节拍和顶层 tick 调用，不继续承载业务接线细节。
- MCU 侧优先静态分配、固定容量数组、无堆分配，减少控制帧开销和不可控抖动。

## 分层职责

### 1. 数据采集与状态服务

`OpenMV/UART` 只负责上游测量输入：圆心坐标、面积、图像尺寸。它不负责 setpoint，也不负责控制误差。

`ImuService` 负责持续更新 IMU 姿态、角速度、加速度、发射参考姿态和相对姿态误差。IMU 状态估计不建议做成可停 task，因为滤波、积分和发射脉冲锁存需要连续运行；但基于 IMU 的控制逻辑应做成可启停的 control action。

### 2. 全局 Profile

后续建议收敛为一个 `GuidanceProfile`，分为：

- `input`：OpenMV measurement、IMU state、外部参数命令。
- `params`：setpoint、image_size、控制模式、PID 参数、平滑器参数、叠加权重。
- `state/output`：target state、control contributions、final PWM、telemetry snapshot。

`setpoint` 应归入 `params`，由远程调参或启动默认值写入。任何模块需要 setpoint 时读取同一份 profile，不在 `main.c` 和控制器之间重复镜像。

### 3. TargetTrackingTask

由当前 `GreenLightTask` 演进而来。它只负责目标几何：

- 消费 OpenMV `measurement` 与 profile 中的 `setpoint / image_size`。
- 归一化 setpoint，例如自动居中和边界限幅。
- 输出 `target_detected / measurement / normalized_setpoint / delta`。
- 不做 PID，不写 PWM，不融合 IMU。

因此，`delta_x / delta_y` 的唯一计算位置应在 `TargetTrackingTask`。控制算法只消费 delta，不再用 measurement 与 setpoint 重算误差。

### 4. ControlAction

每一种控制方式都实现为一个生命周期 action，包含 `enter / running / exit`：

- `PixelDeltaPwmPidAction`：消费 `TargetState.delta_x`，输出水平 PWM contribution。
- `ImuSetpointSmoothAction`：消费 `IMU state + setpoint + smoother + target state`，输出一个 PWM contribution 或控制误差修正量。
- `ImuStabilizeAction`：消费 IMU 姿态/角速度误差，输出自稳 PWM contribution。

每个 control action 的规则：

- `enter` 初始化本 action 的 PID、平滑器、锁存参考值或输出槽。
- `running` 每个控制 tick 只写自己的 contribution slot。
- `exit` 清空自己的 contribution slot，避免 action 停止后 PWM 残留。
- 不直接调用 `Servo_SetPulseUsBatch`，不直接写最终 PWM。
- 不把丢靶回中封装在 action 内；目标丢失、固定角度和模式切换属于顶层控制策略。

### 5. ControlMixer

`ControlMixer` 是唯一负责最终 PWM 合成的模块：

- 从固定初始 PWM 生成 base PWM。
- 收集多个 control action 的 contribution。
- 按配置做串行选择、并行叠加、权重缩放或优先级覆盖。
- 对每路舵机做物理限幅。
- 最终调用 servo 驱动输出。

这层替代当前 `guidance_controller` 中“控制算法直接写舵机”的职责。控制 action 输出的是意图，mixer 才负责把意图映射成 Servo 0/1/2/3 的最终 PWM。

### 6. GuidanceOrchestrator

`GuidanceOrchestrator` 是顶层任务统筹器，负责每个控制 tick 的顺序：

1. 更新 IMU service。
2. 接收 OpenMV measurement。
3. tick `TargetTrackingTask`，得到 target state。
4. 在顶层判断固定模式、目标丢失和无新目标结果超时；需要回中时清空 control contribution。
5. 按当前 control plan tick 一个或多个 control action。
6. 调用 `ControlMixer` 合成并输出 PWM。
7. 组装 telemetry snapshot。

Orchestrator 负责 action 排序、串行推进、并行叠加、阻断插入和失败处理。这样 `main.c` 不再直接维护 task 细节，也不再做 setpoint 镜像或 controller 内部字段同步。

## 控制计划模型

后续控制计划可以表达为固定容量结构，避免动态分配：

- `sequence`：串行执行多个 action，前一个成功后进入下一个。
- `parallel_overlay`：多个 action 同 tick 运行，各自写 contribution，由 mixer 叠加。
- `interrupt`：临时 action 插入当前 action 前方，完成后恢复原 action。
- `complete_policy`：并行 action 可配置为任一成功即完成，或全部成功才完成。

如果某个 action 是长期闭环控制，它仍然可以保持 `TASK_ACTION_RUNNING`，直到 orchestrator 根据外部事件、模式切换或任务阶段显式退出它。

### Hook 事件

`hook` 是 task 向后续 `GuidanceOrchestrator / planner` 抛出的轻量一次性事件，用于表达“本 task 尚未完成，但可以触发某个特定任务或计划分支”。它不是 action 的完成结果，也不和 `waiting` 强绑定；task 可以在 `running` 时抛 hook 后继续运行，也可以抛 hook 后进入等待，后续 `waiting` 机制单独管理。

当前模板在 `TaskSequence_t` 内置一个 `TaskHook_t` 单槽：

- `TaskSequence_EmitHook(sequence, hook_id)` 抛出 hook；`TASK_HOOK_ID_NONE` 无效。
- planner 可通过 `TaskSequence_SetHookNotify(sequence, callback, context)` 注册即时回调；回调返回 `true` 表示已处理，模板会自动清 pending。
- 未注册回调或回调未处理时，hook 保持 pending，可由 `TaskSequence_HasHook` / `TaskSequence_ConsumeHook` 显式读取。
- 单槽 pending 未消费前再次 `EmitHook` 会失败，避免覆盖旧 hook。
- `TaskSequence_Tick` 与 `TaskAction_Tick` 的推进语义不因 hook 改变，hook 只作为调度侧事件存在。

后续 planner 应维护 `hook_id -> task/plan` 的静态映射。这样常态 tick 不需要扫描所有 task 的 hook 状态；只有 task 主动 emit 时才触发回调或留下 pending。

### Waiting 挂起与恢复

`waiting` 表示 task 暂时不能继续推进，但还没有成功或失败。它用于等待外部数据、等待某个 task 完成、等待时间窗口或等待 planner 决策。`waiting` 与 `hook` 是正交概念：task 可以只等待不抛 hook，也可以抛 hook 后继续 `running`，也可以抛 hook 后进入 waiting。

`waiting` 不应做成每 tick 扫描所有 task 的轮询状态。推荐由 planner 维护 ready/wait 两类静态表：

- ready 表只保存本 tick 可能推进的 task，控制主循环只 tick ready task。
- waiting task 从 ready 表移除，挂到对应等待源，例如 `data_ready`、`task_done`、`timeout`。
- 数据到达、被等待 task 完成或超时事件发生时，由事件源通知 planner，再把对应 task 放回 ready 表。
- 等待期间不调用 `TaskSequence_Tick`，避免 action 在条件未满足时反复空转。

恢复时只解除挂起，不重置任务进度：

- 保留 `TaskSequence.current_index`。
- 保留当前 `TaskAction.phase` 与 action 自身上下文。
- 清除 waiting 标志和等待目标后重新进入 ready 表。
- 下一个控制 tick 继续从原 action/sequence 位置推进。

只有等待超时、依赖任务失败或 planner 显式放弃当前任务时，才调用 reset/exit/fallback。这样 waiting 的常态成本接近事件驱动，而不是按 task 数量线性增长。

## 文件放置建议

新增 task/action 时采用一组头文件和实现文件：

```text
APP/Inc/<name>_task.hpp
APP/Src/<name>_task.c
```

跨多个控制 action 共享的状态类型放到明确职责的公共头文件，例如：

```text
APP/Inc/guidance_profile.hpp
APP/Inc/control_mixer.h
APP/Inc/guidance_orchestrator.h
```

建议的后续文件结构：

```text
APP/Inc/target_tracking_task.hpp
APP/Src/target_tracking_task.c

APP/Inc/pixel_delta_pid_task.hpp
APP/Src/pixel_delta_pid_task.c

APP/Inc/imu_setpoint_smooth_task.hpp
APP/Src/imu_setpoint_smooth_task.c

APP/Inc/imu_stabilize_task.hpp
APP/Src/imu_stabilize_task.c

APP/Inc/control_mixer.h
APP/Src/control_mixer.c

APP/Inc/guidance_orchestrator.h
APP/Src/guidance_orchestrator.c
```

辅助逻辑优先贴近调用方：只服务一个 task 的逻辑留在该 task 模块；多个 task 共享时，先判断它属于目标几何、IMU、PWM 合成还是顶层调度，再放入对应职责模块。不要把通用函数堆到无语义的工具文件里。

## 迁移顺序

1. 将 `GreenLightTask` 重命名或收敛为 `TargetTrackingTask`，保留唯一的 setpoint 归一化和 delta 计算。
2. 从 `guidance_controller` 删除 measurement 获取、setpoint 归一化和 delta 重算。
3. 定义 `GuidanceControlContribution_t`，让控制算法写 contribution slot。
4. 新增 `ControlMixer`，统一做 PWM 叠加、限幅和舵机分配。
5. 将当前水平 PID 迁移为 `PixelDeltaPwmPidAction`。
6. 新增 `GuidanceOrchestrator`，把 main loop 中的业务接线迁入 orchestrator。
7. 再接入 `ImuSetpointSmoothAction` 和 `ImuStabilizeAction`。

完成迁移后，`guidance_controller` 这个大而全模块应被拆散或降级为历史兼容层；长期目标是由 `GuidanceOrchestrator + ControlMixer + ControlAction` 取代它。
