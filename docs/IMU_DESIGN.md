# IMU 模块设计基线

状态：历史接口/并发设计目标；当前已有 IMU 驱动、估计器和采样服务。
实际接口见 `include/firmware/src/imu/README.md` 与 `runtime/imu/README.md`；
下述目标 ABI 和 seqlock/history 方案不代表当前实现，也不是本轮迁移要求。

IMU 模块是 L2/L3 的数据服务，不属于控制算法。原厂 C 驱动在 ACL 内完成采样、标定和时间戳；上层只接收稳定的 C ABI 或 C++ 抽象，不包含厂商头文件。

## 1. 职责和数据流

```
原始样本（单生产者）
  → 标定：零偏、温漂、轴变换、量程标志
  → 姿态/角速度积分
  → 比力减重力得到线速度（姿态有效且未饱和时）
  ├─ latest：控制环快速读取的 seqlock 快照
  └─ history：按时间戳查询区间角度/速度增量
```

姿态积分只依赖角速度；线速度积分读取同一模块维护的姿态，不能让两个消费者各自重复积分。采样不等周期时一律使用实际 `dt`；“按控制拍的离散模型”是控制器的另一套约定，不能混用。

## 2. 公共数据类型（目标 ABI）

```c
typedef struct {
    float gyro[3];
    float acc[3];
    uint64_t timestamp_us;
    uint32_t flags; /* saturated, checksum_error, gap ... */
} dart_imu_raw_t;

typedef struct {
    float angle[3];
    float rate[3];
    float velocity[3];
    uint64_t timestamp_us;
    uint32_t valid_flags; /* angle/rate/velocity */
} dart_imu_state_t;
```

正式 ABI 需增加 `struct_size`/版本字段并用 C/C++ 静态断言校验尺寸、偏移和对齐。角度、角速度、速度、加速度和时间单位必须在头文件写明；不要用 `0` 表示无效值。

## 3. 并发和存储

生产者只有一个，消费者可以有多个；模块内部无互斥锁阻塞实时环：

- `latest` 使用偶数/奇数序列的 seqlock；读者复制前后比较序列，超过固定重试次数返回 `STALE`。
- `history` 使用 2 的幂容量环形缓冲；写入样本和累计角度后以 release 顺序发布 head。
- 消费者先复制一个有界窗口，再在副本上插值；复制期间若被覆盖，返回 `OVERWRITTEN`。
- 任何跨窗口的 gap、时间倒退、零时间间隔、超过最大跨帧数，都返回明确状态，不部分计算。
- 环形缓冲的大小由采样率、最大视觉延迟和控制器查询窗口计算；启动日志记录实际字节数。

建议初始参数（均为 `UNVALIDATED`）：

```
IMU_HISTORY_CAPACITY = 512  # 2 的幂
IMU_GAP_US            = 20000
IMU_WINDOW_MAX        = 64
```

不得直接复制这些值上板；要由真实采样 gap 和内存预算重新确定。

## 4. 查询接口语义

建议提供：

```c
dart_status_t dart_imu_get_latest(dart_imu_state_t *out);
dart_status_t dart_imu_get_raw_latest(dart_imu_raw_t *out);
dart_status_t dart_imu_get_delta_angle(uint64_t start_us, uint64_t end_us,
                                        float out_delta[3]);
dart_status_t dart_imu_get_delta_velocity(uint64_t start_us, uint64_t end_us,
                                          float out_delta[3]);
```

`get_delta_angle` 可在加速度计饱和时继续工作；`get_delta_velocity` 只要区间内出现饱和、gap 或速度尚未种入，就必须失败。结束时间超出最新样本时可按策略截断，但必须设置 `CLIPPED/STALE` 标志并由调用方决定是否接受，不能静默当作精确查询。

初始参考系和速度种入是明确事件：

```c
dart_imu_set_reference();
dart_imu_seed_initial_velocity(const float velocity[3]);
```

`set_reference` 只改变对外姿态基准，不清除历史累计角；`seed` 成功后才置 `velocity_valid`。出仓初速、坐标转换、零偏和温漂都要通过配置/标定记录来源。

## 5. 积分和姿态表示

最小实现可以在小采样角下逐轴梯形积分，但实现者必须记录适用范围。若角速度或机动超出范围，应改用四元数/旋转矩阵，并保持对外接口不变。重力转换使用同一姿态快照，不能把不同时间的姿态和比力拼在一起。

时间处理：

- `dt <= 0`、超过 gap 上限或时间戳回退：不积分，发布 gap 标志；
- 角度仍可在加速度计饱和时更新；速度停止积分并置无效；
- 任何传感器校验错误、FIFO 溢出和丢样本进入计数器与事件日志；
- 不在采样回调中写文件、格式化长字符串或等待其他线程。

## 6. 消费者约定

- 控制环消费 `latest`，同时检查 `timestamp_us` 年龄和有效位；
- 观测器按视觉曝光/控制时刻查询区间角度和速度，不能假定帧间隔是控制周期整数倍；
- 视觉去自旋若只需偏航增量，应由一个包装接口统一坐标符号，避免多处重复扣除；
- 诊断线程读取原始快照和 gap 计数，不干扰生产者。

## 7. 验证计划

主机只验证积分数学、时间边界、seqlock 重试、环覆盖、饱和语义和坐标变换；不模拟厂商中断。交叉构建验证 ABI 与无异常约束。板端验证真实采样率、时间戳抖动、缓存一致性、量程、温漂和控制环最老数据年龄。

验收日志至少包含：样本序号、采样/接收时间、校验和、三轴原始/标定值、饱和/gap、当前姿态/速度有效位、history 覆盖范围、查询失败原因和重试次数。所有参数和参考系事件写入试验元数据。
