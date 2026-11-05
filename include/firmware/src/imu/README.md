# imu/ — 设备抽象、估计器与 K230 适配器

分层（全部纯 C++，不含任何 POSIX / K230 SDK 头文件）：

| 文件 | 层 | 内容 |
|---|---|---|
| `math_types.hpp` | L3 数据 | 向量与四元数，字段直接表达数学值 |
| `sample.hpp` | L3 数据 | 原始计数、采样值、单位和时间戳 |
| `interface.hpp` | L3 抽象 | `IImu` 采样源接口 |
| `estimator.cpp` | 计算 | `Estimator`、嵌套的 `Config` / `Estimate` / `SeedResult` / `QualityEvent` 及无分配估计逻辑 |
| `k230/iim42652.cpp` | L2 适配器 | `k230::Iim42652`，唯一包含 `spi/iim42652.h` 的 C++ 类 |

`Iim42652` 对应 IIM-42652，底层只接受 WHO_AM_I `0x6f`，实际型号由初始化日志记录。
初始化会等待六轴有效原始数据，超时返回 `kReadyTimeout`；失败时仍可读取完整阶段诊断。

`IImu` 只是采样源：`read_sample()` 返回原始 ADC 计数、SI 值和一个单调时间戳。
加速度是比力（m/s²），角速度是 rad/s，均在传感器/机体坐标下。寄存器配置和
总线访问由 `spi/` 的 C 驱动负责，适配器只做生命周期、错误码翻译和句柄所有权；
实例之间没有共享可变全局状态。

`Estimator` 是非虚、无堆分配的计算对象。把接受的样本喂给 `update()`，它给出
四元数姿态、RPY 弧度、参考系角速度、重力补偿加速度、积分速度和展开后的累计
旋转矢量。成功的 `seed()` 要求样本静止：它快照重力、清零速度和累计旋转，并
递增 `generation`，消费者可据此丢弃旧坐标系下的缓存。四元数把传感器轴映射到
选定参考系；`reset()` 清空估计并推进 generation。

估计器质量门限会拒绝带标志/饱和、非有限、越限和突变步进的样本，不把它们积分
进去。超过 `max_integration_interval_ns` 的时间戳间隔只上报并用于重置下一个积分
区间。阈值都在 `Estimator::Config` 里；默认 seed 门限要求重力模长误差在 2 m/s²
以内、角速度低于 0.15 rad/s。

运行时的独立采样与原始/积分数据日志由
[`runtime/imu/Session`](../../../../../runtime/imu/README.md) 复用上述接口实现。
上层统一通过 Session 的 `current()` / `seed()` / `reset()` 读取和重新积分，
与日志共用同一个估计器和 generation。`dart_app` 可通过 `--imu on` 显式启用。
六轴 IMU 的偏航角靠陀螺积分，
没有绝对航向传感器就会漂移；速度同样会受加速度计零偏和姿态误差影响。

消费者包含 `imu/estimator.hpp`、`imu/interface.hpp` 或 `imu/k230/iim42652.hpp`。
CMake 的 `dart_imu` / `dart_imu_k230` 为静态库，普通 `.cpp` 实现参与最终链接。

## 构建

```bash
cmake --preset host    # 主机编译 IMU 模块和行为回归，供 clangd 使用
cmake --preset cross   # DART_IMU_K230=ON：dart_imu + dart_imu_k230（链接 dart_iim42652）
```

`DART_IMU_K230` 在 `include/CMakeLists.txt` 的 spi 一节声明，因此那份文件里 spi 一节
必须排在 imu 一节之前（适配器链接 `dart_iim42652`）。交叉构建下两个 C++ 目标都带
`-fno-exceptions -fno-rtti`。
