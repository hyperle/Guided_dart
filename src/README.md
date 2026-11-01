# 业务源码

`src/` 全部属于 L4，按功能划分调用、编排与策略。设备初始化、线程、文件系统、
编码和日志写入的实现放在 `include/firmware/src/`，通过公开接口供 L4 调用。

| 路径 | 职责 |
|---|---|
| `dart_entry.cpp` | 唯一业务进程入口，将板级工厂注入启动流程 |
| `startup/` | 参数解析、服务组装、故障隔离和运行顺序 |
| `vision/` | 相机采集生命周期、逐帧识别、帧分发及视觉舵机策略 |
| `recording/` | 录像参数映射、启停决策和采集线程中的租约排空 |
| `diagnostics/` | 相机错误的业务日志字段和检测 CSV 字段映射 |
| `detector/` | 灯/装甲板检测与 ROI，只消费帧数据 |
| [`control/`](control/README.md) | 四路独立初始位置、左右/上下像素到 PWM 增量映射与合成限幅 |

`startup/` 通过注入的设备工厂调用下层服务，不处理硬件配置和 POSIX 资源。
`vision/` 决定帧如何交给识别、录像和 Wi-Fi 回传；`recording/` 决定何时启停录像，
不拥有写线程或编码器；`diagnostics/` 定义业务记录内容，不管理文件缓冲与同步。
`vision/visual_servo.cpp` 调用 `control/` 合成绝对脉宽并发布给下层 PWM 任务，
不直接访问引脚、设备寄存器或系统线程；控制不依赖 IMU。
检测器只导入 `dart.camera.frame_view`，构建目标只依赖 `dart_camera_types`；
不持有相机、不创建设备、不操作帧租约。
Wi-Fi 只在采集线程调用下层 `streaming::Session::try_submit`，压缩、网络和恢复线程
实现位于 [`streaming/`](../include/firmware/src/streaming/README.md)，由板级工厂注入传输接口。
所有 L4 模块均不包含 K230/MPP 或 POSIX 实现头文件。

`src/CMakeLists.txt` 声明 `dart_detection`、`dart_control` 和 `dart_application` 静态模块库，
所属 `.cpp` 独立编译并参与应用/测试最终链接；领域库不继承应用的设备或运行时依赖。
板级、日志、存储和录像目标由 `include/CMakeLists.txt` 声明；
`runtime/` 中保留现有 IMU 采样行为，launcher 负责进程监督。

应用参数、数据流和退出契约见 [`startup/README.md`](startup/README.md)。
