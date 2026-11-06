# 功能组件

`src/` 按功能组织组件、配置和计算方法。应用装配与通用组件执行接口放在
`runtime/`；设备、文件系统和厂商适配实现放在 `include/firmware/src/`。

| 路径 | 职责 | 构建目标 |
|---|---|---|
| `detector/` | Detector 唯一取帧，调用 Detect/Track，向 Blackboard 发布像素坐标 | `dart_detection` |
| `control/` | 控制组件读取 Blackboard，计算四路舵面并写 PWM | `dart_control` |
| `recorder/` | Recorder 消费帧通道，独立组织编码和文件输出 | `dart_recording` |

每个功能的执行终端实现在所属 `.cpp`，方法类与配置按消费者需要放在 `.hpp`。
检测计算借用只读帧视图；Armor 优先，无目标坐标为 `(-1, -1)`。
控制的目标修正按配置限幅，滚转补偿在限幅之后叠加，保持不限幅。
录像通道只接管一张原帧，忙时拒收；原帧由相机线程归还，识别不等待录像。

`src/CMakeLists.txt` 声明三个功能库，所属源文件独立编译并参与真实应用链接。
应用入口、接线、调度和退出顺序见 [应用运行逻辑](../runtime/APPLICATION.md)。
