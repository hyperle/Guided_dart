# 应用装配与组件执行

`runtime/dart_entry.cpp` 是板级进程入口，通过设备工厂选择相机、PWM、IMU 和网络适配。
`application.hpp/.cpp` 将这些接口连接到功能组件，`argument_parser.hpp/.cpp` 解析启动参数。
`dart_application` 和 `dart_app` 目标由 `runtime/CMakeLists.txt` 声明。

| 文件 | 职责 |
|---|---|
| `component.hpp` | Component 生命周期、输入输出端口、配置与注册接口 |
| `tunner.hpp` | Tunner 持有一个组件和执行线程，提供停止、回收、运行结果 |
| `application.hpp/.cpp` | Application 组装具体组件，共享资源、通道、诊断和故障隔离 |
| `argument_parser.hpp/.cpp` | 命令行参数校验及录像配置映射 |
| `recording_trigger.hpp/.cpp` | IMU 自动录像触发策略，只发布内存请求 |
| `dart_entry.cpp` | main() 注入板级工厂并调用 run_application() |

## 接线与运行

```text
相机 ICamera
  -> Detector（应用线程唯一取帧）
       -> Blackboard.target（Armor 优先；无目标为 -1,-1）
            -> 控制组件（独立 Tunner，333 Hz）-> 唯一 PWM 设备
       -> 识别完成帧的只读视图 -> 可选 Wi-Fi 分支
       -> 租约非阻塞移交 -> FrameChannel -> Recorder（独立 Tunner，240 Hz）
                                             -> 编码 -> 存储
                                             -> 原帧归还给相机线程
IMU Session -> 缓存快照 -> Blackboard.imu_state -> 控制组件
```

`Application` 知道具体组件和它们的依赖，`Tunner` 只依赖 `Component` 接口。
组件在执行线程启动前初始化，线程中调用 `run()` 并执行 `shutdown()`。
`Tunner::finished()/error()` 可并发查询；`start/request_stop/join/stop` 由生命周期所属线程调用。
运行错误优先于退出错误报告，组件、设备和通道在 join 完成前保持有效。

Detector 的逐帧循环在应用线程调用 `step()`；相机创建、初始化、取帧、释放和销毁均在该线程。
每轮取帧前后都排空录像归还的租约。录像忙时通道返回拒收，Detector 自己释放当前帧并继续。
交接只移动 FrameLease，不复制像素；相机 RAW10 解包、编码结果和 Wi-Fi 固有的复制另计。

控制组件从 Blackboard 读目标，不接收 Detector 回调。无目标或目标超过
`--servo-timeout-ms` 未更新时，目标修正归零；滚转补偿继续来自姿态计算。
`--servo-max-delta` 只约束水平、垂直目标修正的合成增量，滚转补偿不参与该限幅。
IMU 采样保持独立；控制读取 Session 缓存并更新 Blackboard，不访问 IMU 设备。

## 配置和输出

完整参数见 `dart_app --help`。原有曝光、识别、四路舵机、录像、Wi-Fi 和 IMU 参数由新入口承接。
默认录像格式为 PGM、目标帧率 60；实际启动清单
[launcher/startup.list](launcher/startup.list) 当前使用 PNGS、30 FPS。
默认四路初始脉宽为 `1445,1490,1150,1475` µs，目标修正上限为 150 µs。
未显式提供 `--aim` 时，装配层从相机实际尺寸取得图像中心。

检测 CSV 的字段与帧身份由 Application 组织，通过有界异步 CsvWriter 输出。
应用日志中的 `module=servo op=status pulse_us` 是实际成功下发的四路值；
录像统计包含 accepted、busy、timed_out 和租约释放错误，不再有原帧复制。

应用创建唯一 WriteControl，供录像、普通日志、检测 CSV、IMU 日志共用。
自动录像触发继续运行在 IMU 采样线程，只发布内存请求。暂停和恢复见
[写入控制](WRITE_CONTROL.md)。网络配置和主机服务见
[Wi-Fi 接收端](../tools/wifi_receiver/README.md)。

## 故障与退出

相机创建、初始化或连续取帧失败后停止采集分支并清理资源，进程与 IMU 继续运行至时限或信号。
PWM 创建、初始化或写入失败只关闭控制输出；录像和识别继续。
录像空间、编码、写盘失败只关闭录像写入；识别和控制继续。
Tunner 的运行与退出错误由应用监控并记入日志。

正常退出时停止控制组件，再停止提交新帧，等待 Recorder 完成退出并归还原帧，
在相机线程排空归还槽，随后关闭 Detector 与相机。归还失败保留租约供重试；
最终清理也在相机销毁前完成。录像会话和编码资源始终先于相机释放。

## 验证状态

当前组件接线完成主机行为测试与 RISC-V 完整 dart_app 静态链接。
主机覆盖图像到 Blackboard/PWM/录像、相机线程中的租约归还、取帧阻塞期间的独立控制、
目标丢失与超时、滚转不限幅，以及相机、录像、PWM 故障隔离。
新装配逻辑的真实波形、目标效果、存储延迟与长期吞吐仍需板端验证。

以下为旧流水线的历史记录，不能作为当前单帧租约通道的吞吐证据：
2026-10-20 连续 PBM 录像共 13,017 帧、55.881 FPS，文件在 499,995,987 字节停止，
下一整帧将超过 500,000,000 字节。当时使用固定像素复制队列，首、中、末样本均为全黑图，
没有验证目标识别质量。证据保存在 `records/board-integration-2026-10-20/`。
2026-10-18 的相机故障隔离记录位于
`records/board-integration-2026-10-18/fault-isolation-*.log`。
