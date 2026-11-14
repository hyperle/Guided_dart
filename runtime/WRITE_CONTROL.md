# 录像与日志的写入控制

本仓库管理的录像、结构化日志、检测 CSV、IMU 日志和 launcher console 日志共用外部暂停开关。
相机采集、视觉识别、IMU 采样和积分不随暂停停止。开关作用于落盘路径。

## 外部开关与清理

板端默认检查 `/sdcard/app/control/write.pause`：任何类型的同名对象存在就暂停，
不存在才允许写；检查出错也保持暂停。启动时先检查开关，暂停状态跨复位保留。
环境变量 `DART_WRITE_PAUSE` 可覆盖板端路径；launcher 和应用须使用相同设置。
主机脚本的路径为所选程序目录下的 `control/write.pause`，自定义板端路径时需自行操作对应开关。

```bash
bash .script/write-rtos off            # 暂停录像与日志，不等待板端应答
bash .script/write-rtos on             # 显式解除全部暂停，恢复 runtime 当前请求
bash .script/clean-rtos-data           # 设置暂停后直接扫描、清理；成功后释放本次暂停
bash .script/clean-rtos-data --keep-paused
bash .script/clean-rtos-data --dry-run  # 只预览，不修改开关
```

脚本支持现有 SSH、读卡器、GIO/MTP 三种访问方式。脚本创建 `write.pause/` 目录，
手动暂停在其中持有 `manual` 文件；每次清理创建自己的 `cleanup.<pid>.<随机数>` 文件。
完成时只移除自己的持有文件，仅在目录为空时删除暂停目录，避免释放其他任务的暂停。
直接通过文件管理器创建一个普通 `write.pause` 文件也有效，脚本保留这个外部暂停。

清理不创建或等待停止应答。应用每 50 ms 检查开关，写入服务约每 20 ms 检查内存请求，
随后排空已接收的数据并关闭文件；具体收尾时间取决于队列和存储 I/O。
RT-Smart 的 `lstat` 在录像负载下曾对不存在的开关成批返回 0 且不填写 `st_mode`；
应用把这种无文件类型的结果视为未知并保持上一状态，launcher 的单次判断仍按暂停处理。
因此清理保留原有写句柄/目录变化检查，尚在关闭或正在写入的目标仍可能被跳过。
清理有跳过项、失败、中断或指定 `--keep-paused` 时保留本次暂停，输出对应持有文件路径；
处理完成后可以运行 `write-rtos on` 显式解除全部暂停，包括失败清理保留的暂停；
也可以单独删除自己的持有文件，并删除已经为空的 `write.pause/` 目录。

## Runtime 接口与生命周期

`platform::WriteControl` 在独立线程轮询外部开关；runtime 调用只更新原子内存状态：

```cpp
writes.request_recording(true);       // 开始请求录像
writes.request_recording(false);      // 撤销录像请求
writes.request_logging(false);        // 应用普通日志、检测 CSV、IMU 日志停止落盘
writes.request_logging(true);         // 恢复应用日志
```

录像、应用日志分别满足“功能已配置、外部未暂停、runtime 正在请求、存储正常”才写入。
`--record off` 不准备录像服务；默认 `--record on` 持续请求录像。
runtime 日志请求控制本应用；独立 launcher 的日志服从外部暂停开关。
暂停期间的新增日志和图像不积压补写；日志队列排空，录像尚未开始写入的输出记为
`stopped` 后关闭文件，正在进行的单帧写入正常完成。

- `RecorderFactory::create(..., &writes)` 创建录像组件，由 `executor::Tunner` 独立驱动。
  组件执行线程管理会话和编码；`BufferedStorage` 的独立线程消费固定容量输出队列，
  串行写图像和事件文件。队列满时丢新输入，逐帧编码/提交与采集不等待文件 I/O。
  暂停关闭图像和元数据文件，恢复时创建新的 `rec_NNNN`，不覆盖旧轮次。
  相机线程继续归还租约；下一轮仅在旧租约全部归还后启动。
  关闭先停止 Runner，再在相机线程排空 `FrameChannel`；重新创建组件前须无在途租约。
  统计按录像会话重新计数，缓冲继续复用。容量上限或写入错误保持停录，
  持续请求不会反复重开；撤销请求后重新请求才开始下一轮，并重新检查空间。
- `imu::Session::start(..., &writes)` 让采样生命周期与日志会话独立。
  关日志只关闭日志线程持有的文件，采样、积分、参考系、generation 不重置。
  恢复时按现有编号规则新建文件，并重新计算该文件的 32 MiB 预算。
  `Session::stop()` 仍用于最终停止整个采样服务。受控模式下日志打开失败也继续采样。
  `log_path()` 可在会话稳定时或最终 stop 后读取，切换期间不跨线程读取可变路径。
- 普通 Logger 后台每批关闭句柄，空闲及暂停时也关闭；恢复后继续追加原路径，缺失目录自动创建。
  固定 256 行队列锁竞争或满时丢新行，文件和控制台开关独立，外部暂停同时静音两者。
  `flush(deadline)` 只等待队列及在途输出结束并关闭文件，不提供持久化屏障。
  检测 CSV 使用启动期分配的 256 行队列，每行最多 1023 字节，满队列丢行并计数；
  无论是否传入 WriteControl，都由后台线程管理文件。恢复追加原文件，文件被清理后重建并写列名。
- launcher 通过管道收集子进程 stdout/stderr，子进程不持有 console 日志文件句柄。
  暂停时继续消费并丢弃输出，恢复后再追加文件。`-n` 或前台超时留下运行中的应用时，
  launcher 交由独立收集进程继续消费管道，直到所有管道 EOF。

普通日志、检测 CSV、录像、IMU Session 日志和 launcher 日志均不显式调用 `fsync`，
也不设置周期性 `fflush`。stdio 缓冲满时正常写出，关闭时由 `fclose` 写出尾部；
`write` 路径关闭描述符。后台写入及关闭仍可能等待文件系统，退出等待排空可能耗时，
不保证异常断电后的数据持久性。独立 `imu_init_probe` 的实验输出不属于应用日志链路。

## 可选的加速度触发录像

`--record auto` 等待 IMU 原始加速度模长持续超过显式配置的阈值。
必须同时配置 `--imu on --imu-bus N --imu-cs N`、`--rec-trigger-accel N`（整数 m/s²）和
`--rec-trigger-ms N`（正整数毫秒）。没有默认触发门限，参数需要实测标定。
`--rec-duration-ms N` 可指定触发后请求录像的时长，默认 0 表示持续请求。

`RecordingTrigger` 在 IMU 采样线程观察原始样本，只发布内存请求，文件工作在后台执行。
它独立于估计器的积分接纳条件，因此估计器因加速度阶跃拒绝积分不屏蔽录像触发。
加速度非有限、加速度饱和或低于阈值时重置连续计时；非单调时间戳或采样间隔超过
5 个目标采样周期（当前 1 kHz，即 5 ms）也重置计时，不能用间断样本补足持续时间。
每次进程运行最多触发一次；到达时长后撤销请求，外部暂停始终优先。

触发时间以单调时钟 ns 保存在 `WriteControl::recording_request_ns()`，
应用状态日志的 `record_trigger_ns` 可与录像 CSV 的 `received_us` 对齐。
目前没有触发前缓存，首帧延迟包含开会话和存储准备时间，须在板端量测。

## 验证

主机 `write_control_test` 覆盖三种录像格式的暂停/恢复、新会话、删除目录后恢复、
两个 runtime 请求独立切换、暂停中持续 IMU 采样和参考系保留、原始加速度连续触发及定时停止。
`recorder_session_test` 验证有未释放租约时禁止重启，释放后重新创建组件并保留旧文件。
`launcher_write_control` 验证正常监督及 `-n` 模式下 console 暂停、恢复与持续消费。
脚本回归覆盖三种传输、手动暂停保留、清理前设置暂停和失败后保持暂停。

板端 FAT/MTP 删除、实际停止延迟、触发到首帧延迟、真实 IMU 阈值和长期反复启停仍待板端验证。
