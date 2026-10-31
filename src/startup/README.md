# dart_app —— 板端业务应用

[`../dart_entry.cpp`](../dart_entry.cpp) 提供唯一业务进程入口，将板级设备工厂注入
`run.cpp` 的 `run_application()`。`argument_parser.cpp` 定义参数、默认值和校验；
`run.cpp` 创建唯一日志对象并组织启动、运行和退出；`services.cpp` 管理服务故障隔离。
本目录与其余 `src/` 一样属于 L4 调用层，不包含设备或系统资源的实现。

| 功能 | L4 调用与策略 | 下层实现（`include/firmware/src/`） |
|---|---|---|
| 设备组装 | `src/dart_entry.cpp` 注入工厂，`startup/` 调用服务 | `board/k230.cpp`、`camera/`、`imu/`、`spi/` |
| 视觉流程 | `vision/capture_pipeline.cpp` 管理采集顺序、识别和帧分发 | `camera/` 管理帧池与硬件资源 |
| 纯视觉舵机 | `vision/visual_servo.cpp` 调用 `control/` 两轴映射与限幅 | `pwm/` 管理独立输出任务、超时与 K230 HAL |
| 录像 | `recording/frame_recording.cpp` 映射参数、决定启停并排空租约 | `recorder/` 管理队列、线程、编码和文件 |
| 诊断 | `diagnostics/` 定义相机错误字段和检测 CSV 行 | `logging/` 管理文件缓冲和同步 |
| 进程时限与退出 | `startup/run.cpp` 决定调用顺序 | `system/` 管理信号、时钟和 launcher 退出标记 |

业务目标统一在 `src/CMakeLists.txt` 声明，下层库由 `include/CMakeLists.txt` 声明。
各模块包含带保护的 `.cpp`，不提供 C++ 头文件。
`runtime/launcher/` 负责进程启动与监督；现有 `runtime/imu/` 采样服务本轮保留。
单进程可以有多个线程或任务：当前相机调用保持串行，
IMU 采样独立于相机逐帧循环；PWM 任务每 10 ms 检查视觉指令，输出 333 Hz 波形。

应用组装一个 `WriteControl` 供录像、日志和 IMU 共用，外部暂停不停止采集和积分。
`--record auto` 可使用显式配置的原始加速度阈值与持续时间请求录像，恢复写入支持新建会话。
命令、接口和待板端验证事项见 [写入控制](../../runtime/WRITE_CONTROL.md)。

## 相机采集、识别与录像为什么放在同一个进程

相机 ACL 在 `create()` 里自建并独占 `dart_media`（全局 VB 初始化 + VICAP/VB 私有
输出池），同一时刻只允许一个消费者。v1 实测把同类程序在清单里放多条，5 个实例
几乎同时抢 VB/VICAP，只有 9/28 次写出日志，其余静默死掉。因此：

- 清单里**只能有一条** `dart_app`；
- "识别"和"录像"在同一进程内完成，不做跨进程帧共享；
- 手工调试时不要与 launcher 同时运行。

## 流水线

```
SC038HGS RAW10 -> CSI/MCM（绕过 ISP）-> RAW10 >> 2 灰度解包 -> 归还 RAW dump
camera->acquire(timeout) -> FrameLease（独立的 0..255 灰度 Y8 缓冲）
  -> detector.Detect(lease.view())      灰度候选提取与背景对比度过滤；同步借用像素
  -> servo.observe(result)             装甲板优先/灯回退 -> 两轴增量 -> 合成限幅
        PWM Session                   独立输出任务；过期或无目标回各路初始脉宽
  -> recorder.try_submit(std::move(lease))
        kAccepted     -> 已拷贝到固定队列；录像线程默认写灰度 PGM，可选灰度 PNG / 二值 DBV、PBM
        其他          -> 采集线程自己 release()
  -> recorder.drain_releases()          提交前后调用：已拷贝帧的租约在采集线程释放，
                                        保证相机 ACL 的调用始终串行
```

三个取帧结果分别计数，语义不同，都不能当错误：

| `AcquireStatus` | 含义 | 处理 |
|---|---|---|
| `kFrame` | 拿到一帧 | 走上面整条流水线 |
| `kNoFrame` | 超时内没有帧 | 正常否定结果，`no_frame++` |
| `kLeaseExhausted` | 租约槽用满，驱动根本没被触碰 | 容量性丢弃，`lease_exhausted++` |

K230 实现管理 CSI、模式、输出尺寸与驱动缓冲池，组合根只选择实现并传入曝光控制。
采集流程消费 `ICamera`，根据只读 `frame_info()` 分配识别和录像空间，再消费 `FrameLease`。
模式由固件驱动的可用模式表选择；CSI 来自固件的 `CONFIG_MPP_SENSOR_DEFAULT_CSI`。

租约容量是编译期参数（当前 `Camera<4>`），取 4 是为了
让"识别中/录像拷贝中/写盘中"的帧彼此不互锁。

## 参数

```
dart_app [选项]
  --seconds N      有界运行秒数，0 = 跑到 SIGINT/SIGTERM（默认 0）
  --exp LINES      固定曝光行数（默认 400；0 保留传感器初始曝光）
  --raw-layout F   packed10-lsb（默认，K230 实板验证）
  --vision on|off  识别开关（默认 on）
  --pwm on|off     纯视觉 PWM 输出（默认 on；vision off 时停用）
  --aim X,Y        全图预设像素（默认实际图像中心）
  --servo-initial A,B,C,D     四路初始脉宽 µs（默认各 1520）
  --servo-min A,B,C,D         四路最小脉宽 µs（默认各 1420）
  --servo-max A,B,C,D         四路最大脉宽 µs（默认各 1620）
  --servo-horizontal A,B,C,D  左右增益 µs/像素（默认各 0）
  --servo-vertical A,B,C,D    上下增益 µs/像素（默认各 0）
  --servo-timeout-ms N       无有效视觉指令时回初始位置（默认 200 ms）
  --record on|off  录像开关（默认 on）
  --rec-keep N     只保留最近 N 轮录像，0 = 不清理（默认 0）
  --rec-fps N      目标录像帧率，按时间采样（默认 60 FPS）
  --rec-format F   pgm（灰度，默认）| png（灰度）| dbv / pbm（二值）
  --rec-dir DIR    录像根目录（默认 /data/app/records）
  --det-csv PATH   识别结果 CSV；off 关闭（默认关闭）
  --threshold N    灰度候选阈值 0..255（默认 50），不改变相机灰度图
  --min-contrast N 候选与背景最小灰度差 0..255（默认 12，0 关闭）
  --log PATH       结构化日志；off 只关文件日志（默认 /data/app/logs/dart_app.log）
  --log-console on|off  日志同时写 stderr，前台串口调试用（默认 off）
  --imu on|off     IMU 独立采样与日志（默认 off）
  --imu-bus N      SPI 总线 0..2，启用 IMU 时必填
  --imu-cs N       片选引脚 0..63，-1 使用 HAL 默认，启用 IMU 时必填
  --imu-log PATH   IMU 日志基名，每次启动加编号（默认 /data/app/logs/imu.log）
  --imu-ignore-id on|off  跳过 WHO_AM_I 值校验并记录原始寄存器（诊断用，默认 off）
  --status-ms N    状态行间隔（默认 1000）
  --acquire-ms N   单次取帧超时（默认 200）
  --max-acquire-errors N  连续取帧错误上限，超过即退出（默认 8）
```

应用不提供采集分辨率、采集帧率或 CSI 请求参数。`--rec-fps` 仅设置录像按时间抽帧的目标速率。
曝光/阈值默认值均为 UNVALIDATED，要用真实板端样本标定，见 [`docs/DEBUG_GUIDE.md`](../../docs/DEBUG_GUIDE.md) §5。
PWM 默认增益为零，目标偏移不会改变脉宽。标定后通过上述参数启用各路修正，
数值列表不含空格，通道顺序和映射符号见 [舵机控制](../control/README.md)。
纯视觉控制不依赖 IMU，当前启动清单默认关闭 IMU。

## 故障隔离

相机创建、启动或连续取帧失败时，停用采集及依赖图像的识别/录像/PWM，记录
`action=disable_capture` / `op=degraded`。应用继续等待原定 `--seconds` 时限或
SIGINT/SIGTERM，独立 IMU 线程继续采样、积分、写日志；不反复重启整个进程。
`--seconds 0` 下即使所有设备不可用也保持运行，周期记录状态，直到收到停止信号。
录像启动失败只停用录像；识别工作区准备失败停用识别与 PWM；IMU 初始化失败只停用 IMU。
PWM 初始化或写入失败只停用输出，识别和录像继续；取帧错误/无帧或检测失败请求初始位置，
采集或写日志阻塞时，独立 PWM 任务仍按 `--servo-timeout-ms` 检查过期并回初始位置。
`log_camera_error()` 只写文件日志，写入失败时保持静默，CLI 参数错误仍返回错误码。

新板 IMU 接线为 SPI1、CS IO14，测试时追加 `--imu on --imu-bus 1 --imu-cs 14`。
如需诊断 ID 异常，可再追加 `--imu-ignore-id on`；复位前后实际 ID 和 14 字节原始
快照会写入应用日志，即使初始化失败也保留。配置读回、有效样本检查和传输错误处理继续生效。
新版固件已接入 SC038HGS，后端通过 MCM 直采 RAW10 并解包为灰度 Y8，详见
[`camera/README.md`](../../include/firmware/src/camera/README.md)。应用默认通过接口设置
400 行曝光，后端同步设置 VTS=600；`--exp` 单位为行，0 保留传感器初始曝光。
这条控制路径无需标定像素时钟或修改固件，RAW 路径不支持 `--ae`。
开机清单使用 400 行曝光；此前已验证 RAW10 收帧与二值录像落盘；本次灰度路径及对比度参数需重新上板标定。
驱动未报告帧率时也允许使用已验证的图像尺寸，消费者不依赖采集帧率。

## 输出

```
/data/app/logs/dart_app.log                        结构化日志（状态与耗时两行共用一次 fsync，写完关闭文件）
/data/app/logs/imu_NNNNNN.log                      每轮独立的 IMU 原始样本、积分快照和重积分事件
/data/app/records/rec_NNNN/img/recording.pgm         单文件灰度连续流（默认）
/data/app/records/rec_NNNN/logs/frames.csv           每帧一行的放行记录（Recorder 写）
/data/app/records/detection.csv                    每帧一行的识别结果（本程序写）
```

程序部署在 `/sdcard/app`，不再部署或加载 ISP XML/JSON。录像及日志默认使用 `/data` 数据分区；
本板 MTP 实测 `/sdcard` 约 524 MB、`/data` 约 2.83 GB，分别对应
`store_ffff0001`、`store_ffff0002`。旧 `/sdcard/app/records` 不自动搬迁或清理。

`detection.csv` 列：`sequence,received_us,source_pts,kind,valid,cx,cy,area,radius,quality,clipped,missed_frames,candidates,full_frame,rescanned,win_x0,win_y0,win_x1,win_y1,admission`
（窗口右下边界是**开区间坐标**，与 `ScanMetadata::search_window` 一致。）

IMU 需要显式指定硬件接线，例如在确认 SPI0 和 HAL 默认片选适用后追加
`--imu on --imu-bus 0 --imu-cs -1`；`--imu-log` 可改日志基名，每次启动创建新编号文件，
例如 `imu_000001.log`、`imu_000002.log`；IMU start/stop 状态行给出实际路径。默认关闭，
当前纯视觉启动清单不启用 IMU，需要时按新板配置 SPI1 / CS IO14。
采样为独立线程，启动时等待静止样本建立单位四元数参考系；
后续重新积分复用同一 Session 的 `seed()` / `reset()`，`current()` 返回日志里的
同一份 Estimate。原始计数、SI 数据、积分状态、质量位及 generation 都写入日志，
格式与接口见 [`runtime/imu/README.md`](../../runtime/imu/README.md)。
SPI 时钟 1 MHz、超时 20 ms、1 kHz ODR/轮询、±16 g / ±2000 °/s 与零陀螺偏置
均为 UNVALIDATED，实际采样率与写盘吞吐待板端验证。

日志是"结构化单行"，正常否定结果（无目标、ROI 无连通域、候选被拒）**不进日志**，
只进 CSV 与计数器：

```
ts_ms=... level=INFO module=app op=status elapsed_s=10 frames=298 fps=29.8 no_frame=2
  lease_exhausted=0 target=37 detect_err=0 rec_input=298 rec_acc=75 rec_down=223 rec_busy=0
  rec_queued=1 rec_pending=0 rec_written=74 rec_event_dropped=0 release_err=0 run_hw=512
```

## 退出契约

有界运行（`--seconds`）或 SIGINT/SIGTERM 后，按固定顺序收尾，每一步都留日志：

```
step=input_stop      主循环出界（reason=signal|deadline|abort）
module=servo op=stop 停止输出任务并关闭四路 PWM（启用时）
step=recorder_stopped  停止录像（先停消费者；此时相机仍持有租约，不能先 stop 相机）
step=leases_drained   排空 release 队列，租约全部在采集线程释放
step=camera_stopped   camera.stop()
step=summary          全部计数器 + Recorder 统计 + 工作区高水位
step=camera_destroyed 相机析构（deinit → destroy → media destroy）
step=exit            相机流程结束
module=imu op=stop   启用时停止 IMU、排空日志队列并同步文件
```

由 launcher 以 `&!` 拉起时，程序清理完相机及可选 IMU 服务后写入 launcher 提供的 `.done`
标记。launcher 据此启动下一轮；仅靠板端 `waitpid` 和 PID 存活探测会误判。

顺序不能颠倒：`recorder` 仍持租约时 `camera.stop()` 会返回 `BUSY`。板端卡死时，
最后一条 `step=` 就是定位点。

## 构建与上板

```bash
bash .script/build-rtos dart_app          # -> build-cross/dart_app/dart_app
msh> /sdcard/app/dart_app --seconds 10 --record off --pwm off  # 先只验证采集+识别
msh> /sdcard/app/dart_app --seconds 10                  # 再验证录像落盘
```

## 验证状态

纯视觉 PWM 链路已通过主机的完整图像到 PWM 回归、目标丢失/超时与故障隔离测试，
并完成 RISC-V `dart_app` 交叉构建；真实波形、机械方向及非零增益仍需板端标定。

当前默认已改为紧凑 1-bit DBV，`/sdcard/app` 启动、`/data/app/records` 保存。
每轮 DBV 文件（包括共享文件头）最多 500,000,000 字节，下一张放不下即停录、识别继续；
`/data` 分区总占用上限为 2,000,000,000 字节，包含旧录像、日志及分配开销，
并在总量与物理空间内保留 64 MiB，不足时停录且保留旧文件。
DBV 采用空间优先的 level 9 压缩，板端实际帧率需重新测量。
以下 PBM 数字为此前版本实测，不能作为新 DBV 的帧率。

2026-10-20，当前完整工作区代码交叉编译通过，主机 14 项回归全部通过，应用经 MTP
写入 `/sdcard/app/dart_app` 并完整回读校验。录像和日志写入 `/data/app`，连续复位分别
创建 `rec_0001`、`rec_0002`、`rec_0003`，保留之前轮次，不自动重启续录。

此前连续 PBM 版本 `rec_0003` 的 640×480 PBM 录像按全部接收时间戳计算为 **55.881 FPS**
（目标 60 FPS）；共 13,017 帧，偏移连续，每帧 38,411 字节。文件在 **499,995,987 字节**
停止，下一整帧会超过 500,000,000 字节；`op=recorder_stop reason=size_limit`、`code=none`。
未产生续录文件，队列忙丢帧为 0。固定 16 帧像素队列约 4.92 MB，吸收写盘停顿，
拷贝完成即归还相机租约；实测拷贝均值 162.6 µs、PBM 打包 1.67 ms、I/O 10.43 ms。
停录后持续取帧和识别，至开机 381 秒仍无取帧、识别或释放错误。

首、中、末完整帧已回读并解码，均为全黑二值图，`target=0`。这些结果验证采集、录像
和识别循环运行，尚未验证目标识别效果；需要在有照明的目标场景标定曝光与阈值。
本固件的 MTP 无法读取仍被录像器打开的文件；录像达到上限关闭后可回读。
板端证据保存在 `records/board-integration-2026-10-20/`。

2026-10-18 的故障隔离记录保存在
`records/board-integration-2026-10-18/fault-isolation-*.log`，覆盖相机创建、启动、取帧失败后
IMU 继续采样，以及 SPI 创建失败后的清理；当时 SPI1 / CS IO14 的 ID 读取尚未通过。

以下板端记录来自旧板及迁移前流水线。

- 已通过（2026-10-15，CanMV 001000000 板端）：CSI2 上电自启、1920×1080 PGM
  完整回读、90 秒分段后的完成标记与自动重新启动、仅保留最近两轮。
  本次复位后上一轮 43 张和当前轮 9 张均完整，零字节文件数为 0。
  黑暗样本平均灰度 0.55；此结果不代表识别质量或目标曝光已标定。
- 已通过（2026-10-13，本机）：交叉构建（`-march=rv64imafdcv -mabi=lp64d`，
  `-fno-exceptions -fno-rtti`，静态链接 RT-Smart ELF，约 1.7 MB）。
- **待板端验证**：目标场景的曝光/阈值标定、RVV 与 cache/DMA 的完整压力测试、
  高帧率下的丢帧与长期稳定性；识别质量与性能尚未验证。
- 本程序**不**包含硬件 H.264/VENC：v2 的默认录像为 DBV，兼容 PNG 与旧 PGM/PBM 流。

## 历史 90 fps 配置与耗时统计（2026-10-15，旧板）

当时的旧启动清单请求 `--acq 1280x720 --fps 90 --csi 2`（这些参数现已移除），输出跟随传感器尺寸，
识别与录像继续使用 v2 实现。没有合并 v1 分支或移植其检测器。
`camera_create` 日志的 `sensor` 是精确匹配后回读的模式；它不是实测帧率。
`status fps` 是本轮累计处理帧率，`timing fps` 是当前窗口处理帧率。

`op=timing` 每个状态周期报告取帧、识别和后处理的平均/最大微秒数：
`acquire_us` 包含等待驱动出帧及 cache invalidate；`detect_us` 只覆盖 Detect；
`pipeline_us` 覆盖录像放行、释放租约、CSV 行写入及归还队列；
`telemetry_us` 是上一个周期状态日志写入/同步及 CSV 刷新的耗时。
90 fps 每帧预算约 11111 us，需要结合这些数值定位 CPU 或 I/O 开销，
不能仅凭请求档位断言实际能处理 90 fps。下述数据已完成板端验证。

当时的启动清单通过 `--rec-fps 30` 注入目标录像帧率，Recorder 按单调接收时间采样，
采集档位从 30 fps 改到 90 fps 时仍以每秒 30 张为目标。未指定 `--rec-fps` 时，
程序沿用 `recorder::Config::fps` 的默认值。启动日志的 `rec_fps` 与
`op=recorder_start fps`、录像 CSV 首行的 `fps` 可核对实际注入值。
输入不足或队列忙时实际落盘帧率会降低，队列忙仍会丢弃录像帧并计数。
原 `--rec-every` 参数已移除。

板端验证：仅把采集模式改到 720p90 时，一轮 7624 个处理帧，CSV 接收时间戳
计算为 84.7 fps。分阶段统计显示无目标场景 Detect 平均约 1224 us，
后处理平均约 1466 us，后处理单帧峰值 128068 us。
此前每 60 帧录一张的配置中，为识别 CSV 设置 64 KiB 缓冲后，连续四个 90 秒完整轮次处理
8008 / 8012 / 8011 / 8006 帧，约 89 fps；后处理平均约 63 us，
识别仍约 1214 us。四轮取帧/识别/写入/释放错误及录像 busy 均为 0，
每轮写出 134 张 PGM。状态日志与 CSV 周期刷新仍会产生停顿；
这些结果只验证黑暗无目标场景吞吐，不代表复杂目标场景的最坏耗时。
证据见 `records/board-performance-2026-10-15/`（本地录像目录，不提交 Git）。
上述吞吐数据来自旧抽帧配置。30 FPS 配置已于 2026-10-15 上板：启动日志与
录像 CSV 均确认目标为 30 FPS，连续六个完整轮次实际落盘约 5.4～6.1 FPS。
其中 `rec_0073` 的 CSV 跨度 89.868 秒，共 6528 个输入帧、512 张已写图片，
实际约 5.7 FPS，录像忙丢弃 1710 帧、队列高水位 2；采集、拷贝、写入、释放与
metadata 错误均为 0，已回读完整 1280×720 PGM 样本。参数注入已生效，当时的
逐帧文件写盘链路不能维持 30 FPS。此结果不代表当前连续文件实现的性能。证据见
`records/board-recorder-fps30-2026-10-15-175445/`（本地目录，不提交 Git）。
