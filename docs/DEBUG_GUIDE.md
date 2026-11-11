# 板端调试手册（v2 工作流）

状态：可执行的调试手册。当前仓库已有可部署 ELF（`launcher`、`dart_app`）与
构建/部署脚本，启动链路见 [`runtime/launcher/README.md`](../runtime/launcher/README.md)。
**板端尚未验证**：凡本文档中的板端结论都必须以真实串口日志与样本为准。

这份手册服务于“用户态业务程序”的板端验证，不调试固件本身。每次试验记录 Git commit、应用名、参数版本、板端固件版本、工具链、日志目录和原始样本。

## 0. 先确认当前状态

部署前必须满足：

- 业务入口位于 `runtime/dart_entry.cpp`，应用装配位于 `runtime/application.cpp`，
  检测、控制、录像组件位于 `src/{detector,control,recorder}/`，分别由 runtime 和 src 的 CMake 声明；
- `bash .script/build-rtos <目标名>` 能生成 ELF；
- 交叉构建、链接脚本（SDK `link.lds`）和 SDK 依赖明确；
- 版本化参数和日志 schema 已写入应用配置；
- 板端有安全的停止/回收路径（`--seconds` 或 SIGINT/SIGTERM，退出逐 step 留日志）。

当前可直接部署的两个目标：`launcher`（进程启动与监督）、`dart_app`（业务应用，当前接入采集、识别和录像）。
板端 msh 里的第一步永远是 `/sdcard/app/launcher -t`（只校验清单，不启动进程）。

## 1. 排查顺序

```
构建/部署 → 进程启动 → 日志落地 → 资源探针
 → 采集/帧池 → 二值化/识别 → IMU/观测器 → 控制/舵机
 → 录像/取证 → 有界退出和资源回收
```

一次只验证一个层级。没有采集和帧池健康证据时，不排查识别；没有识别/IMU 时间戳时，不闭合控制；没有安全退出时，不尝试 VENC 或长时间运行。

## 2. 构建与部署

```bash
bash .script/build-rtos
bash .script/deploy-rtos
```

传输方式和地址通过 `.script/board.env` 或环境变量配置，详见
[脚本说明](../.script/README.md)。早期 FUSE 覆盖的零字节实验仅适用于当时路径，
不构成禁止 GIO/MTP 部署的工程约束；当前脚本以上传后的完整回读校验判断成功。

```text
配置 SDK/toolchain → 构建一个 src/ 下的目标 → 获得 build-cross/<目标名>/<目标名>
→ 通过 scp/读卡器部署到 /sdcard/app → 在 RT-Smart 用户态运行
→ 板端先手动跑，再注册 launcher 开机自启
```

部署清单至少包含 ELF、应用参数、日志配置和 schema 版本。不要把固件镜像、BSP 或 SDK
输出目录提交进业务仓库。首次验证建议带有界运行时长（`--seconds`）和禁用录像的开关
（`--record off`）。板端第一次接触新配置时，先 `/sdcard/app/launcher -t` 校验清单。

## 3. 日志契约

日志由应用/运行时的决策边界写出，叶子 C 模块只返回状态。每条结构化错误包含：

```text
[功能标签] ts_ms level module op code native detail action count msg="..."
```

日志使用单调时间、稳定错误枚举和限频；正文缓冲 512 字节，带标签的整行槽位 640 字节，
截断后仍以换行结束。正常“未检测到目标”、ROI 无连通域、候选被拒绝只进入计数器/CSV，
不刷错误日志。信号处理器只做异步信号安全的最小写入。

启动日志应依次证明：应用版本/参数 → SDK/设备探针 → 帧池和工作区 → 线程/队列 → 第一帧时间戳。任何一步停住，先定位最后一个成功边界。

### 实时查看日志

默认只写文件。前台运行时显式打开后台控制台镜像：

```text
msh> /sdcard/app/dart_app --seconds 30 --record off --pwm off --log-console on
msh> /sdcard/app/dart_app --seconds 30 --record off --pwm off --log off --log-console on
```

第二条只输出控制台。镜像写到 stderr，即当前固件提供的控制台设备；通过 launcher 启动时，
stderr 被收集到 `<app>.console.log`，需要串口现场观察时应前台运行。
镜像限速 30 行/秒、初始突发 30 行，超额累计并以 `op=console_dropped` 汇报；
文件日志队列满或竞争锁时累计 `op=log_dropped`，避免串口或磁盘阻塞采集线程。
`write.pause` 暂停时文件与镜像一起停；普通日志、检测 CSV、录像、IMU Session 与 launcher
均不显式调用文件同步，也没有周期性 stdio 刷新。复制录像和 CSV 前应先停止写入、等待关闭。

`[vision] op=vision_status` 与 `[app] op=timing` 给出窗口内采集/检测/流水线耗时；
`[record] op=recorder_timing` 和 `op=recorder_io` 给出会话累计的编码、复制、写盘耗时。
`op=buffer_status` 的 `rec_buffered / rec_buffer_high_water / rec_buffer_dropped` 用于判断
短时写盘停顿是否耗尽缓冲。`tools/rec_stall_report.py` 可离线分析 PNGS、H.264、DBV、PGM/PBM，
分别报告输入间隙、成功保存源帧间隙及 `buffer_full` 数量；CSV 时间戳不是磁盘写入完成时间。

## 4. 采集和帧池

验证项目：

- 实际 sensor 模式、分辨率、stride、帧时间戳和帧间 gap；
- VB/MMZ 池大小、缓存属性、DMA owner 和 invalidate/flush 位置；
- 帧槽状态是否严格按 `free → captured → processing → queued → released` 迁移；
- 运行态是否出现分配、双重归还、永久占槽或队列满；
- 处理线程不直接做文件 I/O。

证据应包含每秒帧率、丢帧/重复帧、最老输入年龄、池高水位、队列高水位和一次原始帧/stride 样本。没有这些数据时，“fps 正常”没有意义。

## 5. 二值化、识别和跟踪

先用固定帧/探针验证标量参考，再验证 RVV，最后接真实曝光：

1. 二值化输出尺寸、极性、stride 和越界；
2. 全图扫描与标量参考逐值一致；
3. ROI 窗口坐标是绝对坐标，闭区间/半开区间只选一种并写入 ABI；
4. 启动确认、跟踪、丢失、重捕获状态迁移符合 [`DETECTOR_DESIGN.md`](DETECTOR_DESIGN.md)；
5. 记录候选数、拒收/贴边/出框、ROI 面积、扫描 p50/p95/max 和测量年龄。

`cx/cy=-1` 之类的占位约定不要带入 v2；使用 `valid` 和状态枚举。真实视觉质量必须用原始图、结果 CSV 和板端时间戳复盘，主机合成图只能验证逻辑。

## 6. IMU、观测器和控制

接线顺序：先读 `latest` 和 history 的有效位/时间年龄，再接观测器，再接姿态内环，最后开放外环和舵机。每个控制拍记录：实际 `dt`、IMU 年龄、视觉年龄、模式、状态有效位、限幅前后指令、舵效状态和恢复动作。

发现 NaN、时间戳回退、视觉长断、`authority_lost`、`ill_posed` 或执行器反馈缺失时，必须进入安全退化；禁止用默认姿态/默认目标继续闭环。舵面阶跃先在台架确认符号、零位、PWM 和速率，再接飞行试验。

## 7. 录像与取证

当前 v2 的录像**只有灰度取证**：`include/firmware/src/recorder/` 把帧拷成紧凑 Y8 后写
`records/rec_NNNN/img/frame_XXXXXXXX.pgm`（或按阈值打包的 `.pbm`），并在
`records/rec_NNNN/logs/frames.csv` 每帧一行记录放行决定；`dart_app` 另外写
`records/detection.csv`（每帧的识别结果与扫描窗口）。硬件 H.264/VENC 在 v2 尚不存在。

录像是异步数据面，不得阻塞采集或控制。记录帧号、源时间戳和是否重复，不能用图像哈希判断重复帧。队列满可以丢样本，但必须累计丢弃数（`rec_busy`/`event_dropped`）；写线程不在栈上放大对象，不逐包反复映射/解映射。

MPP/VENC 行为高度依赖固件。没有通过资源探针和小规模送帧/取码流验证前，不要进入长时间录制；失败后优先保留原始帧/CSV 取证，不要盲目拆链。任何 SDK stop/destroy 的风险都应写入 ACL 的恢复策略。

## 8. 退出和故障定位模板

每次运行都要有有界停止条件。退出顺序：停止输入 → 停止消费者 → 排空/标记队列 → 写摘要 → 回收 SDK 资源。每一步前后写一条日志，最后一条日志能定位卡死位置。业务进程崩溃时保留信号、应用版本、最后成功边界和资源计数。

故障记录模板：

```text
现象：
应用/commit/固件：
复现参数和时间窗：
最后一条成功日志：
原始样本路径：
推测根因（标注待验证）：
下一次只改变的一个变量：
```

## 9. 禁止事项

- 不要启动多个独占 VICAP/VB 的应用；
- 不要把 SDK 头文件带入业务层；
- 不要在视觉/控制线程写文件、无限等待或动态扩容；
- 不要用占位值掩盖无效传感器、拒收观测或舵效丢失；
- 不要把主机通过当作 RVV、cache、曝光、MPP 或舵机已通过；
- 不要为了“修日志”顺手改固件或删除失败样本。
