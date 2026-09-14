# green_led_ai —— 绿灯识别 + H.264 录像（单进程双子系统）

K230/K230D RT-Smart 业务应用：**VICAP 唯一持有者**，内部两条**完全解耦**的流水，
可以分别单独启停、互不影响。两个产物（同一套源码）：

| 产物 | 大小 | 后端 | 用途 |
|---|---|---|---|
| `green_led_ai` | ≈0.9MB | `color`（RVV + CIELAB 查表阈值，无模型） | 默认；识别+录像跑稳这条 |
| `green_led_ai_kpu` | ≈5.2MB（链 nncase） | `color` + `kpu`（AI2D 零拷贝 + KPU 乒乓 + 解码/NMS） | 接 kmodel 用，见第 8 节 |

```text
sensor(gc2093) 1920x1080@30
  └─ VICAP(dev0, OFFLINE, 唯一一条通道 CHN0 640x480 NV12)
       ├─ vis_cap 线程: 连续 dump（长超时，不节流）-> 单槽信箱(新帧优先)
       │     └─ vis_proc 线程: cached 映射+失效 -> RVV CIELAB 查表阈值 -> 连通域
       │            └─ record_offer_frame(): 交棒给录像（收下则本线程不再归还）
       └─ 录像子系统（共享该帧）
             ├─ 送帧线程: 确定性抽样(30->25) -> kd_mpi_venc_send_frame -> 在途 FIFO
             │            （抽样丢/送失败 -> 自己归还；这是"不反压"的关键）
             ├─ 取流线程: get_stream -> 码流拷进 2MB RAM 环 -> 归还对应在途帧
             │            -> release_stream（唯一归还者，1:1 对应）
             └─ 写盘线程: 会话开关/滚动/容量淘汰 + fwrite + fsync(1 秒 1 次)
```


两条通道**各自持有自己的 VB 帧，不共享任何一帧**；这是本工程与旧工程
`green_led_rtos` 最本质的区别，也是旧工程「跑几帧就把内核/VICAP 打死」的根因修法。

---

## 0. 设计依据：哪些是本板实测事实，哪些是未验证假设

**实测事实（本板，2025-09 连续多轮上板）**

| 事实 | 证据 |
|---|---|
| 单通道 + OFFLINE + **连续长超时 dump** + 应用交棒 = 稳定 | 旧工程 `green_led_rtos` 同形态跑 40079 帧、28fps |
| **双通道 + CHN1 硬件绑定 VENC** 会在 200~400 帧后衰减到停 | 四次运行 `dumped` 每 2 秒增量 58→37→29→6→0，随后 CHN1 的 `streams` 也冻结 |
| 该 VICAP 在"此刻没帧"时返回 `0xa0158010`（module=0x15 VICAP、errid=16 NOTREADY） | 错误码解码 + `noframe=0 / notready=~100/s` |
| 帧归还没有失败过（所有权纪律是对的） | `rel_fail=0` 全程 |
| SD 写入不是停流原因 | RAM 环修好后 SD 基本没写盘，仍然衰减到停 |
| RVV 颜色检测正确且在跟踪目标 | `selftest rvv-vs-scalar PASS`；`detect FOUND center=(317,304) px=2204`；`det=883us`（ROI）|
| 端到端延迟稳态 ≈1.1ms | `lat=1114/1136us` |
| 单帧算力上限：全画面 2.6ms / ROI ≈0.87ms | `bench` 行 + 实测 `det`（对应 385fps / 1150fps）|
| **不支持** AF_UNIX 控制 socket（`Out of memory`） | `control socket failed` —— 运行期启停只能用启动参数/信号 |
| `dump 超时`路径要尽量避开 | 我上一版"节流 26.6ms + 超时 5ms < 帧间隔 33.3ms"→ 每帧必超时一次，与本板的衰减时间线吻合 |

**因此本轮重构的取舍**：默认走**单通道共享**（不用第二条通道），dump 用**连续 + 150ms 长超时**（不节流），并**删掉**了"停流时自动解绑"这类会给出假结论的自动处置。

## 1. 为什么旧工程会死（根因，有日志证据）

旧工程日志（`/tmp/board-logs-run9/green_led_rtos.log` 等）里稳定复现：

```text
trace det seq=1 mapped y=0x1009ab000 uv=0x1009f6000 len=460800/0
...
trace cap seq=3 feed-begin
trace cap seq=3 feed-done ret=1          # 该帧已交给 VENC
...
green_led_rtos: vicap dump failed ret=-1609203696 (fail=1)   # 之后永久失败
```

`-1609203696` = `0xA0158010`。按 `mpp/include/k_errno.h` 解码：

| 位域 | 值 | 含义 |
|---|---|---|
| APPID | `0xA0000000` | `K_ERR_APPID` |
| module | `0x15` | K_ID_VICAP |
| level | 4 | `K_ERR_LEVEL_ERROR` |
| errid | 16 | `K_ERR_NOTREADY` |

即 **`kd_mpi_vicap_dump_frame` 永久返回 VICAP_NOTREADY**：通道再也拿不出帧。
run7 更是直接卡在 `feed-begin` 不再前进。

**原因**：采集线程把 CHN0 dump 出来的**同一帧**做了两件互相冲突的事：

1. `kd_mpi_venc_send_frame()` 送进编码器 —— 成功后该 VB 块归编码器所有
   （MPP 里没有 `kd_mpi_venc_release_frame()`，编码器用完自行归还）；
2. 识别线程随后又 `kd_mpi_vicap_dump_release()` 释放了同一帧。

一次「借出」被「归还」两次 → VB 块引用计数被打崩 → 几个帧之后通道缓冲耗尽，
dump 永久 NOTREADY。而且 `kd_mpi_vicap_dump_release` 当时没有和
`kd_mpi_venc_send_frame` 共用同一把锁，两个线程可以真正并发。

同一份日志还暴露了第二个坑：`kd_mpi_sys_mmap/munmap`、`kd_mpi_venc_send_frame`
与 `get_stream/release_stream` 在多线程并发时会把 RT-Smart 打挂（旧工程注释里
写得很清楚，也是 29 次重启循环的来源）。

### 本工程对应的三条铁律

| 铁律 | 实现位置 |
|---|---|
| **一帧只有一个所有者、只有一条释放路径**：送 VENC 成功即所有权转移，应用绝不再 release；失败/被丢弃才由应用 release 一次 | `record.c: rec_cap_thread()`、`vision.c: vis_cap_thread()/vis_proc_thread()` |
| **所有 MPP 调用全进程串行**（含 mmap/munmap、vicap dump/release、venc send/get/release） | `app.c: mpp_enter()/mpp_leave()`，所有调用点都包在临界区里 |
| **物理地址映射一次、永不逐帧解映射**：按 phys 缓存 VA，只在初始化线程里 mmap，直到所有线程 join 后才统一 munmap；同一物理块长度不一致直接报 FATAL | `app.c: mpp_map_persist()` / `mpp_shutdown()` |

日志里可以直接验证这三条：`status` 行末尾的
`mpp calls=... contended=... maxhold=...us maps=...`
稳定运行时应为 `contended=0`、`maxhold` 远小于 1ms、`maps` 很快停止增长
（稳态不再 mmap）。

---

## 2. 启动与日志（沿用现有模式，未改）

| 项 | 位置 |
|---|---|
| 开机自启 | `/sdcard/app/launcher` 读 `/sdcard/app/startup_final.list`，行尾 `&` 后台拉起 |
| 应用日志 | `/sdcard/app/logs/green_led_ai.log`（每行 open→write→fsync→close，MTP 随时可读） |
| 子进程 stdout | `/sdcard/app/logs/launcher-child.log` |
| pid | `/sdcard/app/green_led_ai.pid` |
| 控制 socket | `/sdcard/app/green_led_ai.ctl` |
| 录像 | `/sdcard/app/recording/rec_%04u.h264` + `rec_%04u.csv` |

部署：`bash scripts/deploy-sdcard.sh <SD卡挂载点> --keep-logs`
（会拷 `launcher`、`green_led_ai`、`green_led_ai_kpu`、`board_probe`、`kpu_bench`、
SDK 示例 `best.kmodel` 和 `startup_final.list`；`--keep-logs` 保留已有录像与日志）。

> ⚠️ **别用板端 MTP（gphoto2/gvfs）覆盖已有文件**：实测它会把文件删掉却写不回来
> （`cp` 还返回 0），随后整个挂载点掉线。新增文件可以走 MTP，覆盖/替换请拔卡用读卡器。

---

## 3. 运行时控制（视觉/录像各自独立启停）

```text
# 板端 msh（或用 socat/nc 连 socket）
vision on | off | status          # 单独启停视觉，不影响录像
record on | off | status          # 单独启停录像（off = 当前会话收尾并暂停常录）
record cap <bytes>                # 目录容量上限
record dir <path>                 # 换录像目录（下次会话生效）
status                            # 一行汇总 + 一行探针
start | stop                      # 等价于 record on / record off（兼容旧脚本）
```

信号：`kill -USR1 <pid>` 开始常录、`kill -USR2 <pid>` 停止常录。

> 通道是在**启动时**按 `--vision/--record` 配置的：`--record off` 启动时连 CHN1
> 都不配置（省一份 VB），此时运行期再 `record on` 会被明确拒绝并提示重启。
> 这是刻意的：运行期重配置 VICAP 通道风险高（旧工程的坑就在这条路上）。
>
> 两个例外值得知道：
> - `--vision off --record on` 时仍会给 CHN0 配一个 160x120 的最小占位通道
>   （2 个缓冲约 56KB）—— 官方示例从来都是 CHN0+CHN1 一起配，只配 CHN1 是没人
>   验证过的组合，用占位换掉这个未知风险；
> - `--detector kpu` 会把视觉通道切成 `RGB_888_PLANAR`（AI2D 的 NCHW uint8 输入
>   正好是 RGB 平面张量），此时 color 后端不可用（它读的是 NV12）。

---

## 4. 内存预算（按最终选型 K230D 128MB 设计）

DDR 总 128MB，SDK 切成 `RTSMART_SIZE=0x4400000`(68MB) + `HEAP 16MB` +
`MMZ 60MB`（见 `configs/k230d_rtos_evb_defconfig` 与 `K230 内存布局指南`）。
媒体/AI 的大块内存全部来自 **MMZ**，所以下表按 MMZ 记账（1920x1080@30 采集、
两级输出各 6 个 buffer）：

| 用途 | 计算 | 占用 |
|---|---|---|
| VICAP dev（OFFLINE 采集缓冲） | 6 × 1920×1080×2 | ≈ 24.9 MB |
| CHN0 视觉输出环形缓冲 | 6 × 640×480×1.5 | ≈ 2.8 MB |
| CHN1 录像输出环形缓冲 | 6 × 640×480×1.5 | ≈ 2.8 MB |
| VENC 输出码流池 | 8 × 640×480 | ≈ 2.5 MB |
| **合计（当前配置）** | | **≈ 33 MB / 60 MB** |

余量留给：kmodel（NanoDet-Plus 约 1–3MB）、AI2D/KPU 输入输出乒乓缓冲
（320×320 输入两张 ≈ 0.6MB + 输出两张 ≈ 0.2MB）。128MB 平台完全放得下。

> 真的紧张时优先降 `dev_attr.buffer_num`（6→4 约省 8MB），**不要**动通道
> `buffer_num`：通道缓冲越少，录像抽样/识别抖动越容易丢帧。

---

## 5. 需求对照（逐条说明实现方式与实测口径）

| 需求 | 本工程做法 | 怎么在板上确认 |
|---|---|---|
| 1. 沿用现有启动/日志模式 | launcher + startup_final.list + 每行 fsync 的日志 + pid + ctl socket，一字未改 | `logs/green_led_ai.log` 首行 `start pid=... argv=[...]` |
| 2. MMZ/物理内存零拷贝 | 录像侧：采集帧物理地址直接进 VENC，零 CPU 搬运；视觉侧 color 用 `kd_mpi_sys_mmap` **一次性**映射 + VA 缓存；kpu 侧：`hrt::create(..., copy=false, pool_shared, phys)` 把 VICAP 帧**物理地址**直接当 AI2D 输入 tensor | `status` 的 `maps=` 稳态不增长；`kpu` 行里 ai2d 耗时不随图像大小暴涨（说明没 memcpy） |
| 3. 大核专用 | 见下方说明：**整个 RT-Smart 就跑在 1.6GHz 的 RVV 大核上**，无需也不能够从用户态按线程绑核；改为**职责隔离 + 线程优先级**（视觉链路不碰 SD/编码器，录像慢只丢自己的帧） | `board_probe` 实测 CPU 频率与 RVV 加速比 |
| 4. 异步事件驱动 + 乒乓 | 同步后端(color)：单槽信箱 + 新帧优先，采集永不等待处理；异步后端(kpu)：`submit()` 只投递、`collect()` 取结果，pre 线程做 AI2D 与 kpu 线程跑 KPU **双缓冲乒乓重叠**，等 KPU 完成的那次 `poll()` 在 KPU 线程里由**硬件中断**唤醒（不是轮询） | `status` 的 `kpu ai2d=… kpu=… post=… lat=… fps=…` 一行；`drop=`/`err=` |
| 5. 视觉/录像解耦、可分别启停 | 两条 VICAP 通道 + 两套线程 + 两套 VB + 独立开关；录像 25fps 抽样，视觉按传感器帧率跑 | `--vision off` / `--record off` 可分别启动；socket 可运行期切换 |

### 关于「绑核到 Core 1」（需求 3）的事实说明

- 硬件：K230D 是双核 C908 —— **CPU0 = 800MHz（RV64GCB，无 RVV，L2 128KB）**，
  **CPU1 = 1.6GHz（RVV 1.0，128bit 向量单元，L2 256KB）**（K230 数据手册）。
  你说的「Core 1 = 1.6GHz + RVV」是对的。
- **RT-Smart 本来就跑在 CPU1 上**：U-Boot `board/kendryte/common/k230_boot.c` 按
  镜像 TOC 的 boot 字段选核，`genimage-sdcard.cfg` 里 rtt 分区是 `boot = 0x3`
  → `(0x3 & 0x6) >> 1 = 1` → `k230_boot_core(1, ...)`；U-Boot 自己在 hart0 上最后
  `while (1) wfi`。所以业务代码（含视觉流水）**已经独占 1.6GHz 大核**。
- **按线程绑核在当前 SDK 做不到**（不是配置问题）：
  - `RT_USING_SMP` 未开启（`bsp/maix3/.config` / `rtconfig.h`），且 C908 端口
    没有 SMP 移植：没有 `cpuport_smp.c`，`rt_hw_secondary_cpu_up()` 全树只有
    k210 实现，`sbi_hsm_hart_start()` 有定义但无人调用 → 第二个核没有可调度的
    RT-Smart 线程；
  - `rt_thread_control(thread, RT_THREAD_CTRL_BIND_CPU, ...)` 是**内核 API**，
    用户态链接不到；
  - 用户态唯一的 `sched_setaffinity()` → `sys_setaffinity()` → `lwp_setaffinity()`
    会把**整个进程的所有线程**绑到一个核（不是单线程），且非 SMP 下等于空操作。
- 结论与做法：**不改固件**，用「职责隔离 + 线程优先级 + 让硬件引擎干活」达到同样
  效果 —— 预处理交给 AI2D、推理交给 KPU、编码交给 VENC、搬运交给 DMA，CPU 只做
  调度和 RVV 后处理；视觉链路的线程优先级高于录像链路，录像慢只丢自己的帧。
  要真正双核并行（CPU0 跑录像/系统、CPU1 跑视觉），需要给 C908 移植 SMP 或做
  AMP 双镜像并重烧固件，属于另一个独立工程。

### 关于「异步、不轮询」（需求 4）的实现层次

KPU/AI2D 在 SDK 里已经是**中断驱动**的，不需要我们轮询寄存器：

- 内核驱动 `bsp/maix3/drivers/interdrv/gnne/gnne_dev.c`（KPU，IRQ 16+173）与
  `ai2d_dev.c`（AI2D，IRQ 16+175）都实现了 `poll()`：中断处理函数里
  `rt_wqueue_wakeup()` + `rt_event_send()`，`poll` 回调里 `rt_event_recv(WAITING_FOREVER)`；
- nncase 运行时的 `runtime_function.device.cpp` / `ai2d_builder.cpp` open 的就是
  `/dev/gnne_device`、`/dev/ai_2d_device` 并调用 `poll()` 等结果；
- 所以「发起推理 → 线程挂起 → 完成中断唤醒」这条链路是现成的，`interpreter::run()`
  期间线程确实让出 CPU。本工程在此之上做**流水级乒乓**：让 KPU 算第 N 帧时，
  CPU 侧后处理第 N−1 帧，从而把「KPU 时间 + CPU 时间」从串行变重叠。

---

## 5.5 不上板也能做的验证

```bash
# 宿主机跑检测器正确性测试（灰色不命中 / 灰底绿灯中心误差<=1px / 全画面与 ROI 一致）
bash apps/green_led_ai/test/run_host_test.sh
```

板上 RVV 路径不靠宿主机验证，而是**应用开机自检**：`detect_color.c` 里
`det_selftest()` 会造一张含灰底/绿块/渐变的 NV12 小图，用 RVV 路径与标量路径
各扫一遍，逐位比较掩码、命中数、Y 统计与探针，结果写在日志第一行：

```text
green_led_ai: selftest rvv-vs-scalar PASS (mask_mismatch=0 green=... probe=(...) rgb=(...) y=[..,..])
```

## 6. 板端自查（先看这几行，缺哪行就是卡在哪一步）

```text
green_led_ai: start pid=2 argv=[/sdcard/app/green_led_ai]
green_led_ai: probe request csi=2 1920x1080@30 (适配表精确命中要求)
green_led_ai: sensor actual 1920x1080@30 type=23          # 出现 WARN = 没命中期望模式
green_led_ai: vicap dev=0 mode=offline acq=1920x1080@30 vis=640x480 rec=640x480 ae=0 awb=1 dnr3=1
green_led_ai: exposure fixed 200 us (range 27-33315 us) rc=0
green_led_ai: detector=color lab(L>=12,A<=-20,B>=8) roi=(160,120,320,240) step=(2,2) pix_min=50 [RVV on]
green_led_ai: selftest rvv-vs-scalar PASS (...)          # ★ RVV 路径与标量逐位一致
green_led_ai: vision thread started (chn=CHN0 640x480) enabled=1
green_led_ai: rec thread started chn=CHN1 640x480 fps=25 bitrate=12000 kbps dir=/sdcard/app/recording 常录 on
green_led_ai: rec session rec_0001 start (dir=/sdcard/app/recording cap=8589934592)
green_led_ai: running (vision=on record=on)
```

之后每 `--status` 秒一行汇总：

```text
green_led_ai: status vis=on fps=29 center=(312,241) found=120 lost=3 det=180/900us lat_max=4200us
              drop=1 dump_fail=0 | rec=on auto=1 sess=1 fps=25 in=600 sent=150 written=149 drop=450
              pending=0 bytes=12345678 dir=/sdcard/app/recording |
              mpp calls=4200 contended=0 maxhold=180us maps=9
green_led_ai: probe det_avg=180us det_max=900us lat_max=4200us thr L>=12 A<=-20 B>=8 step=(2,2) ...
```

怎么读：

| 现象 | 结论 / 处理 |
|---|---|
| `vis fps` ≈ 传感器帧率且 `drop` 很小 | 识别跟得上 |
| `center=(-1,-1)` 且 `probe_rgb/lab` 离阈值很远 | 放宽 `--lab` / 调 `--expo-us`，口径与旧工程一致 |
| `det=` 远小于 5000us | 满足「单帧处理 <5ms」；后续换高速相机时看这个数 |
| `dump_fail` 持续增长（尤其录像侧） | 该通道缓冲被占满：检查 SD/编码速度，或降低 `--rec-fps`/`--bitrate` |
| `mpp contended` 不为 0 | 有并发 MPP 调用，需要复盘调用点（不允许出现） |
| `maps` 持续增长 | 出现逐帧 mmap，必须修（不允许出现） |
| `rec sent/streams/written` | 三者应同步增长（sent≈streams≈written） |
| `sample_out` | 按 25fps 抽样主动丢掉的帧数（正常，≈sent/5） |
| `send_fail` / `ring=` | 应为 0；`ring=` 增长说明 SD 撑不住（只影响录像） |
| `fifo_ovf` / `fifo_udf` | **必须恒为 0**：非 0 说明"送进去的帧"和"取出来的包"失去 1:1，帧归还会错位 |
| `dto=`（dump 空转累计） | 稳态应几乎不增长；猛涨说明帧率低于预期或流水异常 |

CSV 列（与旧工程一致，便于沿用离线脚本）：

```text
frame_seq,result_seq,mono_us,center_x,center_y,detect_fps
```

主机侧封装（帧率要和录制时一致）：

```bash
ffmpeg -f h264 -framerate 25 -i rec_0001.h264 -c copy rec_0001.mp4
```

---

## 7. 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `--csi <0-2>` | 2 | CSI 号；庐山派摄像头在 CSI2 |
| `--fps <n>` | 30 | 探测请求帧率，必须与 1920x1080 成对（适配表精确命中） |
| `--vision <on\|off>` | on | off 则**不配置 CHN0**、不建视觉线程 |
| `--record <on\|off>` | on | off 则**不配置 CHN1**、不建录像线程 |
| `--vis-out <WxH>` | 640x480 | 视觉通道输出尺寸 |
| `--rec-out <WxH>` | 640x480 | 录像通道输出尺寸 |
| `--roi <x,y,w,h>` | 160,120,320,240 | 识别 ROI；`0,0,0,0` = 全画面 |
| `--step <x,y>` | 2,2 | 采样步长；`1,1` = 逐像素（走标量路径，最准） |
| `--lab <L,A,B>` | 12,-20,8 | CIELAB 阈值，与 Python 原型同一套数值 |
| `--pix-min <n>` | 50 | blob 最小像素数 |
| `--miss <n>` | 3 | 连续丢失 n 帧后整幅重扫；0=关 |
| `--expo-us <us>` | 200 | 固定曝光；0=不设；`--ae` 开自动曝光 |
| `--keep-gain` | 关 | 默认把增益压到最低 |
| `--rec-mode <shared\|bind>` | shared | shared=单通道、识别交棒（**默认，本板验证过**）；bind=双通道硬件直连（实验特性） |
| `--dump-timeout <ms>` | 150 | dump 等待上限。dump 是"阻塞等下一帧"，给足即可（短超时会让通道劣化） |
| `--rec-fps <n>` | 25 | 录像抽样帧率（shared 模式下由应用确定性抽样，30→25 即每 6 帧丢 1 帧） |
| `--bitrate <kbps>` | 12000 | H.264 码率 |
| `--rec-dir <path>` | /sdcard/app/recording | 录像目录 |
| `--cap <bytes>` | 8GB | 目录总容量上限，超限删最旧会话 |
| `--detector <color\|kpu>` | color | 检测器后端（kpu 需 `green_led_ai_kpu` 产物 + kmodel） |
| `--kmodel <path>` | — | kpu 后端模型路径（uint8 输入 / float32 输出 / YOLOv8 风格） |
| `--kpu-conf` / `--kpu-iou` / `--kpu-class` / `--kpu-nc` | 0.35 / 0.45 / -1 / 0 | kpu 后端阈值与类别（见 8.2） |
| `--pm-perf` | 关 | 把 CPU/KPU 的 PM governor 设为 performance（板端 DVFS 降频时加；失败只记日志） |
| `--status <s>` | 2 | 状态日志周期 |
| `--trace <n>` | 5 | 前 n 帧逐帧 trace（定位卡在哪一步用） |

---

## 8. 识别后端

### 8.1 `color`（默认，无模型）

口径与旧工程/Python 原型完全一致（NV12→RGB565→CIELAB 阈值），但把「逐像素
float Lab（powf/cbrtf）」换成 **64KB 预计算查表**，颜色变换用 **RVV 向量化**
（16 像素/迭代）。宿主机标量路径实测全画面 640×480 逐像素扫描 ≈1.4ms；
默认 ROI 320×240 + step2 是百微秒量级。开机会跑一次 `rvv-vs-scalar` 自检，
逐位比对两条路径的掩码/统计/探针。

### 8.2 `kpu`（AI2D 零拷贝 + KPU 乒乓，需要 kmodel）

```text
vision 线程  submit(frame)  —— 只投递，帧所有权转交后端（不阻塞）
   ↓
pre 线程     hrt::create(物理地址, copy=false) 绑 VICAP 帧  →  ai2d_builder::invoke()
             （硬件 resize+pad letterbox；内部 poll 等 AI2D 中断）→ 归还 VICAP 帧
   ↓  双缓冲乒乓：这一步与下一行的 KPU 可以同时进行（两个独立硬件引擎）
kpu 线程     interpreter::run()（内部 poll 等 **KPU 完成中断**）
             → 读输出 tensor → YOLOv8 解码(postproc.c) → NMS → 结果环
   ↓
vision 线程  collect() 取结果 → 统计/FOUND-LOST 日志/发布
```

用法（SDK 自带示例模型即可先跑通链路）：

```bash
msh> /sdcard/app/green_led_ai_kpu --detector kpu --kmodel /sdcard/app/best.kmodel \
        --kpu-class 1 --vis-out 640x360 --status 2
```

`best.kmodel` 是 SDK 自带的 YOLOv8 三分类模型（class 0/1/2 = apple/banana/orange），
用来验证「AI2D 零拷贝 + KPU 中断等待 + 乒乓 + 解码/NMS」整条链路与耗时；
绿灯光模型训好后换 `--kmodel/--kpu-class/--kpu-nc` 即可。

| 参数 | 默认 | 说明 |
|---|---|---|
| `--detector kpu` | — | 切到 KPU 后端（需要 `green_led_ai_kpu` 这个产物） |
| `--kmodel <path>` | — | 模型路径；**要求输入 uint8、输出 float32、单输出 YOLOv8 风格** |
| `--kpu-conf` / `--kpu-iou` | 0.35 / 0.45 | 置信度与 NMS IoU 阈值 |
| `--kpu-class <n>` | -1 | 只认该类；-1=所有类别里取最高分 |
| `--kpu-nc <n>` | 0 | 类别数；0=按输出形状推断 |
| `--kpu-sync <0\|1>` | 0 | 是否对输入 tensor 做 `sync_write_back`（帧由 DMA 写入、CPU 不碰，默认不需要） |
| `--vis-out` | 640x480 | kpu 后端会自动把视觉通道切成 `RGB_888_PLANAR`（ISP 硬件做色彩空间转换） |

启动日志会明确打印模型信息与解析结果，例如：

```text
green_led_ai: kpu: 模型 dtype: input=1 output=9（期望 1=uint8 输入 / 9=float32 输出）
green_led_ai: kpu: 已启用：kmodel=/sdcard/app/best.kmodel 输入 640x360x3 -> 320x320x3, nc=3 n_box=2100 layout=CxN, conf=0.35 iou=0.45 class=1
```

每个状态周期会多打一行（这就是「单帧 <5ms」的判断依据）：

```text
green_led_ai: kpu ai2d=xxx/xxxus kpu=xxxx/xxxxus post=xx/xxus lat=xxxx/xxxxus fps=NN done=.. busy=.. drop=.. err(a2d=..,kpu=..)
```

- `ai2d + kpu` 是硬件两级耗时；`post` 是 CPU 解码+NMS；`lat` 是端到端延迟；
- 判定 200fps：**pipe 下 `kpu` 那一项 < 5000us**；`busy=`/`drop=` 表示投递时双缓冲都被占（调小模型或加缓冲）。

#### 关于模型与后处理的边界

- 已实现 **YOLOv8/v11 风格 anchor-free 单输出** 的解码（`postproc.c`，支持
  `C×N` 与 `N×C` 两种排布，带 letterbox 反算与按类别 NMS，**有宿主机单元测试**）。
- NanoDet-Plus（GFL + softmax 分布式回归）的解码不同：只需在 `postproc.c` 里
  加一个同级解码函数，NMS / 选目标 / 流水 / 缓冲全部复用。
- 量化（int8）输出需要 dequant，当前会明确拒绝并落日志，不做静默降级。

#### RVV 后处理的落点（先量后优）

已经用到 RVV 的是 **color 后端的颜色空间变换 + 阈值扫描**（16 像素/迭代，
开箱自检 RVV vs 标量一致）。KPU 后端的解码 / NMS 目前是标量实现（有宿主机
单元测试）—— 这是刻意的**先量后优**：YOLOv8 的候选只有 2100~8400 个、按
置信度筛完通常只剩个位数，`post=` 那一项大概率是几十微秒量级，而 KPU 本身是
毫秒量级。等板上 `kpu` 那一行打出来，如果 `post` 真的变成本瓶颈
（`post` 接近 `kpu`），优先 RVV 化这两处：①候选扫描（`C×N` 排布下按通道跨步
取数 + 类别最大值 + 阈值比较，天然适合 128bit 向量）；②IoU 批量计算。

#### 一个实测事实：SDK 的 `librvv` 并没有实现 NMS

`rvvlib/include/nms.h` 里声明了 `nms()` / `nms_e()`，但 `librvv.a` 的 32 个导出
符号全是数学核（`softmax_vec`、`expf_vec`、`sigmoid_vec`、`sgemm_vec`、
`maxindex_vec` …），**没有 `nms`**；官方 `usage_kpu` 示例也是自己写 NMS。
所以本工程用自研 NMS（可单元测试），并已把 `nms.h` 这条依赖去掉（否则链接
会报 `undefined reference to nms(std::vector<BoxInfo>&, float)`）。
`librvv` 的数学核在将来接 NanoDet-Plus（softmax）时可直接链接使用。

### 8.3 先量预算再定模型：`apps/kpu_bench`

## 9. 与旧工程文件对照

| 旧 | 新 | 说明 |
|---|---|---|
| `green_led_rtos/main.c`（1242 行，含 `#include "/绝对路径/sample_vicap_sensor.c"`） | `green_led_ai/main.c` + `vicap_src.c` + `vision.c` + `control.c` | 不再引用主机绝对路径，VICAP 逻辑自带 |
| `mpp_mem.c/h`（一把锁 + 逐帧 map/unmap） | `app.c: mpp_enter/mpp_map_persist` | 锁仍在，但映射改成「一次映射、永不逐帧解映射」 |
| `recorder.c`（与识别共用 CHN0 帧） | `record.c`（独占 CHN1） | 根因修法 |
| 识别（逐帧 mmap + float Lab） | `detect_color.c`（映射缓存 + 查表 + RVV） | 消除 CPU 瓶颈，保留阈值口径 |

旧应用 `apps/green_led_rtos/` 保留在仓库里作为对照，已从 `startup.list` 移除，
不再随开机启动。
