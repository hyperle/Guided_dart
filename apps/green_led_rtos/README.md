# green_led_rtos（识别 + 默认常录）

K230 RT-Smart 业务应用：VICAP 唯一持有者。

```text
sensor(gc2093) 1920x1080@30 -> VICAP OFFLINE -> CHN0 缩放到 640x480 NV12
   |- 识别线程: CIELAB 阈值 -> blob 筛选 -> 绿灯重心（LAB 阈值与 Python 原型同一套）
   |- 录像模块(recorder.c): H.264 落盘 + 逐帧 CSV，**不阻塞识别**
```

不开 USB 预览：运行情况用板载日志 `/sdcard/app/logs/green_led_rtos.log` 与
录像 `/sdcard/app/recording/` 自查（见下面"用板载日志 / 录像自查"）。

## 默认行为：开机即录、自动滚动、总量上限 8GB

- **常录模式**：进程启动后自动开始录像，无需任何外部触发；之后每段会话
  录到默认 **256MB**（`recorder.c` 的 `REC_ROLL_BYTES`）自动收尾并滚动到
  下一段，如此持续到进程退出。
- **总容量上限**：`/sdcard/app/recording/` 目录默认上限 **8GB**
  （`recorder.h` 的 `REC_DEFAULT_CAP_BYTES`），每段收尾时自动删除最旧会话，
  始终保持总量 ≤ 上限。
- **手动 stop 可暂停**：`kill -USR2 <pid>` 或 socket `stop` 停止当前段并退出
  常录（暂停）；再次 `kill -USR1 <pid>` 或 socket `start` 恢复常录。
- 每段之间收尾/新建有极短间隔（毫秒级），会丢少量帧；`status` 的统计可查。

## 与识别解耦的录像模块

识别线程与录像线程互不等待：

- 采集线程每帧只做一次**有界**操作：把 NV12 帧 memcpy 进录像自有的 VB
  环形缓冲（满则丢帧并计数），随后立即继续采集；
- H.264 硬件编码 + 落盘由录像模块自己的 feed/drain 线程完成，编码器/SD
  变慢只导致录像丢帧，绝不影响识别帧率。

启动项默认在 `/sdcard/app`（与日志目录 `/sdcard/app/logs` 平级），因此
录像目录为 **`/sdcard/app/recording`**，每个会话生成一对文件：

```text
/sdcard/app/recording/
├── rec_0001.h264      # Annex-B H.264 裸流（含 SPS/PPS，可直接封装）
└── rec_0001.csv       # 逐帧识别结果
```

CSV 列（`frame_seq` 与录像帧一一对应，`result_seq` 是该帧入队时刻已完成的
最近一次识别，通常为 frame_seq-1，用于离线逐帧对齐）：

```text
frame_seq,result_seq,mono_us,center_x,center_y,detect_fps
```

## 手动启停 / 状态（供调试与外部控制）

进程 pid 写入 `/sdcard/app/green_led_rtos.pid`，供外部定位。

1. **同进程其它模块**：调用 `rec_start()` / `rec_stop()`（见 recorder.h）。
2. **信号**：`kill -USR1 <pid>` 恢复常录并立即开录、`kill -USR2 <pid>`
   停止并暂停（msh 串口终端或任意板端进程均可发）。
3. **控制 socket**：`/sdcard/app/green_led_rtos.ctl`，命令：

   ```text
   start                     # 恢复常录并立即开录
   stop                      # 停止当前段并暂停常录
   status                    # active/auto/roll_bytes/dir/cap/帧统计
   cap <bytes>               # 修改总容量上限（本次运行生效）
   dir <path>                # 修改录像目录（本次运行生效，下次会话起）
   ```

   板端可用 `socat`/`nc` 或任意小程序连 socket 发送命令。

录像模块日志：`/sdcard/app/logs/green_led_recorder.log`（会话起止、滚动、
帧统计、丢帧、evict）。

## 主机查看

```bash
# H.264 裸流封装成 mp4（帧率必须和录制时的实际帧率一致，现在默认 30）
ffmpeg -f h264 -framerate 30 -i rec_0001.h264 -c copy rec_0001.mp4

# 对照 csv 逐帧看识别结果（frame_seq 对齐）
```

文件通过 MTP/gphoto2 从板端拷回主机即可（`/sdcard/app/recording/`）。

## 用板载日志 / 录像自查（本版本的验证方式）

不看 USB 预览，直接读这三份产物：

| 产物 | 用途 |
|---|---|
| `/sdcard/app/logs/green_led_rtos.log` | 管线配置、sensor 实际模式、识别状态、每秒统计 |
| `/sdcard/app/logs/green_led_recorder.log` | 每段录像的开始/结束与帧统计 |
| `/sdcard/app/recording/rec_XXXX.h264` + `.csv` | 识别看到的画面 + 逐帧结果 |

开机后**先看这几行**（缺哪一行就说明卡在哪一步）：

```text
green_led_rtos: sensor actual 1920x1080@30 type=31          # 探测命中；出现 WARN 说明没命中
green_led_rtos: vicap dev=0 mode=offline acq=1920x1080 out=640x480 ae=0 awb=1 dnr3=1 buffers=6+6
green_led_rtos: exposure fixed 200 us (range ... ) rc=0     # 固定曝光生效
green_led_rtos: frame 640x480 stride=640 fmt=.. phys1-phys0=307200 stride*h=307200   # 平面布局自检
green_led_rtos: running (detector owns VICAP)
```

运行时每 `--status` 秒一行状态，**这是判断识别是否正常的主要依据**：

```text
green_led_rtos: status det_fps=29 center=(312,241) px=214 blobs=1 green_px=241 scan=roi miss=0 found=120/3 | dumped=360 drop=1 dump_fail=0 | rec active=1 sess=1 in=358 sent=358 drop=0 written=356 bytes=...
green_led_rtos: probe rgb=(18,96,36) lab=(34,-35,25) at (312,244) thr L>=12 A<=-20 B>=8 step=(2,2) roi=(160,120,320,240)
```

怎么读：

- `det_fps` 接近 30 且 `dump_fail=0` → 采集/识别都跟得上。
- `center=(-1,-1)` / `green_px=0` → 阈值内没有绿色像素：看 `probe lab=(...)`。
  `probe` 是窗口内最绿的像素，把它的 LAB 和阈值比：
  A 不够负（如 -8）说明灯没在阈值内 → `--lab` 放宽 A 上限（如 -12）；
  B 太小（如 2）→ 放宽 B 下限（如 0）；整幅都暗（L<12）→ `--expo-us` 调大。
- `px` 与 `blobs`：`green_px` 很大但 `blobs=0` → 形状筛选太严（灯不是圆斑/被裁），
  放宽 `--pix-min` 或把 `--step` 调到 1,1。
- `detect FOUND/LOST` 行给出状态切换时刻，和 csv 里 `center_x/center_y` 变号对得上。
- `dumped` 持续增长而 `drop` 增长很快 → 识别跟不上（看 det_fps 与耗时）；
  `dump_fail` 增长 → 采集侧有问题（先看上面 sensor/vicap 两行）。
- `rec written` 应该 ≈ `sent`；差很多说明 SD 写不过来，调 `--bitrate`。

## 参数（都可写在 startup.list 那一行里，改完重启即生效）

| 参数 | 默认 | 说明 |
|---|---|---|
| `--csi <0-2>` | 2 | CSI 号；庐山派摄像头在 CSI2 |
| `--fps <n>` | 30 | 探测请求帧率，**必须和 1920x1080 一起用**（见 main.c 头注释） |
| `--out <WxH>` | 640x480 | 识别/录像尺寸（CHN0 缩放输出） |
| `--roi <x,y,w,h>` | 160,120,320,240 | 识别 ROI；`0,0,0,0` = 全画面（慢但不会漏） |
| `--step <x,y>` | 2,2 | 采样步长；`1,1` = 逐像素（最准，`px` 也是真实像素数） |
| `--lab <Lmin,Amax,Bmin>` | 12,-20,8 | CIELAB 阈值，和 Python 原型 `TH_GREEN` 同一套数值 |
| `--pix-min <n>` | 50 | blob 最小像素数（按图像像素估算） |
| `--miss <n>` | 3 | 连续丢失 n 帧后整幅重扫；0=关闭 |
| `--expo-us <us>` | 200 | 固定曝光；0=不设（保持驱动默认）。`--ae` 打开自动曝光 |
| `--keep-gain` | 关 | 默认把增益压到最低（防 AGC 反扑） |
| `--no-record` | 关 | 本次不开录像 |
| `--bitrate <kbps>` | 12000 | H.264 码率 |
| `--rec-dir <path>` | /sdcard/app/recording | 录像目录 |
| `--status <s>` | 2 | 状态日志周期（秒） |

## 与上一版的差异（为什么改）

1. **探测请求 640x480@120 → 1920x1080@30**。`kd_mpi_sensor_adapt_get()` 是
   "精确(宽高fps) → 同宽高 → 找更大分辨率"三级匹配且候选按 fps 降序；gc2093 在
   CSI2 只有 1920x1080@30/60 和 1280x960@90 / 1280x720@90，请求 640x480@120 会
   **静默**落到 1280x960@90 —— 采集侧实际跑的分辨率/帧率和代码常量全都对不上。
   1920x1080@30 才是精确命中。
2. **ONLINE → OFFLINE，缩放交给 CHN0**（官方 sample_uvc_dev_vicap 与 CanMV 运行时
   的做法；官方 sample_vicap_sensor 的 ONLINE + CHN0 不缩放只适合全尺寸直出）。
3. **ISP：AE 关（配固定曝光 200us + 最低增益）、AWB 开、DNR3 开**，曝光/增益在
   `start_stream` 之后用 sensor ioctl 写死，和 Python 原型 `apply_exposure()` 一致。
4. **识别换成 CIELAB 阈值 + blob 筛选**（最小像素数 / 填充率 / 长宽比 / 取最大 blob
   的包围盒中心），数值与 Python 原型 `green_led_test_k230.py` 的 `TH_GREEN`、
   `PIX_MIN`、`FILL_*`、`ASPECT_*` 一致；LAB 用标准的 sRGB→CIELAB(D65)，并先做
   RGB565 量化，与 CanMV `find_blobs` 用的 `lab_table` 口径相同（20k 随机色对照，
   最大偏差 4/255）。
5. **采集线程不再阻塞**：信箱满就把刚 dump 的帧直接 release 并计数，避免识别慢
   一拍时攥着 VICAP buffer 把 6 个 buffer 耗光；录像在投递前先喂，所以识别丢帧
   不影响录像连续性。
6. **录像帧率改为 30**（原来是 120）：H.264 的 `src/dst_frame_rate` 与 GOP 都用它，
   帧率写错会让封装出来的视频播放速度不对。
7. **recorder 支持 UV 平面独立物理地址**（`phys_uv != phys_y + stride*height` 时单独
   映射），避免非连续布局把录像录花。

## 说明与限制

- 帧率上限取决于传感器/编码器实际能力：探测请求 30fps，识别实测帧率看 `status det_fps`；
  若 H.264 编码跟不上（丢帧），`status` 与 recorder 日志的 dropped/written 会体现。
- 录像 VB 内存约 5MB（6×NV12 输入块 + 8×输出块，见 recorder.c 宏），
  第一个会话启动时才申请；申请失败只影响录像（自动重试），识别照常。
- 每段 256MB 与总上限 8GB 都可改：单段滚动量在 `recorder.c` 的
  `REC_ROLL_BYTES`，总上限在 `recorder.h` 的 `REC_DEFAULT_CAP_BYTES`
  （或运行时 `cap` 命令）。
- 早期版本曾把处理后画面经 USB CDC ACM 回传主机实时预览，方案存在恶性
  bug 已放弃并删除；`usb_cam_stream` 作为"USB 回传原始画面"的独立方案保留在
  `apps/usb_cam_stream/`，与 green_led_rtos 互斥（都要独占 VICAP），默认不上板。
