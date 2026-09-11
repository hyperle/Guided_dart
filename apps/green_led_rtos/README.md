# green_led_rtos（识别 + 默认常录）

K230 RT-Smart 业务应用：VICAP 唯一持有者，640x480 采集 → 绿灯重心识别
（120fps 目标）。录像功能为独立解耦模块 `recorder.c`，**不阻塞识别**。

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
# H.264 裸流封装成 mp4（带时间基；如需逐帧步进分析可不解码直接 -c copy）
ffmpeg -f h264 -framerate 120 -i rec_0001.h264 -c copy rec_0001.mp4

# 对照 csv 逐帧看识别结果（frame_seq 对齐）
```

文件通过 MTP/gphoto2 从板端拷回主机即可（`/sdcard/app/recording/`）。

## 说明与限制

- 帧率上限取决于传感器/编码器实际能力：识别按 DETECT_FPS=120 运行；
  若 H.264 编码跟不上（丢帧），`status` 的 dropped/written 会体现。
- 录像 VB 内存约 5MB（6×NV12 输入块 + 8×输出块，见 recorder.c 宏），
  第一个会话启动时才申请；申请失败只影响录像（自动重试），识别照常。
- 每段 256MB 与总上限 8GB 都可改：单段滚动量在 `recorder.c` 的
  `REC_ROLL_BYTES`，总上限在 `recorder.h` 的 `REC_DEFAULT_CAP_BYTES`
  （或运行时 `cap` 命令）。
- 早期版本曾把处理后画面经 USB CDC ACM 回传主机实时预览，方案存在恶性
  bug 已放弃并删除（usb_cam_stream、green_led_ipc.h、usb_cam_viewer.py）。
