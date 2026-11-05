# 原生 JPEG / Wi-Fi 回传

`dart_app` 在识别、舵机观测之后、录像移交租约之前，将同一张 Gray8 图送到
`Session::try_submit`。默认 `--wifi off`；用 `--wifi on --wifi-config /data/app/wifi.conf`
开启。应用使用 `start_controlled`，回传管理线程负责配置加载和工作线程生命周期；
采集线程不等待配置文件 I/O、设备初始化或回收。配置失败只关闭回传。

## 边界与所有权

- `interface.cpp`：定长配置、严格文件解析、`ITransport` 和注入的 `TransportFactory`。
- `jpeg.c/.h`：C11 CPU 基线单分量 JPEG，缩放、边缘补齐、FDCT、量化和熵编码；
  `dart_status_t` + 输出参数，配置时一次性生成量化/Huffman/余弦表，无逐帧分配。
- `session.cpp`：C++23 实现，固定缓冲、编码/发送两个 pthread 和独立管理线程；
  线程创建使用返回码，不依赖异常。所有工作线程生命周期归管理线程，底层不依赖业务目录。
  同步 `start/stop` 仅供串行拥有者和整体退出使用，采集热路径只调用非阻塞 `try_submit`。
- `http.c/.h`：非阻塞 IPv4 TCP + HTTP/1.1 上传，处理短写、EINTR/EAGAIN、分段响应、
  Content-Length、有界响应体和取消。一个请求在途，以完整 2xx 响应确认接收。
- `k230/radio.c`：唯一调用 `hal_netmgmt` 的边界，优先复用已有 STA 关联；只有尚未
  关联且给定 SSID 时选择 SDIO 并请求关联/DHCP。已有 SSID 不匹配返回 EBUSY，不能
  断开别的使用者；不清全局 IP，不调用默认路由和自动重连策略的修改接口。
  新建关联的底层路由行为仍由固件网络策略管理。
- `k230/transport.cpp`：将平台结果翻译为 `std::expected<void, Error>`；只在发送线程使用。
  `board/k230.cpp` 创建具体实现，入口将工厂传给启动编排；L4 不包含平台头文件。

输入 `FrameView` 只在 `try_submit` 返回前借用。它先验证尺寸/stride/数据长度，按墙钟
限制输入 FPS，`try_lock` 失败直接丢帧；成功后逐行复制到两个组件启动期分配的 CPU 缓冲之一。
生命周期锁和队列锁都只 try_lock，工作线程正在初始化或回收时立即拒绝提交；
不会等待 join、文件操作、消费者、压缩或网络操作。复制结束后，原相机租约立即回到原录像/归还流程。

原始缓冲状态为 `free/pending -> encoding -> free`。压缩线程从 pending 取最新帧，
按配置尺寸最近邻缩小并编码到另两个预分配的 JPEG 缓冲。JPEG 状态为
`free/pending -> sending -> free`。当 pending 已有旧帧时覆盖它，并分别累计
`raw_replaced` / `jpeg_replaced`；工作线程正在使用的缓冲绝不被覆盖。
两组缓冲、状态和条件变量归 Session 管理；像素计算和网络等待在锁外执行。
缓冲只供 CPU 使用，不是 DMA/VB 内存，无额外 cache 同步，工作线程不持相机/VB 租约。

每次组件启用后的像素缓冲总量为 `2 * source_width * source_height +
2 * (4 * output_width * output_height + 4096)` 字节，另加固定表和线程栈。
640×480 输入、320×240 输出时约 1.18 MiB。输出空间不足会丢弃该 JPEG 并累计
`jpeg_errors`，不截断上传。JPEG 是未经检测标注的原始灰度图，板端不进行 H.264 编码。

## 时间、故障与停止

序号和 `received_us` 从原帧复制，后者来自单调时钟；未经校验的 sensor PTS 不当作时间。
HTTP 请求带 `X-Frame-Sequence` 与 `X-Timestamp-Us`。当前主机视频按自身 FPS 定时消费
最新 JPEG，保留的是接收时间轴，不是无损采集时间轴。

网络失败关闭连接、丢弃当前 JPEG，发送线程按 250 ms 到 8 s 指数退避；新帧仍可替换
pending。重新连接后的帧如果超过 `io_timeout_ms` 的年龄预算则丢弃。计数
`uploaded` 只代表主机确认接收，不能解释为已经完成视频编码或落盘。主机日志与
`/api/status` 另报编码次数；固定输出帧率会复用近期图像，因此两者不要求相等。

TCP connect、每次完整 HTTP 上传/回复分别有单调截止时间；每次 poll 最多 100 ms，
停止信号会取消等待，退避也可唤醒。SDIO 的同步 ioctl 由固件控制时限，用户态只能
在 ioctl 返回后取消；DHCP 轮询最多 20 s。

## 独立关闭与恢复

```bash
bash .script/stream-rtos off
bash .script/stream-rtos on
```

独立控制标记默认 `/sdcard/app/control/stream.pause`，可由 `--wifi-pause-file PATH`
修改；主机脚本随 BOARD_DIR 定位控制目录。任何现存对象或访问错误都禁止回传，
只有 ENOENT 才允许开启。它不依赖、不修改 `WriteControl`、录像/日志开关或其他设备。
管理线程每 50 ms 检查标记，统计与错误使用原子快照供调用层读取。

- `starting`：正在加载配置、分配自身缓冲或创建工作线程；此时不接受采集帧。
- `running`：允许非阻塞复制；Wi-Fi 是否连接由独立的 connected 字段表示。
- `stopping`：已经关闭输入，编码/发送线程收到取消，但旧设备调用可能仍在退出。
- `paused`：编码/发送线程已 join，套接字和像素缓冲均回收；仅保留管理线程。
- `failed`：配置、分配或线程启动失败，工作资源已回收。失败锁存，off/on 后显式重试。
- `stopped`：整个 Session 已结束，管理线程也已退出。`--wifi off` 不启动任何回传线程。

关闭顺序是停止接收新帧 → 取消编码/发送 → join 工作线程 → 关闭自己的 socket →
释放像素缓冲。生命周期锁保护帧复制与回收的互斥，但采集只尝试加锁，不等待这个过程。
关闭期间即使 SDK 调用暂未返回，也只有管理线程等待，主采集流程照常决定原帧的去向。
`state=paused active=0 buffers=0` 是完成回收的确认；主机脚本仅提交请求，不伪造板端确认。

共享网卡的关联不随组件暂停而断开，也不关闭全局 VB/VICAP；避免影响其他使用者。
恢复时在管理线程重新加载配置并创建新的工作资源，保留本次 Session 的累计计数。
读取配置失败同样不影响主线程，也不会在每个采集周期重试文件或设备初始化。

这是线程与资源生命周期隔离，不是独立地址空间的故障隔离；同一进程中的内存破坏、
内核故障或共享硬件资源争用不能据此保证完全无影响。实际开销仍需实板验收。
未来接入 H.264 时编码器只能释放自己持有的通道和缓冲，不得重新初始化/退出全局
VB，也不得停止共享相机；编码器退出等待仍必须留在组件管理线程。

配置、内存和线程启动错误锁存在组件状态，由调用层周期报告；叶子函数不重复打印。
运行期错误在周期状态行累计报告：
`state/active/buffers/accepted/encoded/uploaded/bytes/downsampled/raw_replaced/jpeg_replaced/
stale_dropped/busy/invalid/jpeg_errors/network_errors/native/lifecycle_code/lifecycle_native`。SSID、密码和 token 不写入日志。

## 构建与验证

目标统一在 `include/CMakeLists.txt` 声明；`DART_WIFI_K230` 默认随
`DART_CAMERA_K230` 开启。原生 SDK 必须具备 RTL8189 SDIO1、`/dev/netmgmt`、
相应 socket syscall 以及 `netmgmt_wlan_select_device` 等接口。
构建用户态 ELF 不会替换固件或修改板级 SDIO 引脚/供电。

```bash
cmake --build build --parallel
ctest --test-dir build --output-on-failure
bash .script/build-rtos dart_app
```

主机测试需要 libjpeg 开发库。`streaming_jpeg` 用独立 libjpeg 解码验证 1..100 质量、奇数尺寸、stride、缩放和容量保护；
`streaming_session` 验证配置、拥塞替换、失败重连、停止取消与重新启动；
`streaming_control` 验证运行时停用/恢复、异步回收、故障锁存及并行本地录像；
`wifi_radio` 用真实 SDK 头文件验证不修改默认路由、重连策略或其他使用者的关联。
完整主机集成测试见 [Rust 接收端说明](../../../../tools/wifi_receiver/README.md)。
实板带宽、JPEG 耗时、识别帧率影响、驱动恢复与 SDIO 关联仍需在对应固件上验收。
