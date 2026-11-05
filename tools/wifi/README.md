# CanMV Wi-Fi 相机回传与诊断

原生 RT-Smart 的 `dart_app` 已接入 JPEG 压缩/发送线程，配套的
[Rust 主机接收端](../wifi_receiver/README.md) 负责 HLS/MP4 编码输出。
板端配置示例见 [wifi.conf.example](wifi.conf.example)。本页以下保留 MicroPython
诊断、手机中继与历史固件验证说明，便于迁移对照。

打开 [wifi_test.py](wifi_test.py)，在 CanMV VS Code 插件中连接板子并运行当前文件。
脚本独立运行，无需交叉编译。上电开关、Wi-Fi 机械开关由用户手动打开；建议在启动前
打开 Wi-Fi 开关，让固件开机时能枚举到模块。

已知模块为 RTL8189、接在 SDIO1。使用支持该板 RTL8189 的 MicroPython 固件，
测试路由器或手机热点设为 **2.4 GHz**。Python 的 `WLAN_SDIO` 只选择 SDIO 类型的网卡，
SDIO0/1 控制器、引脚和模块供电时序由固件决定。


## 相机图片上传、后端编码、手机网页播放

当前方案：**开发板 → 一帧一帧 JPEG → 电脑后端 FFmpeg → H.264/HLS 视频 → 手机网页**。
板子只做相机采集、JPEG 压缩和 WebSocket 连续发送，视频编码在电脑端。电脑保持自己的网络；
板子可以继续连接手机热点。两者网络不通时，用公网隧道为后端提供可访问的 HTTPS 地址。

### 当前默认：手机网页中继

`STREAM_MODE="phone_relay"` 时，板子不自行建立公网 TLS 连接：
**板子 JPEG → 手机热点内 WebSocket → 手机浏览器 WSS → 电脑 FFmpeg → H.264/HLS 播放**。
这用于对照排查此前板端直传公网时反复停传的问题；是否改善需要手机实测。

将 [phone_relay.py](phone_relay.py)、[phone_relay.html](phone_relay.html) 和
[wifi_test.py](wifi_test.py) 一起保存到板子 `/sdcard/`，运行 `wifi_test.py`。
电脑后端和公网隧道照下文启动。串口打印 `[wifi] relay READY` 后，在提供热点的手机上
打开该行的 `http://板子IP:8081/`，例如 `http://172.17.196.136:8081/`。
页面会自动连接相机并转发，内嵌播放器显示电脑编码的视频。**需要保持该网页在前台**；
锁屏或关闭网页会停止中继。其它设备仍可直接打开公网地址观看后端视频。

局域网网页包含上传 token，只应在受密码保护的热点内使用；公网播放网页不包含该 token。
浏览器通过 `/api/relay/ws` 建立连接后，先发送文本 JSON `{"token":"..."}`，后台鉴权成功
返回 `{"ready":true}`，才接受后续二进制 JPEG。鉴权前最多读取 512 字节、等待 5 秒。
手机只保留有界的待发数据，公网拥塞时跳过新帧；板端窗口为 4 张，避免积压旧画面。
手机回传局域网接收数和后端确认数，板端 `uploaded_frames` 只统计后端已确认的图片。

`STREAM_MODE="direct"` 保留板端直传模式，使用下面的窗口、TLS 和恢复配置。
手机中继的前提是手机允许访问热点客户端 IP；若该地址打不开，需要先检查这一局域网条件。

### 1. 在电脑运行后端

依赖 Python 3.9+ 和带 `libx264` 的 FFmpeg，不需要 pip 安装 Python 包：

```bash
ffmpeg -version
python3 tools/wifi/video_server.py --port 8080 --fps 15
```

电脑浏览器打开 `http://127.0.0.1:8080/`。刚启动显示“等待相机”，后端终端会打印
`UPLOAD_TOKEN`，下一步将其填到板端脚本。该 token 只用于上传鉴权；网页播放地址不带 token。
可用环境变量 `CANMV_UPLOAD_TOKEN` 或 `--token` 固定 token，便于后端重启后复用。
默认仅监听本机；同局域网直接访问时可加 `--bind 0.0.0.0`。

### 2. 用公网隧道连接两个网络

另开终端，在已安装 `cloudflared` 的电脑上运行：

```bash
cloudflared tunnel --url http://127.0.0.1:8080
```

它会显示一个临时地址，例如 `https://YOUR-TUNNEL.trycloudflare.com`。
保持后端和隧道两个进程运行：手机浏览器打开这个地址，板子上传地址则是在同一地址后
追加 `/api/frame`。无需电脑连接手机热点，也无需手机热点开放入站端口。
本次已将官方 `cloudflared` 下载到 `/tmp/dart-cloudflared`，按发布资产 SHA-256 验证后启动。
当前主播放地址为 `https://seek-med-delayed-enhancement.trycloudflare.com`，公网 `/api/status`
读取通过。当前脚本已配置此地址及后端上传 token。隧道重建后地址可能变化，需同步更新脚本；
`/tmp` 工具不会作为系统安装保留。不要把含热点密码和上传 token 的本机脚本直接提交或公开。

### 3. 配置并运行 wifi_test.py

在 [wifi_test.py](wifi_test.py) 保留正确的热点 SSID/密码，并填写：

```python
STREAM_ENABLED = True
STREAM_MODE = "phone_relay"
PHONE_RELAY_PORT = 8081
UPLOAD_URL = "https://YOUR-TUNNEL.trycloudflare.com/api/frame"
UPLOAD_TOKEN = "后端终端打印的token"
STREAM_WIDTH = 320
STREAM_HEIGHT = 240
STREAM_FPS = 15
UPLOAD_TRANSPORT = "websocket"
STREAM_MAX_IN_FLIGHT = 16
JPEG_QUALITY = 50
```

在 CanMV IDE 运行脚本。手机中继模式打开串口打印的局域网页面；直传模式连网后自动上传。
手机网页在首批视频分片就绪后播放，
也可点“播放 / 重连”。手机和电脑不需要同一局域网，但都需要能访问公网隧道。
`UPLOAD_URL` 留空或 `STREAM_ENABLED=False` 会跳过上传，保留原来的 Wi-Fi 测试。
原有 `TCP_TEST_HOST` 是局域网握手诊断；使用公网隧道时通常留空，不用填写公网域名。

默认 320×240、目标 15 FPS、JPEG 质量 50。更改 `STREAM_FPS` 时同步更改后端 `--fps`；
后端 `--width` / `--height` 控制视频输出尺寸。
默认 `UPLOAD_TRANSPORT="websocket"`，脚本自动在 `UPLOAD_URL` 路径后追加 `/ws`，
仍以一张完整 JPEG 为一条二进制消息。后端异步返回累计收帧数，板子不逐张等待公网往返。
`STREAM_MAX_IN_FLIGHT=16` 限制尚未确认的图片数；达到上限时先等确认，再采集新图。
累计确认超过 `STREAM_IO_TIMEOUT_MS` 无进展则重连；未确认帧不计入成功帧数。
发送按 1 KiB 分块，开启 TCP_NODELAY；相机内只持有当前 JPEG，不存放 16 张图片。
窗口越大，越能掩盖公网确认延迟，也可能增加网络中尚未确认图片的年龄；可按实际网络调小。
`UPLOAD_FALLBACK_URLS` 可填写同一后端的其它隧道入口（共用同一 token）。连接失败会换路线；
15 秒平均收帧率低于目标的四分之一也会尝试其它入口。404/410 的过期路线会被停用；
401/403 仍明确报错，不会用重试掩盖错误 token。
`UPLOAD_IPV4_HINTS` 只对字典中匹配的域名生效，避免本轮手机热点 DNS 失败；TLS 的 SNI 和 HTTP Host
仍是原域名。未指定地址时缓存本次运行中成功的 DNS 结果，连接全部失败后清除缓存重新解析。
更换域名时可清空该字典，恢复正常 DNS；IP 提示需要随服务端地址变化更新。
兼容模式 `UPLOAD_TRANSPORT="http"` 仍用每张图一个 POST/204，公网往返较慢时可能只有约 0.5–1 FPS。
连接/DNS/后端暂时错误会持续退避重试，间隔从 1 秒增加到最多 15 秒；
IDE 停止会关闭相机/socket。`STREAM_SECONDS` 可设置有限运行秒数，默认持续运行。

直传 TLS 握手在非阻塞 socket 上进行，由首个写入驱动并受超时控制，避免阻塞握手无限等待。
HTTPS 会发送 SNI。`TLS_CA_FILE` 留空时沿用 MicroPython 的默认 TLS 行为，不验证服务端
证书；需要证书验证时，将受信任的 PEM CA 文件上传到板子，并填入该文件路径，脚本会启用
`CERT_REQUIRED`。固件必须支持 TLS；HTTP 仅用于本地可达的后端或专用网络测试。

### 相机接口和实板限制

当前 Dart V1 设置 `CAMERA_BACKEND="raw"`，使用 `_dart_raw` 的 640×480 灰度图。
本轮普通 `Sensor` 虽能探测到传感器，初始化未正常返回；因此不将它作为本板自动回退路径。
其它已验证支持普通 Sensor 的板可显式选择 `sensor`，或用 `auto` 优先 RAW、缺失时回退；
其 VGA 探测参数为 `Sensor(width=640, height=480, fps=0)`。
`RAW_EXPOSURE_LINES=400` 仅对 RAW 后端生效；普通 Sensor 使用固件的曝光设置，
两条路径的亮度响应不同。可以设 `CAMERA_BACKEND="raw"` 要求 RAW 接口，缺失时明确报错。
不需要在脚本中启用 Display 或 IDE 图像预览。相机与其它占用 VICAP/VB 的应用互斥。

2026-10-31 后续检查发现 SDK 的非阻塞 TCP `recv` 将 EAGAIN 转成零字节读，
会向 TLS 错误地表示 EOF；阻塞模式遇到真正 EOF 还会继续循环。
[network-socket-fix.patch](network-socket-fix.patch) 修正短读、暂不可读和 EOF 语义，
并让非阻塞 connect 识别 EISCONN、在等待中让出 CPU。
构建只补丁化私有 `source/port/network`，原 SDK 保持不变：

```bash
python3 tools/canmv_tuning/build.py --build-dir build/canmv-wifi-video-socket --wifi-socket-fix
```

该解释器 SHA-256 为 `48ee2c0ae81a967ac6e95604dccb18c8630d2c62785aa872ffcf13b937a33fcd`，
已上传、回读校验、激活并用 `machine.reset()` 重启。上一解释器保存在
`build/canmv-wifi-video-socket/board-backups/20261031-222016-706499/micropython`，
板端还保留 `/sdcard/micropython.before-dart-raw-20261031-222016-706499`。
板内 TCP 回环已验证无数据抛出 would-block、8 字节请求立即读回 2 字节、关闭后立即返回 EOF。
原生 C 回归还用真实 POSIX socket 验证同样三种语义。这些检查通过不代表公网链路已稳定。


2026-10-31 用户重开热点/重上电后，首次连接用时 3143 ms，取得 `172.17.196.136`，
并通过 5 秒链路检查；不能仅凭该恢复确认前次失败根因。
本轮以当前 SDK 独立构建 `build/canmv-wifi-video/micropython`，包含 WLAN、TLS 和 RAW 接口；
SHA-256 为 `d7361de32b193daa462ac64045b9e313ee44f3f0ff39817548c9d7dcef326222`。
板端旧解释器已备份到 `build/canmv-wifi-video/board-backups/20261031-174452-135169/micropython`，
并保留 `/sdcard/micropython.before-dart-raw-20261031-174452-135169`；新解释器已完整回读校验、激活和重新上电。
`_dart_raw.PIXEL_FORMAT == "gray8"` 验证通过，三张实拍 JPEG 均为 320×240，大小 2238、2283、2282 字节。
手机网页播放仍需实际手机验证；本机测试成功不能代替该验收。

本轮跨网实测：板子经手机热点对公网隧道进行 HTTPS 逐帧上传，15 秒测试后端收到 16 张
真实 JPEG，合计 36,461 字节，板端报告 `stream=PASS`。公网 `/api/status` 返回
`received_frames=16`、`hls_ready=true`；下载 HLS 片段后，ffprobe 确认为 H.264、
320×240、5 FPS，并成功解码图片。上传实际约 1 FPS，后端按 5 FPS 重复最近帧形成连续视频。
JPEG 原图和解码视频均偏暗，尚未标定曝光/照明；图片上传和视频编码通过不代表画面质量已达标。

新解释器重上电后曾再次遇到连接拒绝，后来直接扫描对象连接被接受，约 16.4 秒后取得 IP；
失败根因仍未确认。随后尝试持续回传时，板端再次丢失 IP；手机自身移动数据正常且播放网站
可达，但板端曾出现公网 TCP 超时。重新打开热点后，三次连接仍被拒绝，因此早期测试仅确认短时 16 帧回传成功。之后增加了持续恢复机制与 WebSocket 连续发送；
最新实测见下方持续回传记录。
固件输出 `Network (rt-smart) cannot be disabled`，所以
`active(False)` 不构成一次射频复位。后续默认复用已验证的同热点 IPv4，减少重新关联。

### 后端行为与排查

| 地址 / 输出 | 用途 |
| --- | --- |
| `/` | 手机和电脑的播放网页 |
| `GET /api/frame/ws` | WebSocket 升级，带相同 Bearer token；每条二进制消息是一张 JPEG，后端用 4 字节大端累计帧数确认 |
| `GET /api/relay/ws` | 手机浏览器 WebSocket；首条文本消息鉴权，后续逐帧 JPEG 和累计确认同上 |
| `POST /api/frame` | `image/jpeg` 上传，带 `Authorization: Bearer <token>`；成功返回 204 |
| `/api/status` | 接收帧数、编码输入帧数、最近收帧时间、编码状态 |
| `/live/index.m3u8` | 后端 H.264/HLS 播放列表 |
| `/snapshot.jpg` | 最近收到的 JPEG，方便检查相机原图 |
| 后端终端打印的 `ffmpeg.log` | 编码错误诊断 |

`[wifi] stream PASS uploaded_frames=...` 表示后端确认收到了 JPEG；
`/api/status` 的 `hls_ready=true` 表示已生成视频播放列表；最终还需手机实际播放验证。
后端保留最新一帧，按固定视频帧率重采样；网络慢时重复上一帧，不累计旧图队列。
超过 5 秒没有新图，网页显示“相机离线”，最后一帧仍保留。
HLS 按 1 秒分片并保留滚动窗口，播放通常有数秒延迟；不是录像归档，退出后临时分片删除。

iPhone Safari 使用原生 HLS；其它支持 MSE 的浏览器按需从 CDN 加载固定版本的 hls.js。
这些浏览器第一次打开页面需要能访问 CDN，无法访问时网页会显示播放器加载失败。
上传 token 不会出现在公网播放网页或板端日志中；手机中继的局域网页面包含该 token。
公网地址的持有者可以观看画面。

```bash
python3 -m unittest discover -s tests -p 'test_wifi*.py' -v
```

73 项测试覆盖原有 Wi-Fi 行为、上传短写/背压/超时/重试/资源清理，以及真实 HTTP/WebSocket 长连接
接收、手机浏览器鉴权与局域网转发的完整确认链路、延迟确认时连续发送、未确认窗口限制、确认丢失超时、服务端主动断开后的重连、地址缓存/显式地址提示、备用路线切换与过期路线跳过，及上传鉴权和异常输入。集成测试用测试图片调用实际的板端上传函数，再由本机 FFmpeg
生成 H.264/HLS，用 ffprobe 确认格式并解码检查像素；它替换了相机采集，不使用真实射频链路。


## 持续回传与恢复

已配置 SSID、`STREAM_ENABLED=True` 和 `UPLOAD_URL` 时，脚本直接进入持续回传模式；
它复用已验证的同热点 IPv4，不先执行全网扫描或断开已有连接。
扫描和连接由恢复流程管理；SSID 留空、上传地址留空或关闭推流时，仍运行原有诊断模式。

- 掉线或 IP 变化：关闭旧 socket，重新关联、获取 IP，按新地址绑定上传连接。
- TCP、TLS、DNS 或后端临时错误：丢弃本次未确认图片，按 1、2、4、8、15 秒退避；
  不累计待发图片，恢复后重新抓取最新帧。
- 连续三次网络失败：Dart SDIO 驱动可通过一次受密码保护的临时 AP 模式切换，
  在后续 STA 连接中调用 `wifi_off()` / `wifi_on()` 重新初始化模块；退出时关闭临时 AP。
  这与没有实际关闭网卡的 `active(False)` 不同。此路径已经在本板验证，
  可用 `STREAM_RADIO_RECOVERY=False` 关闭。普通后端繁忙（HTTP 503 等）不触发无线复位。
- 错误上传地址/token、相机采集错误仍明确失败；不会作为网络抖动无限掩盖。
- `STREAM_SECONDS=0` 持续运行，有限秒数用于长测；结束时若仍在恢复，返回 `INCONCLUSIVE`。

本板 RAW 相机启动后会执行一次无线恢复初始化策略，由 `STREAM_RADIO_RECOVERY` 控制。
反复传输卡顿独立累计；零星收到一张图片不再清除卡顿计数，三个卡顿会触发无线重建。
当最近一段接收速度恢复到目标的一半以上时，清除卡顿计数。

日志和返回值增加 `stream_stats`：上传错误数、Wi-Fi 恢复尝试数、丢弃图片数、
启动耗时、测试时长以及相邻确认帧的最大间隔；另有 `sent_frames` 和 `unconfirmed_frames`，
分别记录本地发送完成数与断线时无法确认的帧数。后端 `/api/status` 增加
`uptime_seconds`、`last_minute_frames`、`max_upload_gap_seconds` 和最近 10 秒的 `upload_fps`。
网页“新图 FPS”使用后端实际收帧速率。`stream_id` 变化时播放器自动重连，HLS 分片使用时间戳编号，
减少后端重启后手机播放器继续等待旧分片的问题。
**验收看 `received_frames` 是否持续增加以及收帧间隔，不能用重复最后一帧生成的
`encoded_frames` 数量证明相机持续在线。**

2026-10-31 实板恢复测试：原连接状态连续拒绝关联，临时 AP 切换后 2750 ms 内接受 STA 连接，
随后取得 `172.17.196.136`。新版持续程序遇到三次 DNS 失败后，也自动完成无线重新初始化、
DHCP 和真实 JPEG 回传。具体连续收帧时长以本节后续实测记录为准。

2026-10-31 优化实测：早期每张 HTTP POST 等待一次 204，手机报告约 0.5 FPS。
改用 WebSocket、1 KiB TLS 写入后，板端一次 303.053 秒测试确认了 2072 张真实 JPEG，
平均约 6.84 FPS；多段达到 10–15 FPS，后段仍有低谷和路线恢复，结束时为 `INCONCLUSIVE`。
这条记录不作为“稳定 15 FPS”验收。扩大到 64 的后续部署中，独立五分钟观察只有 93 张新图（0.31 FPS），已退回 16；不将此版本称为稳定回传。
暂停相机后重复发送同一张图的诊断结果未计入真实相机验收。

本轮 Cloudflare 临时 QUIC 隧道发生控制连接中断，板端拿到 HTTP 530 / `error code: 1033`。
主入口恢复使用原 HTTP/2 隧道；另测试了 SSH TCP 443 的 Pinggy 临时入口作为备用：

```bash
ssh -p 443 -R 0:127.0.0.1:8080 a.pinggy.io
```

命令会打印新的临时 HTTPS 地址，填入 `UPLOAD_FALLBACK_URLS`。免账户 Pinggy 本轮提示
**60 分钟到期**，不能视为长期固定地址；到期后返回 404/410 的路线会被跳过；本轮到期的临时入口已从默认备用列表中移除。
后端和原 Cloudflare 进程需保持运行。板端脚本已保存到 `/sdcard/wifi_test.py` 并回读核验，
`STREAM_SECONDS=0` 持续运行，可由 IDE 运行或停止。

保持 CanMV IDE 断开，或让 IDE 独占串口运行脚本；不要让主机串口测试工具与 IDE 同时写串口。
手机播放只需要公网地址，不占用板端 USB。

## 原有 Wi-Fi 诊断使用

1. 将文件顶部设为 `SSID = ""`，直接运行：检测 WLAN、MAC、扫描附近热点，按信号强度输出。
2. 填入 `SSID`、`PASSWORD` 后重跑：扫描、连接、等待有效 IPv4，再观察 5 秒链路。
   `UPLOAD_URL` 留空时只做诊断；填写上传配置后继续回传相机。
   开放热点的密码留空；脚本不会打印密码。编辑过的密码会保存在本机脚本中，提交前清空。
3. 默认保留成功的连接，方便后续 socket 程序复用；`DISCONNECT_ON_EXIT = True` 可在测试后断开。
   `REUSE_CONNECTION=True` 时，确认已有连接的 SSID 相同且 IPv4 有效后复用，日志明确显示
   `REUSE`，不重新检验密码；其它/未知 SSID 的旧连接先断开，再尝试本次连接。
   需要测试新关联或更新密码时设 `REUSE_CONNECTION=False`；扫描模式保留已有连接。

串口输出示例：

```text
[wifi] device PASS WLAN active
[wifi] scan PASS count=3
[wifi] connect PASS
[wifi] ip PASS ip=192.168.1.42 netmask=255.255.255.0 gateway=192.168.1.1 dns=192.168.1.1
[wifi] summary PASS scan_count=3 ip=192.168.1.42 tcp=SKIP
```

`summary PASS` 只表示本次启用的检查通过。扫描模式不证明连接成功；取得 IP 也不证明
能访问电脑或互联网。没有扫描到热点时显示 `INCONCLUSIVE`，因为部分 CanMV 固件将底层
扫描错误转换为空列表。设备检测失败时检查 Wi-Fi 开关和固件驱动；连接失败时检查热点、
密码、信号和 DHCP。`CONNECT_TIMEOUT_MS` 限制脚本等待 IP 的时间，底层扫描/连接调用
仍使用固件自身的超时；可从 IDE 停止脚本。


## `run connect failed`：扫描成功但连接被拒绝

`device PASS` 和扫描到目标热点只能证明网卡/扫描正常。若紧接着显示
`run connect failed.`、`driver rejected connection`，说明固件的连接接口返回了失败；
还没有进入脚本的 IPv4 等待、相机采集或后端上传，调整上传地址不能修复这一步。
现有固件只向 Python 返回 `False`，没有提供具体认证失败原因，因此不能单凭这条日志
断言密码错误、WPA3 不兼容或 DHCP 故障。

脚本增加 `CONNECT_ATTEMPTS=3`，仅在驱动明确返回 `False` 时重试；
每次记录 `attempt` 和 `elapsed_ms`。重试前显式断开并等待
`CONNECT_RETRY_DELAY_MS=2000`，避免未完成的旧连接与新连接重叠。
驱动异常、IDE 停止、断开失败均直接退出；驱动接受连接后仍需取得有效 IPv4，才报告成功。
`CONNECT_TIMEOUT_MS` 只限制等待 IP/断开的时间，不能中断固件自身的阻塞连接调用。

2026-10-31 用户确认热点密码与脚本一致。实板另测扫描对象连接 `connect(info=..., key=...)`
也被拒绝，耗时 6299 ms、`link=False`、`ip=0.0.0.0`；先断开、等待 2 秒后按 SSID
重连同样失败，耗时 9266 ms。扫描上报 WPA2/AES（`4194308`，即 `0x400004`）。
没有据此确认根因或宣称重试已修复实板连接。

建议下一步：

1. 关闭后重新打开手机热点，检查已连接设备上限、允许设备名单和屏蔽名单；
   板子 MAC 为 `e0:3d:9f:2c:89:17`。
2. 板子整机断电约 10 秒，再上电；上电前保持 Wi-Fi 机械开关打开。
3. 运行更新后的脚本，观察三次尝试的结果。仍失败时，用另一个已知可用的
   2.4 GHz WPA2 热点做对照，以区分热点侧和板端/固件侧问题。


## 可选 TCP 验证

在同一局域网电脑上运行：

```bash
python3 -m http.server 8000 --bind 0.0.0.0
```

填写 `TCP_TEST_HOST` 为电脑的局域网 IPv4，`TCP_TEST_PORT = 8000`。脚本使用板端 WLAN
地址绑定 socket，测试 TCP 握手并关闭连接；连接被拒绝或超时会报告失败。电脑需允许
局域网访问该端口，此检查不访问外网。

## 本机检查与板端状态

```bash
python3 -m unittest discover -s tests -p test_wifi_test.py -v
```

测试使用假的 `network`/时钟/socket，覆盖 CanMV 扫描对象与 MicroPython 扫描元组、
旧接口、设备缺失、空扫描、连接失败、无 IP、时钟回绕、链路丢失、停止和 socket 清理。
原有 21 项本机模拟测试保留。首次固件的 WLAN API 缺失已修复；新固件已上板，
2026-10-31 三轮扫描均找到 `hyperlee`，信道 6、RSSI −76～−78 dBm，耗时约 1.46 s。
用户随后连接 `hyperlee` 成功，取得 IP `172.17.196.136` 并通过 5 秒链路检查。
TCP 未配置，未验证电脑或互联网可达性。

API 依据：已读取 CanMV MCP 缓存中的 `network.pyi`、`socket.pyi`、`time.pyi`、
`libs/Network.py` 和 `examples/28-APP_CENTER/demo_wifi.py`；兼容旧版单网卡构造接口。


## 当前固件缺少 WLAN API 的修复

这次报错发生在构造 WLAN 之前。源码 `src/canmv/port/mpconfigport.h` 仅在
`CONFIG_ENABLE_NETWORK_RT_WLAN` 开启时向 `network` 注册 `WLAN`。
当前 SDK 板级默认值将其关闭，内核默认配置也未启用 `RT_USING_WIFI`。
仅修改 Python、SSID 或拨动开关无法添加未编译的接口。

[rtl8189-sdio1.patch](rtl8189-sdio1.patch) 对本机 SDK 的两层 defconfig 做以下调整：

| 层 | 配置 | 值 |
| --- | --- | --- |
| SDK / MicroPython | `CONFIG_ENABLE_NETWORK_RT_WLAN` | `y` |
| RT-Smart | `CONFIG_RT_USING_WIFI` | `y` |
| RT-Smart | `CONFIG_RT_USING_REALTEK` | `y` |
| RT-Smart | `CONFIG_REALTEK_SDIO_RTL8189FTV` | `y` |
| RT-Smart | `CONFIG_BSP_WIFI_SDIO_HOST_1` | `y` |
| RT-Smart | `CONFIG_BSP_WIFI_SDIO_REG_ON_PIN` | `-1`（不操作使能 GPIO） |

SDIO1 引脚沿用板级 DTS：IO26 CLK、IO27 CMD、IO28–31 D0–D3。
开关由用户手动打开。eMMC 继续使用独立的 SDIO0。

2026-10-31 已将补丁应用至 `/mnt/mydata/kRTOSSDK`，离线容器完整编译返回 0。
修改前的配置、旧镜像和本轮完整构建日志在
`/tmp/dart-wifi-firmware-backup.XVNGzC/`。
重新在未打补丁的同版 SDK 复现可运行：

```bash
patch --dry-run -d /mnt/mydata/kRTOSSDK -p1 < tools/wifi/rtl8189-sdio1.patch
patch -d /mnt/mydata/kRTOSSDK -p1 < tools/wifi/rtl8189-sdio1.patch
cd /mnt/mydata/kRTOSSDK
make k230d_canmv_dart_v1_defconfig
make NCPUS=8
```

固件更新必须同时包含 **RT-Smart 内核和 MicroPython 可执行文件**。
单独复制 Python 脚本、单独替换解释器或只刷 `_ota.kdimg` 均不完整：
当前 OTA 镜像中的 RTApp 槽是占位内容，不含应用 FAT 分区的 `/sdcard/micropython`。
完整 `.img` 包含两部分；烧录会覆盖其范围内的旧系统与应用文件，需先备份板端文件。

升级后在 REPL 检查，再运行原脚本：

```python
import network
print(hasattr(network, "WLAN"))  # 应为 True
```

完整镜像编译与内容校验通过，用户已烧录；板端 WLAN 激活、MAC 读取和热点扫描通过。
连接、DHCP 与 5 秒链路检查也已通过；TCP 未配置，未验证。


## 空扫描诊断与新固件实测

`INCONCLUSIVE count=0` 表示当前调用没有报告热点；此时 WLAN 已注册不等于 RF 收发已验收。
脚本默认 `SCAN_ATTEMPTS = 3`，空结果间隔 `SCAN_RETRY_DELAY_MS = 2000` 毫秒重试，
发现热点立即停止重试，并为每次扫描打印 `attempt`、`elapsed_ms` 和 `count`。
原有驱动扫描异常会直接报告失败；重试后仍为空则保留 `INCONCLUSIVE`。

2026-10-31 新固件实测：MAC 为 `e0:3d:9f:2c:89:17`，网卡 `wlan0` / `wlan0ap`。
在附近开启 2.4 GHz 热点 `hyperlee` 后，三次扫描均得到 1 个热点；扫描时间
1456、1457、1456 ms，信道 6，RSSI −76、−78、−76 dBm。
首次空扫描未复现，现有证据不能把它归因于初始化时序或 RF 故障。

只做扫描时 SSID 保持空。验证连接时填写 `SSID = "hyperlee"`、对应密码，再运行同一脚本；
空 SSID 的 `connect SKIP` 是按配置跳过，扫描热点本身不需要密码。


## 连网实测通过

用户配置 `hyperlee` 后实际输出：

```text
[wifi] connect PASS
[wifi] ip PASS ip=172.17.196.136 netmask=255.255.255.0 gateway=172.17.196.247 dns=172.17.196.247
[wifi] rssi INFO -82
[wifi] monitor PASS 5/5 seconds
[wifi] tcp SKIP TCP_TEST_HOST is empty; no LAN/Internet reachability claim
[wifi] summary PASS scan_count=1 ip=172.17.196.136 tcp=SKIP
```

此结果表示本次扫描、连接、IPv4 与 5 秒链路观察成功。`WAIT link=False elapsed_ms=5`
是开始连接后的瞬态等待；`tcp SKIP` 是未配置可选测试；脚本完成后的 CanMV 版本和 REPL
提示符是正常结束输出。RSSI −82 dBm 表示信号偏弱，长时间或高速传输可将热点靠近板子后再测。
没有据此宣称互联网、TCP 或长时间连接稳定性已验收。
