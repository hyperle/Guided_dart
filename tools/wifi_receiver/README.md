# Rust Wi-Fi 视频接收端

配合原生 RT-Smart `dart_app` 使用：板端复用识别/录像的 Gray8 帧，异步压缩 JPEG
并经 Wi-Fi 上传；本工具接收后用 FFmpeg 编码一次，输出浏览器 H.264/HLS 和保留的
fragmented MP4。主机运行不需要 Python；Python 只用于独立集成测试。

## 启动主机

需要 Rust/Cargo（支持 edition 2024）、`ffmpeg`（含 `libx264`）；`ffprobe` 用于验收。
以下命令从仓库根目录运行。主机与板子需要互通的 IPv4 网络，例如同一个 2.4 GHz
热点或路由器，且电脑监听端口可达。

```bash
cargo build --release --locked --manifest-path tools/wifi_receiver/Cargo.toml
# 自行设置一段随机 token；同一值写入板端 wifi.conf。
export DART_WIFI_TOKEN='REPLACE_WITH_YOUR_RANDOM_TOKEN'
./tools/wifi_receiver/target/release/dart-wifi-receiver \
    --bind 0.0.0.0 --port 8080 --width 320 --height 240 --fps 15
```

默认仅绑定 `127.0.0.1`；板端直接上传时显式用 `--bind 0.0.0.0`。主机浏览器打开
`http://127.0.0.1:8080/`；同网络手机可打开 `http://电脑IPv4:8080/`。
播放器及 hls.js 复用 `tools/wifi/` 的本地资源，不依赖外部 CDN。
监听端口可用 `--port 0` 自动分配，实际端口打印在启动日志中。

`--output DIR` 默认 `tools/wifi_receiver/output`，每次启动在其下创建独立轮次目录。
终端打印实际目录，HLS 使用 `index.m3u8` 和滚动 TS 分片；`recording.mp4` 随会话持续
增长，停止后保留，可从 `/live/recording.mp4` 获取。HLS 仅保留近期分片，MP4 保留
本轮完整编码输出，需按实际使用周期管理电脑磁盘空间。

Ctrl-C 或 SIGTERM 停止 HTTP/WebSocket 输入，关闭 FFmpeg stdin，等待它结束视频输出。
FFmpeg 退出或输入管道阻塞超时会停止服务并返回非零退出码，终端保留编码器错误。

## 启动原生板端

复制 [wifi.conf.example](../wifi/wifi.conf.example) 到板端 `/data/app/wifi.conf`，
填入 SSID、密码、电脑的实际 IPv4、端口和相同 token；文件内容不加引号，键不留空格。
将本地副本保存为 `tools/wifi/wifi.conf` 时 Git 会忽略它。

```ini
ssid=YOUR_2_4_GHZ_SSID
password=YOUR_PASSWORD
host=192.168.1.100
port=8080
token=REPLACE_WITH_YOUR_RANDOM_TOKEN
width=320
height=240
fps=15
quality=50
connect_timeout_ms=2000
io_timeout_ms=2000
```

空 `ssid=` 表示只复用预先建立的关联；空 `password=` 表示开放热点。
SSID 最长 32 UTF-8 字节，密码为 8..63 字节或 64 位十六进制 PSK。
输出宽高必须为偶数、不大于采集尺寸，并与主机参数一致；FPS 范围 1..120、质量 1..100，
TCP 连接/完整上传时限范围 1..30000 ms。未知键、重复键或非法值会拒绝配置。
不要将真实热点密码或上传 token 提交到仓库。

构建和部署沿用仓库脚本，配置文件使用现有板端文件传输方式单独复制：

```bash
bash .script/build-rtos dart_app
bash .script/deploy-rtos dart_app
```

板端从交互 shell 执行（确保 launcher 没有同时运行另一个 dart_app/CanMV 相机进程）：

```text
/sdcard/app/dart_app --wifi on --wifi-config /data/app/wifi.conf --seconds 30
```

可独立组合 `--vision` / `--record` / `--wifi`，例如录像关闭后仍回传图像。
默认回传关闭。自动启动需要在板端 `/sdcard/app/startup_final.list` 的现有 dart_app 条目追加 Wi-Fi 参数，
不能另开第二个相机进程。配置或网络故障只影响 Wi-Fi 功能。

运行时关闭/恢复整个回传分支，不重启 dart_app：

```bash
bash .script/stream-rtos off
# 观察板端 module=wifi 周期日志：state=paused active=0 buffers=0 后已完成回收。
bash .script/stream-rtos on
```

开关默认位于 `/sdcard/app/control/stream.pause`；任何现存对象或无法访问的标记都
禁止回传，删除标记恢复。板端可用 `--wifi-pause-file PATH` 指定位置；自定义
`BOARD_DIR` 时让该参数与主机脚本的 `${BOARD_DIR}/control/stream.pause` 对应。
暂停标记跨重启保留，`--wifi off` 启动时不会被 on 命令开启。

控制文件检查、配置加载、缓冲分配、编码/发送线程创建和销毁都归回传管理线程，
采集侧只尝试提交，关闭时直接返回，不等待设备清理。暂停后编码/发送线程与
像素缓冲已释放，只保留低频的开关管理线程；恢复时重新加载配置。启动失败
停在 `state=failed`，修复配置后 off/on 重试，不进行连续的分配重启。
它不修改本地录像、日志写入开关、相机资源或全局默认网络路由/自动重连策略。
已连接其它 SSID 时返回 EBUSY，不强制切换；只有没有关联且配置了 SSID 时才尝试新建连接。
关闭发送会释放自己的 socket，保留共享网卡的关联，不执行网卡断电或全局断网。

`state=stopping` 期间已停止接收新帧。同步驱动 ioctl 的结束时限由固件决定，
因此回收完成时间必须看状态确认；等待发生在管理线程，不能强杀线程并释放仍在使用的缓冲。
软件开关隔离的是正常错误返回、网络拥塞和组件启停等待；同一进程中的内存破坏或
内核驱动故障仍需单独定位，不能由此宣称系统级故障完全隔离。

原生板端目前使用字面 IPv4 + HTTP/1.1，不包含公网 DNS、HTTPS/WSS 客户端或手机
局域网中继服务器；电脑与板子分处不互通网络时不能直接填写公网 HTTPS 隧道域名。
电脑播放器可放在现有反向代理/隧道后访问。Rust 服务也接受原有 Python 客户端和手机
网页中继的 WebSocket 协议，可用于对照迁移前后的行为；它不替代手机到原生板端的中继服务。

## 接收与编码语义

- `POST /api/frame`：`Authorization: Bearer TOKEN`、`Content-Type: image/jpeg` 和有界
  JPEG 请求体。成功返回 204，保持连接；原生发送端逐帧确认接收后发送下一张。
- `GET /api/frame/ws`：握手带相同 Bearer，二进制消息是一张完整 JPEG；确认是该连接
  累计接收数的 4 字节大端二进制消息，与 `wifi_test.py` 一致。
- `GET /api/relay/ws`：握手后 5 秒内先发 `{"token":"..."}`，收到 `{"ready":true}` 后
  再发二进制 JPEG。支持旧手机中继发送的短状态消息。
- `GET /api/status`：接收数、编码数、尺寸、FPS、在线状态、最近帧年龄和 HLS 就绪状态。
  上传不携带 token 到播放 URL。

每张 JPEG 上限 2 MiB；检查完整 SOI/EOI、受支持的 SOF、尺寸、精度和分量数后交给
FFmpeg 解码。尺寸不匹配返回 422，错误类型返回 415，未授权返回 401。
HTTP 处理有 10 s 时限，WebSocket 有大小/空闲/发送超时。

接收端仅保留最新图像，没有无限增长的帧队列。FFmpeg 按 `--fps` 定时消费，网络抖动
时重复最近 2 s 内的图像；超过 2 s 停止补帧，新图到达后恢复，跳过漏掉的时钟节拍。
录像时间轴因此表示实际输出节拍，长断网期间不填满黑帧，也不保存逐帧采集时间戳。
编码帧数可以大于或小于接收数，204/WebSocket 确认代表接收成功，不保证这一帧已落盘。

## 验证

```bash
cmake --build build --parallel
ctest --test-dir build --output-on-failure
cargo fmt --check --manifest-path tools/wifi_receiver/Cargo.toml
cargo clippy --locked --manifest-path tools/wifi_receiver/Cargo.toml --all-targets -- -D warnings
cargo test --locked --manifest-path tools/wifi_receiver/Cargo.toml
cargo build --locked --manifest-path tools/wifi_receiver/Cargo.toml
python3 tests/test_wifi_native_link.py
```

集成测试使用相同原生 C JPEG/HTTP 代码，覆盖分段响应、拒绝/超大/重复长度/截断/超时，
然后启动真实 Rust 服务与 FFmpeg，上传 60 张图，验证直连及手机中继 WebSocket 累计确认、
网页、本地播放器资源、HLS 分片、MP4 的 ffprobe 信息与完整解码，以及 FFmpeg 失败传播。
使用本机回环网络，不连接实板；完整性能与 Wi-Fi 掉线恢复仍待实板验收。

`streaming_control` 另外用独立传输接口模拟取消后迟迟不返回的驱动，验证此时帧生产和
真实本地录像仍持续工作；覆盖运行时关闭/恢复、像素缓冲回收、启动暂停、配置错误和
失败后显式重试。`wifi_radio` 验证复用现有关联、SSID 冲突保护、取消和禁止共享网络策略修改。
