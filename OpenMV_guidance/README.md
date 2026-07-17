# OpenMV Guidance

OpenMV 侧绿灯识别、板载录像与经控制板导出工程。

职责：

- 采集图像并识别绿灯圆心
- 支持过曝时中心变白、边缘呈绿环的灯目标，仍在 OpenMV 侧解算圆心
- 通过 UART 向 STM32 发送 `x/y/area`
- 可选地在 SD 卡持续分段保存 MJPEG 录像

## 工程结构

- `src/main.py`：OpenMV 主入口
- `src/camera_config.py`：相机侧静态配置，包含 `SD_RECORD_FLAG`
- `src/green_light_detector.py`：绿灯识别与静态检测配置
- `src/video_recorder.py`：滚动录像管理
- `src/stream_debug.py`：USB-C 直连预览的独立帧源，只由主机调试工具临时执行
- `src/tuner_runtime.py`：主机 tuner 的 OpenMV 侧运行时支持
- `tools/openmv_cli.py`：本地校验、部署和 ST-Link 固件烧录入口
- `openmv.ini`：OpenMV 工程配置，包含源码目录、串口和 ST-Link 参数

## 依赖

- `python3`
- `pyserial`
- `openocd`，仅在使用 `ST-Link` 烧录固件时需要

安装：

```bash
python3 -m pip install -r OpenMV_guidance/requirements.txt
```

如果需要把模块预编译成 `.mpy`，还需要本机安装 `mpy-cross`，然后在构建或部署时加 `--mpy`。

## 命令

仓库根目录提供两段式脚本入口，日常操作优先用脚本，不直接记底层 Python 参数：

```bash
./.script/camera-ports
./.script/camera-view
./.script/camera-control
./.script/camera-photo
./.script/camera-record
./.script/camera-dump
./.script/openmv-check
./.script/openmv-build
./.script/openmv-flash
./.script/openmv-stlink-flash
```

相机脚本默认自动选择 OpenMV USB 口；如果同时插了多个串口设备，可用环境变量指定：

```bash
OPENMV_PORT=/dev/ttyACM0 ./.script/camera-view
```

## 工作流

### `ports`

列出当前系统检测到的串口设备，方便确认 OpenMV 或其他调试板真正占用了哪个 `tty`。

### `check`

- 对 `src/` 下的 Python 源文件做本地语法校验
- 不生成无意义的 C/C++ 构建产物

### `build`

- 默认行为：语法校验 + 打包到 `OpenMV_guidance/.openmv_build/fs`
- 可选 `--mpy`：把除入口脚本外的模块编译成 `.mpy`

示例：

```bash
./.script/openmv-build --mpy
```

### USB-C 相机预览

相机调试入口按功能拆成短脚本：

```bash
./.script/camera-ports     # 列出 USB 串口
./.script/camera-view      # 只预览，不驱动控制板
./.script/camera-control   # 预览，同时把目标测量从 OpenMV UART1 发给 STM32
./.script/camera-photo     # 抓一帧到 /tmp/omv_photo.jpg
./.script/camera-record    # 录制 MJPEG 和同名 metadata JSONL 到 record/
./.script/camera-dump      # 保存原始 OMVJ 帧流到 /tmp/openmv.bin
```

`camera-view`/`camera-control` 默认不再打断 OpenMV，也不进入 Raw REPL；主机只监听 `/flash/main.py` 在 USB VCP 上输出的 JPEG 帧，并在本机启动 MJPEG HTTP 预览页。`camera-control` 看到的是上电入口同一套识别和 UART1 控制逻辑。需要旧的临时调试入口时显式加 `--raw-repl`。

打开终端输出的 `http://127.0.0.1:8081/` 即可查看画面。相机脚本默认自动选择 OpenMV USB 口；如果自动选择失败，先运行 `camera-ports`，再用 `OPENMV_PORT` 指定：

```bash
OPENMV_PORT=/dev/ttyACM0 ./.script/camera-control
```

说明：

- USB-C 在主机上通常表现为 `/dev/ttyACM*`，只负责预览、调试和临时运行脚本。
- 默认预览模式下，OpenMV UART1 始终由 `/flash/main.py` 负责：启动前几帧发送 `x/y/area/image_width/image_height` 扩展帧，常态发送 `x/y/area` 短帧；主机脚本只读取 USB 预览流。
- 默认预览会在画面左上角叠加分辨率、帧率和目标像素坐标；高级参数仍可透传，例如 `./.script/camera-view --no-debug-detector`。
- `src/main.py` 是上场入口；USB 被主机打开时，它会在同一识别循环里额外输出 `OMVJ` 预览帧，不切换到另一套检测代码。
- `camera-record` 默认写入 `record/openmv_<timestamp>.mjpeg` 和同名 `.jsonl`；可用 `OPENMV_RECORD=/path/out.mjpeg` 覆盖。
- `camera-dump` 默认写入 `/tmp/openmv.bin`；可用 `OPENMV_RAW_DUMP=/path/openmv.bin` 覆盖，再用 `./.script/parse-frames /tmp/openmv.bin` 提取 JPEG。

### `flash`

- 先执行本地构建
- 再通过 OpenMV 的 USB serial raw REPL 上传构建产物到相机内部 `/flash`
- 每个上传文件都会从 `/flash` 回读校验大小和校验和，复位前执行 `os.sync()`
- 默认上传后触发一次硬复位，让板子重新执行 `/flash/main.py`

示例：

```bash
./.script/camera-ports
./.script/openmv-flash --port /dev/ttyACM1
./.script/openmv-flash --mpy --port /dev/ttyUSB0
```

### `stlink-flash`

- 通过 `OpenOCD + ST-Link` 烧录 OpenMV 固件镜像
- 这一步烧的是 MCU 固件，不是 Python 文件系统
- 烧完固件后，Python 业务代码仍然需要通过 `flash` 部署到内部 `/flash`

示例：

```bash
./.script/openmv-stlink-flash \
  --target-cfg target/stm32h7x.cfg \
  --image /path/to/openmv_firmware.bin \
  --address 0x08000000
```

如果你烧录的是 `.elf` 或 `.hex`，通常不需要额外传 `--address`；`.bin` 需要显式 flash 起始地址。

## 配置

默认配置位于 `openmv.ini`：

```ini
[project]
src_dir = src
entry_script = src/main.py
build_dir = .openmv_build
remote_manifest = _guided_dart_manifest.txt

[serial]
port =
baudrate = 115200

[stlink]
openocd = openocd
interface_cfg = interface/stlink.cfg
target_cfg =
transport = hla_swd
firmware_image =
firmware_address =
```

建议：

- `serial.port` 只在确认 OpenMV 的真实串口后再填写
- `stlink.target_cfg` 和 `stlink.firmware_address` 需要按你的 OpenMV 板型和固件格式填写

## 说明

- 这个子工程是纯 Python/OpenMV 工程，不再依赖 PlatformIO 或占位 `.c` 文件。
- 本地“构建”定义为源码校验与部署包生成；可选地用 `mpy-cross` 产出 `.mpy` 模块。
- 上传器会维护一个内部 `/flash` manifest，并在每次部署前清理旧的 `.py/.mpy` 同名文件，避免残留文件影响导入；同时自动刷新 SD 根目录的最小启动跳板。
- 对 OpenMV 来说，`ST-Link` 只用于烧固件镜像；Python 应用通过 USB Raw REPL 部署到内部 `/flash`。
- 这版固件复位后当前目录仍是 `/sdcard`，因此 SD 根目录保留一个最小 `main.py` 跳板：只切换到 `/flash` 并执行 `/flash/main.py`；业务代码仍只部署到内部 `/flash`。
- `SD_RECORD_FLAG` 在 `src/camera_config.py` 中静态配置，默认关闭以避免 SD 写入影响识别和 UART1 控制输出；设为 `1` 后录像只保存到 `/sdcard/recordings` 或 `/sd/recordings`，未检测到 SD 卡时禁用录像，不回写板载 flash；UART1 只负责向 STM32 发送目标测量，不参与图像存储。
- 主机侧可通过仓库脚本 `./.script/record-list` 与 `./.script/record-pull` 在“只连接控制板串口”的前提下列出并下载录像。
