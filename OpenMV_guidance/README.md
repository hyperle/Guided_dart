# OpenMV Guidance

OpenMV 侧绿灯识别、板载录像与经控制板导出工程。

职责：

- 采集图像并识别绿灯圆心
- 通过 UART 向 STM32 发送 `x/y/area`
- 接收 STM32 转发来的参数热更新命令并立即生效
- 在 OpenMV 本地文件系统持续分段保存 MJPEG 录像

## 工程结构

- `src/main.py`：OpenMV 主入口
- `src/green_light_detector.py`：绿灯识别与参数逻辑
- `src/video_recorder.py`：滚动录像管理
- `src/tuner_runtime.py`：主机 tuner 的 OpenMV 侧运行时支持
- `tools/openmv_cli.py`：本地校验、部署和 ST-Link 固件烧录入口
- `openmv.ini`：OpenMV 工程配置，包含源码目录、串口、SD 卡和 ST-Link 参数

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

仓库根目录提供了这些脚本入口：

```bash
./.script/openmv-ports
./.script/openmv-check
./.script/openmv-build
./.script/openmv-flash
./.script/openmv-sd-sync
./.script/openmv-stlink-flash
```

也可以直接调用：

```bash
python3 OpenMV_guidance/tools/openmv_cli.py ports
python3 OpenMV_guidance/tools/openmv_cli.py check
python3 OpenMV_guidance/tools/openmv_cli.py build
python3 OpenMV_guidance/tools/openmv_cli.py flash
python3 OpenMV_guidance/tools/openmv_cli.py sd-sync
python3 OpenMV_guidance/tools/openmv_cli.py stlink-flash
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

### `flash`

- 先执行本地构建
- 再通过 OpenMV 的 USB serial raw REPL 上传构建产物到板载文件系统
- 默认上传后触发一次硬复位，让板子重新执行 `main.py`

示例：

```bash
./.script/openmv-ports
./.script/openmv-flash --port /dev/ttyACM1
./.script/openmv-flash --mpy --port /dev/ttyUSB0
```

### `sd-sync`

- 先执行本地构建
- 再把 Python 应用同步到一个已经挂载到主机上的 OpenMV 启动文件系统目录
- 适合没有可用 REPL 串口，但可以通过 `SD` 卡启动 `main.py` 的场景

示例：

```bash
./.script/openmv-sd-sync --dest /media/$USER/OPENMV
./.script/openmv-sd-sync --mpy --dest /media/$USER/OPENMV_SD
```

### `stlink-flash`

- 通过 `OpenOCD + ST-Link` 烧录 OpenMV 固件镜像
- 这一步烧的是 MCU 固件，不是 Python 文件系统
- 烧完固件后，Python 业务代码仍然需要通过 `flash` 或 `sd-sync` 部署

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

[sdcard]
root =

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
- `sdcard.root` 填宿主机上已经挂载的 OpenMV 启动文件系统目录
- `stlink.target_cfg` 和 `stlink.firmware_address` 需要按你的 OpenMV 板型和固件格式填写

## 说明

- 这个子工程是纯 Python/OpenMV 工程，不再依赖 PlatformIO 或占位 `.c` 文件。
- 本地“构建”定义为源码校验与部署包生成；可选地用 `mpy-cross` 产出 `.mpy` 模块。
- 上传器会维护一个板端或目标目录 manifest，并在每次部署前清理旧的 `.py/.mpy` 同名文件，避免残留文件影响导入。
- 对 OpenMV 来说，`ST-Link` 适合烧固件镜像；Python 应用更适合走 REPL、USB 启动文件系统或 `SD` 卡启动文件系统。
- 录像优先保存到 `/sdcard/recordings`，未插卡时回退到 `/flash/recordings`。
- 主机侧可通过仓库脚本 `./.script/record-list` 与 `./.script/record-pull` 在“只连接控制板串口”的前提下列出并下载录像。
