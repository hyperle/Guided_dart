# STM32 Dart Guidance

## 项目说明

这是一个基于 STM32G431 微控制器的飞镖引导系统项目。该项目旨在为 RoboMaster 比赛中的飞镖发射系统提供精确的姿态控制和引导功能。通过集成 IMU（惯性测量单元）、PID 控制器和伺服电机驱动，实现飞镖的稳定发射和瞄准。

项目使用 STM32CubeMX 生成基础代码，并采用 CMake 进行构建管理，支持多种编译器工具链。

## 目录

- [STM32 Dart Guidance](#stm32-dart-guidance)
  - [项目说明](#项目说明)
  - [目录](#目录)
  - [安装](#安装)
    - [环境要求](#环境要求)
    - [构建步骤](#构建步骤)
  - [使用情况](#使用情况)
  - [功能](#功能)
  - [配置](#配置)
  - [许可证](#许可证)

## 安装

### 环境要求

- STM32CubeIDE 或其他支持 STM32 的 IDE
- CMake 3.16 或更高版本
- GCC ARM None EABI 工具链
- STM32CubeG4 HAL 库

### 构建步骤

1. 克隆项目到本地：
   ```bash
   git clone <repository-url>
   cd STM32
   ```

2. 配置 CMake：
   ```bash
   mkdir build
   cd build
   cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake
   ```

3. 构建项目：
   ```bash
   make
   ```

4. 烧录到 STM32 设备：
   使用 STM32CubeProgrammer 或其他烧录工具将生成的二进制文件烧录到 STM32G431 微控制器。

## 使用情况

1. 连接硬件：
   - 将 IMU 传感器（BMI088）连接到 SPI 接口
   - 连接伺服电机到 PWM 输出引脚
   - 确保电源和通信接口正确连接

2. 编译并烧录代码到 STM32 板子

3. 上电运行：
   系统将自动初始化传感器、启动控制循环，并根据配置参数进行姿态控制

4. 监控输出：
   通过 UART 接口查看调试信息和传感器数据

## 功能

- **IMU 数据采集**：集成 BMI088 传感器，支持加速度计和陀螺仪数据读取
- **姿态解算**：使用 Mahony 滤波算法进行姿态估计
- **PID 控制**：实现多轴 PID 控制器用于姿态稳定
- **伺服电机驱动**：控制伺服电机进行瞄准和发射
- **通信接口**：支持 UART 和 SPI 通信

## 配置

项目配置主要通过以下文件进行：

- `STM32.ioc`：STM32CubeMX 项目配置文件，定义引脚分配和外设配置
- `CMakeLists.txt`：CMake 构建配置文件
- `Core/Inc/` 目录下的头文件：包含宏定义和配置参数

主要配置参数：

- IMU 采样率和滤波参数（在 `imu.h` 中定义）
- PID 控制参数（在 `pid.h` 中定义）
- 伺服电机参数（在 `servo.h` 中定义）
- UART 通信波特率（在 `usart.h` 中定义）

### 主机热调参

主机侧可通过 [Host_tools/guidance/guidance_params.yaml](/home/hyperlee/guided_dart/Host_tools/guidance/guidance_params.yaml:1) 管理当前在线参数，配合 [Host_tools/guidance/guidance_param_sync.py](/home/hyperlee/guided_dart/Host_tools/guidance/guidance_param_sync.py:1) 将变更通过串口发送给 `ESP32`，再由 `ESP32` 转发到无线控制板。

参数协议已抽到顶层共享定义 [protocol/guidance_protocol.yaml](/home/hyperlee/guided_dart/protocol/guidance_protocol.yaml:1)。主机脚本直接读取这份 schema，STM32 侧则在构建前自动生成 [Drv/Inc/guidance_protocol_generated.h](/home/hyperlee/guided_dart/Dart_guidance/Drv/Inc/guidance_protocol_generated.h:1)，因此保持“单一真源”，同时不影响热传参链路。

参数文件采用受限 YAML 子集，当前支持：

- `green_light.setpoint_x / setpoint_y`
- 绿灯检测器阈值、面积、填充率、跟踪窗口、滤波增益、丢帧容忍
- `controller.control_mode`
- `controller.pid_kp / pid_ki / pid_kd / pid_integral_limit / pid_output_limit_us / pid_invert_output`
- `controller.zero_pulse_us_0..3`

一次性推送：

```bash
python3 ../Host_tools/guidance/guidance_param_sync.py --config ../Host_tools/guidance/guidance_params.yaml
```

也可以通过仓库内的一键启动脚本直接运行：

```bash
./.script/param-push
```

监听文件并在保存后自动热更新：

```bash
python3 ../Host_tools/guidance/guidance_param_sync.py --config ../Host_tools/guidance/guidance_params.yaml --watch
```

也可以通过仓库内的一键启动脚本直接运行：

```bash
./.script/param-sync
```

关闭监听可使用：

```bash
./.script/param-sync-kill
```

### 终端追踪图形查看

新增了一个独立的终端查看器 [Host_tools/guidance/guidance_terminal_viewer.py](/home/hyperlee/guided_dart/Host_tools/guidance/guidance_terminal_viewer.py:1)，用于实时查看 `setpoint` 与绿灯测量值的对位情况。

运行方式：

```bash
python3 ../Host_tools/guidance/guidance_terminal_viewer.py --config ../Host_tools/guidance/guidance_params.yaml
```

仓库内也提供了对应的一键启动脚本：

```bash
./.script/track-viewer
```

它会复用与热传参脚本相同的 `serial / ble` 连接配置，并在终端中持续刷新：

- 以当前图像宽高绘制边框
- 在 `setpoint` 位置绘制“短竖直径 + 一三象限圆弧”
- 在绿灯测量位置绘制“变长横线 + 二四象限圆弧”
- 当测量坐标为 `0xFFFF, 0xFFFF` 时，自动回落到图像中心绘制

说明：

- 终端查看器兼容旧版 `18-byte payload` telemetry，但旧固件下会退回到默认图像尺寸，并根据 `area` 近似反推半径
- 当前固件已将 `image_width / image_height / measurement_radius_px` 一并放入 telemetry，查看器优先使用扩展字段
- 若使用 BLE 模式，仍需先安装 `bleak`

说明：

- 脚本只依赖 Python 3 标准库，默认按当前固件的 `0xA5 0x5A + type + len + checksum` 协议逐项发送参数并等待 `ParamAck`
- `float` 参数会按 IEEE754 `float32` 打包，下位机在线解包并刷新对应运行时变量
- 当前串口实现基于 `termios`，适用于 Linux / macOS 主机环境

### 协议与目录解耦

- `../protocol/`：顶层协议单一真源，定义消息类型、参数 key、ACK 状态
- `../Host_tools/guidance/`：主机侧调参与终端查看工具，与 `Dart_guidance`、`Esp32_bridge` 同级，不参与 STM32 固件烧录
- `Drv/Inc/guidance_protocol_generated.h`：由 schema 自动生成的 STM32 侧头文件

这样做的目的不是缩小板端 bin 体积，而是避免主机端与固件端手写两份协议常量后逐渐漂移。

## 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。


### OpenMV USB-C 相机预览

这个功能是独立调试工具：主机用 Type-C/USB 直连 OpenMV，临时运行 OpenMV 端帧源并在本机播放相机画面。它不经过控制板 UART1，也不会把预览逻辑挂进 `OpenMV_guidance/src/main.py`。

运行：

```bash
./.script/openmv-ports
./.script/camera-stream --port /dev/ttyACM0
```

工具会输出本地预览地址，默认是：

```text
http://127.0.0.1:8081/
```

只抓一帧：

```bash
./.script/camera-grab --port /dev/ttyACM0
```

说明：

- OpenMV 的 USB-C 在主机上通常表现为 `/dev/ttyACM*`；这是 OpenMV USB VCP 调试通道，不是 OpenMV 到 STM32 的 UART1。
- 默认预览会在画面左上角叠加分辨率、帧率和目标像素坐标；需要原始画面可加 `--no-debug-detector`。
- 按 Ctrl+C 退出后，工具默认复位 OpenMV，使其重新回到正常上场入口。
- 需要排查帧流时可使用 `--raw-dump /tmp/openmv.bin`，再运行 `./.script/parse-frames /tmp/openmv.bin`。

### OpenMV 视频内录与导出

OpenMV 端现在会在运行时自动进行 MJPEG 分段录像：

- 只写入 `/sdcard/recordings` 或 `/sd/recordings`；未检测到 SD 卡时禁用录像，不回写板载 flash
- 默认单段 `15s`，最多保留 `12` 段
- 达到段数上限或剩余空间不足时会自动删除最旧录像

主机侧不需要直连 OpenMV，只连接控制板串口即可导出录像。仓库提供了两个脚本：

```bash
./.script/record-list
./.script/record-pull
```

说明：

- `record-list`：列出当前板上可下载的录像段
- `record-pull`：默认下载最新一段到仓库 `record/` 目录
- 若要下载指定文件，可直接运行：

```bash
python3 Host_tools/guidance/guidance_video_fetch.py pull --name rec_00012.mjpeg
```

导出流程为：主机先通过现有参数协议让 STM32 进入 OpenMV 透传模式，再经该透传访问 OpenMV 的 MicroPython REPL 拉取录像文件。
