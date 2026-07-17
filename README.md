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


### BLE 遥测可视化

ESP32 蓝牙桥现在走稳定 snapshot 流：STM32 通过 USART2 向 ESP32 发送二进制上行帧，ESP32 聚合后以固定频率通过 BLE notification 发布目标位置、三轴角速度和三轴加速度。USART1 继续用于 OpenMV 到 STM32 的目标测量输入。

运行方式：

```bash
./.script/ble-connect
```

常用参数：

```bash
./.script/ble-connect --scan
./.script/ble-connect --address AA:BB:CC:DD:EE:FF
./.script/ble-connect --plot
./.script/ble-connect --no-log
```

默认终端仪表会显示连接状态、接收频率、丢包数、目标 `<x, y>`、三轴速度 `deg/s` 和三轴加速度 `g`；目标丢失时显示 `<-1, -1>`。`--plot` 会额外打开 matplotlib 实时曲线窗口。

### 终端追踪图形查看

独立终端查看器 [Host_tools/guidance/guidance_terminal_viewer.py](/home/hyperlee/guided_dart/Host_tools/guidance/guidance_terminal_viewer.py:1) 用于实时查看 `setpoint` 与绿灯测量值的对位情况。

运行方式：

```bash
python3 ../Host_tools/guidance/guidance_terminal_viewer.py --config ../Host_tools/guidance/guidance_params.yaml
```

仓库内也提供了对应的一键启动脚本：

```bash
./.script/track-viewer
```

它会复用 `serial / ble` 连接配置，并在终端中持续刷新：

- 以当前图像宽高绘制边框
- 在 `setpoint` 位置绘制“短竖直径 + 一三象限圆弧”
- 在绿灯测量位置绘制“变长横线 + 二四象限圆弧”
- 当测量坐标为 `0xFFFF, 0xFFFF` 时，自动回落到图像中心绘制

说明：

- 终端查看器兼容旧版 `18-byte payload` telemetry，但旧固件下会回退到默认图像尺寸，并根据 `area` 近似反推半径
- 当前固件已将 `image_width / image_height / measurement_radius_px` 一并放入 telemetry，查看器优先使用扩展字段
- 若使用 BLE 模式，仍需先安装 `bleak`

### 协议与目录解耦

- `../protocol/`：顶层协议单一真源，定义上行帧头和消息类型
- `../Host_tools/guidance/`：主机侧终端查看与 OpenMV 调试工具，与 `Dart_guidance`、`Esp32_bridge` 同级，不参与 STM32 固件烧录
- `Drv/Inc/guidance_protocol_generated.h`：由 schema 自动生成的 STM32 侧头文件

这样做的目的不是缩小板端 bin 体积，而是避免主机端与固件端手写两份协议常量后逐渐漂移。

## 许可证

本项目采用 MIT 许可证。详见 [LICENSE](LICENSE) 文件。


### OpenMV USB-C 相机预览

相机调试入口按功能拆成短脚本，不需要在日常命令里记底层参数：

```bash
./.script/camera-ports
./.script/camera-view
./.script/camera-control
./.script/camera-photo
./.script/camera-record
./.script/camera-dump
```

`camera-view`/`camera-control` 默认只监听 OpenMV 上电入口 `/flash/main.py` 输出的 USB 预览帧，不打断相机、不进入 Raw REPL、不临时运行另一套 OpenMV 代码。工具会输出本地预览地址，默认是：

```text
http://127.0.0.1:8081/
```

相机脚本默认自动选择 OpenMV USB 口；如果自动选择失败，先运行 `camera-ports`，再用环境变量指定：

```bash
OPENMV_PORT=/dev/ttyACM0 ./.script/camera-control
```

说明：

- OpenMV 的 USB-C 在主机上通常表现为 `/dev/ttyACM*`，这是 OpenMV USB VCP 调试通道，不是 OpenMV 到 STM32 的 UART1。
- 使用默认预览时，目标坐标仍由 `/flash/main.py` 从 OpenMV UART1 发到 STM32 USART1，控制板继续按现有 `GuidanceController` 逻辑映射到舵机 PWM。
- 默认预览会在画面左上角叠加分辨率、帧率和目标像素坐标；高级参数仍可透传，例如 `./.script/camera-view --no-debug-detector`。
- 按 Ctrl+C 退出默认只关闭主机预览，不复位 OpenMV。旧的临时 Raw REPL 预览仍可通过 `--raw-repl` 显式启用。
- `camera-photo` 默认抓图到 `/tmp/omv_photo.jpg`；`camera-record` 默认写入 `record/`；`camera-dump` 默认写入 `/tmp/openmv.bin`，可再运行 `./.script/parse-frames /tmp/openmv.bin` 提取 JPEG。

### OpenMV 视频内录与导出

OpenMV 端现在会在运行时自动进行 MJPEG 分段录像：

- 只写入 `/sdcard/recordings` 或 `/sd/recordings`；未检测到 SD 卡时禁用录像，不回写板载 flash
- 默认单段 `15s`，最多保留 `12` 段
- 达到段数上限或剩余空间不足时会自动删除最旧录像

录像文件保存在 OpenMV 本地文件系统或 SD 卡中。当前控制链路不再提供经 STM32 的 OpenMV 透传导出；需要取回录像时请使用 OpenMV USB/SD 卡工具。
