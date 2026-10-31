# K230D / K230 RT-Smart 业务代码仓库（v2）

本仓库**与固件无关**：它面向 K230D RT-Smart 用户态业务代码的开发和部署。
不编译、不烧录固件；板子上必须是已支持 RT-Smart 用户态（lwp）的固件，
交叉编译出的静态 ELF 才能直接运行。

已落地：相机 C 数据面 + C++ ACL（`include/firmware/src/camera/`）、IMU/SPI
（`.../imu/`、`.../spi/`）、识别与 ROI（`src/detector/`）、灰度取证录像
（`include/firmware/src/recorder/`）、L4 业务入口与功能编排（`src/`）、开机自启动器与清单
（`runtime/launcher/`）、构建/部署脚本（`.script/`）。

先读 [`docs/STRUCTURE_DESIGN.md`](docs/STRUCTURE_DESIGN.md)（分层与依赖方向），
启动链路读 [`docs/LAUNCHER_DESIGN.md`](docs/LAUNCHER_DESIGN.md)，
板端调试读 [`docs/DEBUG_GUIDE.md`](docs/DEBUG_GUIDE.md)。
录像与日志可通过 `write-rtos off|on` 暂停、恢复；清理前自动设置暂停，
不等待板端应答。runtime 接口与可选加速度触发见 [写入控制](runtime/WRITE_CONTROL.md)。

工作流（官方文档同款流程，见
[如何运行和新增 HelloWorld 应用](https://www.kendryte.com/k230_rtos/zh/v0.8/userguide/helloworld.html)）：

```
编写业务代码 → 交叉编译出 ELF → scp/读卡器拷到板子 → 利用开机自启的 launcher 启动
```

---

## 1. 业务代码放在哪里？（最重要）

`src/` 全部属于 L4，按启动、视觉、录像、诊断和检测功能组织调用与策略。
设备、线程、文件系统、日志和编码的下层实现统一放在 `include/firmware/src/`。
`runtime/` 保留进程启动器与本轮不迁移的 IMU 采样服务。
各目录的 `CMakeLists.txt` 分别声明本目录目标，模块依赖保持分离。

| 目录 | 内容 | 目标 / 产物 |
|---|---|---|
| `runtime/launcher/` | 开机自启动器（纯 POSIX C11，平台无关）+ `startup.list` 母版 | `launcher` → `build-cross/launcher/launcher` |
| `src/dart_entry.cpp`、`src/{startup,vision,recording,diagnostics}/` | L4 入口、配置、流程与故障决策 | `dart_app` → `build-cross/dart_app/dart_app` |
| `src/detector/` | L4 灯/装甲板检测与 ROI，只消费帧数据 | `dart_detection`（INTERFACE，包含类 `.cpp`） |
| [`src/control/`](src/control/README.md) | L4 四路舵机初始位置、左右/上下坐标映射与脉宽合成 | `dart_control`（INTERFACE，包含类 `.cpp`） |
| `include/firmware/src/recorder/` | 录像队列、线程、紧凑 DBV 与兼容 PNG/PGM/PBM | `dart_recording`（INTERFACE，包含类 `.cpp`） |
| `runtime/imu/` | 独立 IMU 采样、上层共用的积分状态和异步原始/积分日志 | `dart_imu_logging`（INTERFACE，包含类 `.cpp`） |
| `include/firmware/src/camera/` | L3 帧数据/相机抽象 + L2/C 数据面实现 | `dart_camera_types` / `dart_camera_core` / `dart_camera_k230` / `dart_camera_acl` |
| `include/firmware/src/{imu,spi}/` | IMU 抽象/估计器 + SPI 驱动 + K230 适配器 | `dart_imu` / `dart_iim42652` / `dart_imu_k230` |
| [`include/firmware/src/pwm/`](include/firmware/src/pwm/README.md) | 四路 PWM 接口、独立输出任务与 K230 引脚适配 | `dart_pwm` |
| `include/firmware/src/{board,system,logging,storage}/` | 板级组装、进程资源、日志与目录操作 | `dart_board` / `dart_platform` |
| `.script/` | 构建、部署、板端清理和相机回归检查；`install-rtos` 可注册短命令 | — |
| [`tools/`](tools/README.md) | 主机播放器、IMU 日志工具与独立 RAW 相机诊断探针 | 独立构建的工具 |

`dart_entry.cpp` 将板级工厂注入 `startup/run.cpp`，启动流程调用视觉、录像与诊断模块；
设备初始化和系统资源操作由 `include/` 的实现承担。可选 IMU 采样已独立运行，
纯视觉控制通过独立 PWM 任务消费每帧的两轴映射结果，不依赖 IMU。`launcher` 负责进程启动与监督。应用说明见
[`src/startup/README.md`](src/startup/README.md)。

当前启动清单启用 `dart_app` 纯视觉 PWM 与录像，预设点为图像中心，默认零增益使四路保持初始脉宽。
标定参数与故障处理见 [舵机控制说明](src/control/README.md)。独立的
[`PWM 测试`](tools/pwm_swing/README.md) 保留为手动工具，与 `dart_app` 的 PWM 输出互斥。

硬件资源是独占的：**同一时刻只能有一个进程持有 VICAP/VB**，所以"识别"和"录像"
合在 `dart_app` 一个 ELF 里，清单里也只能出现一条。详见
[`docs/LAUNCHER_DESIGN.md`](docs/LAUNCHER_DESIGN.md) §5。

C++ 文件按紧密相关的职责组织：相关值类型可以同文件，只有一个所有者的内部
状态放进所属对象；无对象状态的简单操作可以直接写成函数。新增类或调用层应当
隐藏实现、维护约束或管理资源，并能减少调用方需要理解的细节。
文件以完整功能为边界，配置、结果、临时数据和辅助步骤跟随所属功能。
例如曝光钳位和单位换算直接位于相机设置流程，估计器在同一文件定义自己的
`Config`、`Estimate` 和 `SeedResult`；内部步骤不因有名字就独立成文件或类。
帧数据、设备接口等确有独立消费者的契约可以单独保留。
消费者包含带包含保护的 `.cpp`；类外方法和普通函数使用 `inline`，模板保留完整定义。
每个可包含文件仅在 includes 后声明一次具名命名空间，不使用匿名命名空间、
`using namespace` 或命名空间别名。进程入口 `dart_entry.cpp` 仅连接板级工厂与 L4 启动流程，
由全局 `main()` 调用 `run_application()`。
文件名按所在目录省略重复信息，例如 `detector/armor.cpp`。
C 数据面继续使用 `.c` / `.h`。

CMake 的 C++ `INTERFACE` 目标传播依赖，`*_checks` 对所有层级的可包含文件逐一编译，
避免遗漏独立 include 依赖。主机回归与结构检查：

```bash
cmake --preset host
cmake --build --preset host
ctest --test-dir build --output-on-failure
python3 tests/check_cpp_structure.py
```

## 2. 目录结构

```
dart/
├── cmake/
│   └── toolchain-riscv64-musl.cmake   # 交叉编译工具链配置（rv64imafdcv/lp64d）
│                                      # 链接脚本用 SDK 的 src/rtsmart/libs/mk/link.lds
├── .script/                           # 所有构建、部署和检查脚本
│   ├── install-rtos                   # 一次性把 .script/ 追加进 ~/.bashrc / ~/.zshrc
│   ├── build-rtos                     # 交叉编译（-r 重编 / <目标名> / --host）
│   ├── deploy-rtos                    # 统一部署入口：SSH/scp、读卡器、USB/MTP
│   ├── clean-rtos                     # 清空 /sdcard/app（保留目录本身）
│   ├── clean-rtos-data                # 清除新旧分区的日志与录像，支持 --dry-run
│   ├── _rtos-common.sh                # 构建/部署/清理共用的定位、环境与板端访问层
│   ├── deploy-mtp.sh                  # USB/MTP：GIO 上传、备份和 SHA-256 回读校验
│   ├── check-camera-pool.sh           # 相机资源池与生命周期的 host 回归检查
│   ├── board.env.example              # 板子地址配置示例（board.env 已被 gitignore）
│   └── README.md                      # 命令用法与依赖
├── src/                               # 业务入口、应用编排与领域算法
│   ├── dart_entry.cpp                 # 业务进程入口
│   ├── startup/                       # L4 参数、服务组装与生命周期
│   ├── vision/                        # L4 采集、识别与帧分发
│   ├── recording/                     # L4 录像配置与启停决策
│   ├── diagnostics/                   # L4 诊断字段与检测记录
│   ├── detector/                      # L4 检测与 ROI
│   └── CMakeLists.txt                 # 分别声明领域库与业务应用目标
├── runtime/                           # launcher、启动清单与现有 IMU 采样服务
├── include/                           # 下层接口、设备/系统/录像实现及库目标声明
├── docs/                              # 设计、调试与验收契约
├── CMakePresets.json                  # host / cross 两个预设
└── CMakeLists.txt                     # 顶层构建骨架
```

## 3. 工具链与 SDK（一次性）

```bash
# 工具链：riscv64-unknown-linux-musl-gcc。build-rtos 会按以下顺序自动探测：
#   $K230_TOOLCHAIN_DIR
#   <仓库>/third_party/toolchain/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin
#   /mnt/mydata/SelfGuidingDart/third_party/toolchain/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin
#   /opt/toolchains/riscv64-linux-musleabi_for_x86_64-pc-linux-gnu/bin
export K230_TOOLCHAIN_DIR=/path/to/riscv64-musl/bin     # 探测不到时显式指定
```

环境要求：`cmake >= 3.20`、`ninja`、`scp`（网络部署用）。SDK 位于
`/mnt/mydata/kRTOSSDK`（仅用于提供 SDK 头文件与链接脚本；用 `K230_SDK_ROOT` 或
`-DK230_SDK_ROOT=...` 覆盖）。

HAL 头文件和库只取 SDK `.config` 的 `CONFIG_BOARD_CONFIG_NAME` 对应输出目录，
避免 SDK 保留多块板子的构建产物时误链旧板库。使用其它固件输出时可显式设置
`cmake --preset cross -DK230_SDK_OUTPUT_DIR=/path/to/sdk/output/<board_defconfig>`。

新版 Dart V1 固件的 SC038HGS 使用 CSI2、640 × 480 RAW10。应用通过 MCM 直采并
按 `raw10 >> 2` 输出保留 256 级亮度的灰度 Y8，识别与录像共用同一图像，
不依赖 ISP 标定文件。识别以 `--threshold`（默认 50）提取候选，再用候选与周围背景的
灰度均值差（`--min-contrast`，默认 12）过滤低对比度干扰；阈值不改变相机图像。
录像默认保存单文件灰度 PGM，也可选择灰度 PNG；DBV/PBM 是可选的二值导出格式。
每轮最多 500 MB；
多帧合并到一个文件，达到容量上限后停止录像，识别继续运行。
`/data` 分区另设 2 GB（2,000,000,000 字节）总占用保护，计入旧录像、日志和分配开销，
并预留 64 MiB；空间不足时停止录像并保留已有文件。
本机 MCM RAW10 的 DDR 排列已通过 640×480 渐变图逐像素验证：
默认 `--raw-layout packed10-lsb`，每 4 个像素按低位连续打包为 5 字节，每行 800 字节。
后端同时修正本机 SDK 把 DMA 字节跨度当成输入像素宽度的问题；更换固件时需重新验证。
应用默认通过曝光接口设置 400 行曝光（`--exp 400`）；K230 后端同步设置
帧长 VTS=600，通过现有传感器寄存器接口完成，无需修改固件或标定像素时钟。
`--exp 0` 保留传感器初始化曝光；RAW 路径不支持 `--ae`。
开机清单使用 `--exp 400`，对应帧长 VTS=600。此前连续 PBM 版本实测录像
55.881 FPS，在 499,995,987 字节正常停录；目标场景曝光/阈值尚未标定，详见 [实板记录](src/startup/README.md#验证状态)。
适配细节及板端验证步骤见 [相机说明](include/firmware/src/camera/README.md#sc038hgs--dart-v1)。

## 4. 编译

```bash
build-rtos              # 短命令：全部板端目标（等价于下面第一条）
bash .script/build-rtos              # 全部板端目标
bash .script/build-rtos dart_app     # 只编业务应用
bash .script/build-rtos launcher     # 只编启动器
bash .script/build-rtos --host       # host 编译检查（不需要工具链，不可上板）
```

产物：`build-cross/<目标名>/<目标名>` —— riscv64 **静态链接** ELF（`-T link.lds`，
入口 `0x200000000`），可直接拷板运行。`build-cross/compile_commands.json` 供 clangd 使用。

> 板端 ELF 必须链 `k230_rtos_app`（提供链接脚本与 `-static`）；少了它就会链成动态
> ELF，RT-Smart 用户态加载不了。

## 5. 部署到板子

日常短命令（在 `.script/`，用法见 [`.script/README.md`](.script/README.md)）。先做一次
PATH 注册，之后就能直接敲、不带 `./`：

```bash
bash .script/install-rtos --both   # 一次性：追加到 ~/.bashrc 与 ~/.zshrc（-u 可撤销）
source ~/.bashrc                   # 当前终端生效（zsh 用 source ~/.zshrc）

build-rtos            # 交叉编译（-r 清 build-cross 重编；<目标名> 只编一个；--host 编译检查）
deploy-rtos           # 上板 launcher + dart_app + startup_final.list -> /sdcard/app
clean-rtos            # 清空 /sdcard/app 下的内容（目录本身保留；固件自带的 examples/ 不动）
clean-rtos-data       # 清除两个分区的日志与录像，跳过写入中的项；--dry-run 可预览
```

应用重命名后，首次更新已有板子应运行 `deploy-rtos`，将 `dart_app` 与
`startup_final.list` 一并部署；之后可继续按目标增量部署。

`deploy-rtos` 默认通过 GIO/MTP 部署到当前 USB 板子的 `/sdcard/app`，
无需先设置环境变量。其它地址或传输方式可通过环境变量或 `.script/board.env` 配置：

```bash
export BOARD_IP=192.168.1.100     # ssh/scp（需免密），优先级最高
export SDCARD=/media/$USER/<卡>   # 读卡器：直接写已挂载的 SD 卡
export SDCARD='gphoto2://Kendryte_CanMV_001000000/store_ffff0001'   # 板子用 USB 挂着（MTP）
```

`store_ffff0001` = 板端 `/sdcard`（程序与清单），`store_ffff0002` = `/data`（日志与录像）；
脚本按两个分区分别操作，数据地址可通过 `DATA_SDCARD` 覆盖。
MTP 部署会检查分区根目录的 `revision.txt`；缺少时须有已部署的 launcher 与清单，否则
拒绝写入。地址写进 `.script/board.env` 更省事：
`cp .script/board.env.example .script/board.env`（该文件已被 gitignore）。

USB/MTP 部署使用 `gio copy` 直接操作存储 URI，不通过 GVfs 的 FUSE 路径写文件。
脚本先将旧文件备份到主机 `/tmp/dart-mtp-backup.*`，上传临时文件并完整回读校验
SHA-256，再改名替换并再次校验；日志和录像保留。官方
[如何运行示例程序](https://www.kendryte.com/k230_rtos/zh/main/userguide/how_to_run_samples.html)
也支持用 MTP 手动拷贝独立编译的 ELF。此前 FUSE `cp` 出现的 0 字节现象不能据此判断
所有 MTP 写入都失败，部署结果以回读校验为准。

各种传输方式统一使用 `deploy-rtos`，地址通过环境变量或 `board.env` 配置：

```bash
# 板子有网络：scp + md5 校验
BOARD_IP=192.168.1.100 bash .script/deploy-rtos              # 两个 ELF + 清单
BOARD_IP=192.168.1.100 bash .script/deploy-rtos dart_app     # 只推一个目标

# 读卡器：同时挂载程序和数据分区；全量部署默认清日志与录像，--keep-logs 可保留
SDCARD="/media/$USER/SDCARD" DATA_SDCARD="/media/$USER/DATA" bash .script/deploy-rtos --keep-logs
SDCARD="/media/$USER/SDCARD" bash .script/deploy-rtos dart_app

# USB/MTP：默认板子无需配置；其它 URI 也可通过 SDCARD 指定
bash .script/deploy-rtos
```

自定义构建目录统一使用 `DART_BUILD_DIR`，构建和部署时设置为相同路径；部署兼容旧变量
`BUILD_DIR`。相对路径以仓库根目录为基准，绝对路径也可使用。

然后在板端 msh 里**先手动验证，再注册开机自启**：

```text
msh> /sdcard/app/launcher -t                            # 只校验清单，不启动进程
msh> /sdcard/app/dart_app --seconds 10 --record off      # 采集 + 识别
msh> /sdcard/app/dart_app --seconds 10                   # 加上录像
msh> /sdcard/app/dart_app --seconds 60 --record off --log-console on   # 日志实时刷串口
```

`--log-console on` 只在**前台手动运行**时有意义：日志同时写 stderr，可直接在 115200
串口上观察（限速 30 行/秒）；`--log off` 可关掉文件日志，两者可组合成纯控制台模式。
详见 [`docs/DEBUG_GUIDE.md`](docs/DEBUG_GUIDE.md) §3.1。

```bash
make menuconfig
# RT-Smart Configuration
#   Rtsmart auto execute command string = /sdcard/app/launcher
make && 烧录固件
```

之后日常迭代只替换 `/sdcard/app/` 下的 ELF 或编辑 `startup_final.list`，重启即生效。
**不要**把自启命令写成 `@preload`（Fast Boot 跑镜像内的 rtapp 备份、不读文件系统，
替换 SD 卡不生效）；secure boot 固件强制 `@preload`，本方案不可用。
清单语法、日志布局与 v1 教训见 [`runtime/launcher/README.md`](runtime/launcher/README.md)。

## 6. VSCode + clangd 使用

1. VSCode 打开本仓库，按提示安装推荐扩展：
   **clangd**（`llvm-vs-code-extensions.vscode-clangd`）、
   **CMake Tools**（`ms-vscode.cmake-tools`）、**CMake**（`twxs.cmake`）。
2. 先跑一次 `bash .script/build-rtos`（或 `--host`）生成 `build-cross/compile_commands.json`。
3. clangd 自动读取编译数据库 → 获得完整补全、跳转、诊断。
   - `include/CMakeLists.txt` 里有 `*_CLANGD_COMMANDS` 目标，专门为只在交叉构建下
     编译的 SDK 相关源码（camera/k230、spi、imu/k230）生成编译命令。
4. 常用命令：`Ctrl+Shift+B` 编译；"Tasks: Run Task" 可部署到板子。

## 7. 写业务代码时常用的 SDK 头文件（已自动加进 include 路径）

| 功能 | 头文件示例 | 路径来源 |
|---|---|---|
| 标准 C | `<stdio.h>` 等 | musl 工具链自带 |
| 外设 HAL | `<drv_gpio.h>` `<drv_i2c.h>` 等 | SDK `output/*/rtsmart/libs/rtsmart_hal/include` |
| 多媒体 MPP | `<vb.h>` `<sys/ioctl.h>` 等 | SDK `src/rtsmart/mpp/include` 等 |
| 板级配置宏 | `CONFIG_*`（autoconf.h 自动 include） | SDK `include/generated/autoconf.h` |

需要 MPP 库/OpenCV/nncase 等链接库时，参照官方 SDK
`src/rtsmart/libs/mk/libmpp.mk` 等文件把 `-L`/`-l` 参数补进应用即可。

## 8. 常见问题

- **找不到工具链**：`export K230_TOOLCHAIN_DIR=...`（见 §3），或把工具链放到
  `<仓库>/third_party/toolchain/<工具链目录>/bin`。
- **SDK 路径不同**：`export K230_SDK_ROOT=/你的SDK路径` 后再 `build-rtos`。
- **`Cannot find source file`**：模块目录改名后没同步所属目录的 `CMakeLists.txt`（曾在
  `recording/`→`recordor/` 改名时发生过），路径以本文件 §1 表格为准。
- **板端提示"已在运行"**：上一轮 launcher 未退干净，检查 `/data/app/logs/*.pid`。
- **clangd 无补全**：确认 `build-cross/compile_commands.json` 存在（跑过一次
  `build-rtos`），并已安装 clangd 扩展。
- **与固件的关系**：本仓库不编译、不烧录固件；板子上必须是已支持 RT-Smart
  用户态（lwp）的固件，ELF 才能直接运行。
