# 主机工具与板端诊断探针

本目录的 Qt 播放器、主机 Python 和 ROS 2 launch XML 不参与板端应用 CMake、
交叉编译或部署。主机 Python 使用标准库组织解析、时间轴和几何计算；ROS 2 导入集中在
bag 写入边界，无需创建 ROS package 或运行 colcon。

[`canmv_tune.py`](canmv_tune.py) 在 Dart V1 的 CanMV MicroPython 上运行，
用于调整与 RT-Smart C/C++ 同量纲的曝光行数、灰度候选阈值、背景对比度和四路舵机零点。
调参项集中在文件顶部；相机需配套 RAW 接口，解释器构建及运行方法见
[`canmv_tuning/README.md`](canmv_tuning/README.md)。关闭相机后可直接在当前固件标定 PWM。

[`sc038hgs_raw/`](sc038hgs_raw/README.md) 是单独交叉构建的板端相机诊断探针，
包含 RAW 转换器和历史 ISP 对照资源。它不参与正式应用的构建或部署，
手动运行时独占相机资源。

[`pwm_swing/`](pwm_swing/README.md) 是独立的四路舵机诊断程序，按 333 Hz、
1520 µs 中心、±100 µs 摆幅逐路和同时测试。显式运行
`bash .script/build-rtos pwm_swing` / `bash .script/deploy-rtos pwm_swing` 构建与部署，
板端手动启动后约 43 秒结束。
当前 launcher 清单启用识别与录像，PWM 测试行已注释；按需手动测试时，
开机配置与日志说明见该工具说明。

[`wifi/wifi_test.py`](wifi/wifi_test.py) 是 CanMV MicroPython 独立 Wi-Fi 测试脚本，
在 VS Code 插件中直接运行。默认检测 RTL8189 / SDIO 网卡并扫描附近热点，空结果自动重试；
填写 SSID 和密码后验证连接、IP 和链路；填写上传地址后逐帧上传相机 JPEG。
[`wifi/video_server.py`](wifi/video_server.py) 在电脑端用 FFmpeg 编码 H.264/HLS，
供手机网页播放，可通过公网隧道跨网络访问。运行方法、实测和固件要求见 [Wi-Fi 说明](wifi/README.md)。

[`wifi_receiver/`](wifi_receiver/README.md) 是原生 RT-Smart 回传配套的 Rust 主机服务。
`dart_app --wifi on --wifi-config /data/app/wifi.conf` 将同一张采集图异步 JPEG 压缩并上传；
Rust 接收后调用 FFmpeg 编码 H.264，提供本地网页/HLS，同时保存每轮 MP4。
构建、配置、网络条件和验收命令见该工具说明。

## SC132GS MicroPython 出图测试

[`sc132gs_test.py`](sc132gs_test.py) 在 CanMV VS Code 插件中直接运行，不参与
`dart_app` 构建、部署，也不修改启动脚本。先停止其他使用相机的程序和 IDE 脚本。
默认 `CHECK_CHIP_ID=False`，不调用 `dart_board.camera_probe()`，不读取或匹配
`0x3107/0x3108`，直接使用固件配置的 SC132GS 模式启动采集。
该开关只控制 Python 层预检查，不能禁止原生驱动或 ISP 初始化读取 ID；
固定绑定固件也不等于完全跳过底层 I2C 通信。
`CHECK_CHIP_ID=True` 才显式检查 `0x30..0x33` 地址上的芯片 ID `0x0132`；
停止采集会由驱动断开相机电源，启用校验时若启动前 I2C 无响应，会尝试驱动启动后复查。
`PROBE_ONLY` 不启动驱动，断电时无法探测，不应据此判定模组损坏。
默认选择手册明确列出的 1080 × 1280 全幅 RAW10 模式、5000 µs 手动曝光和 1 倍模拟增益，
经 K230 ISP 裁剪、转换后向 IDE 显示 640 × 480 RGB565 彩色图像，并检查实际图像格式。
默认 `TEST_SECONDS=0`，持续预览直到在 IDE 中手动停止，不保存图片；每秒报告接收帧数和接收帧率。
RGB565 是 ISP 输出格式，不是传感器 MIPI 原始格式。将 `TEST_SECONDS` 设为正整数可运行指定秒数，
结束后输出接收帧数和平均接收帧率并释放资源。
手动曝光参数与已实板出图的彩色预览一致；当前固件的自动曝光错误仍未修复，保持关闭。
`[PASS] probe` 只证明 I2C 识别成功；`[PASS] first frame` 表示收到首帧，
`[PASS] capture` 才表示本轮限时采集及清理完成。出图不代表图像质量、标定或满帧率已验收。

参数集中在脚本顶部：`PROBE_ONLY=True` 显式只读芯片 ID（不受 `CHECK_CHIP_ID` 开关影响）；`PREVIEW_ENABLED=False`
关闭 IDE 显示但仍采集；`TEST_SECONDS=0` 为默认持续运行，正整数为限时秒数；`USE_VGA_MODE=True`
改用 SDK 的 640 × 480 传感器裁剪模式，标称 240 fps；该模式的完整时序不在所提供的四份 PDF 中。
`USE_VGA_MODE=False` 保持默认全幅输入，输出仍是 640 × 480 中心裁剪。
`EXPOSURE_US` 单位为微秒，`ANALOG_GAIN` 为线性倍率；实际曝光上限及增益量化由
驱动处理。收到错误或 IDE 停止请求后清理相机和虚拟显示，不重启板子，也不无限重试。
日志中的 `received_fps` 包含 Python 和 IDE 显示开销，不是传感器帧率测量值。

默认启用 `COLOR_SAMPLING_ENABLED=True`：在 IDE 预览中依次点击目标区域的两个对角，
确定至少 4 × 4 的矩形；当前插件以点击发送虚拟触摸，不使用拖动框选。
黄色框表示正在采样，默认采集随后 30 帧未叠加标记的 RGB565 输出，完成后变为绿色，
显示 Lab 均值，并在控制台输出 `COLOR_THRESHOLD = (L_min, L_max, a_min, a_max, b_min, b_max)`，
可作为 CanMV `find_blobs()` 的颜色阈值。再次点击两个对角会重新采样，旧区域的数据不混入新区域。
框选坐标来自预览，统计在板端绘制边框和文字之前完成；采集期间请保持目标位于选区内。
这里采集的是选区确定后的实时帧，不冻结此前在 IDE 中看到的某一帧。

结果默认写入 `/sdcard/sc132gs_color_sample.json`，每次完成覆盖上一次结果，包含选区、
采样帧数及首尾帧编号、请求的曝光/增益、Lab 均值与标准差、分位数包络和颜色阈值。
标准差合并各帧内部波动及帧间变化，精度受 CanMV 原生整数统计限制。
阈值取每帧各通道 5%～95% 分位数的整体包络，再加 `COLOR_THRESHOLD_MARGIN` 容差，
限制在 L=0～100、a/b=-128～127 内；它是目标外观特征参数，不是 ISP 颜色校正文件。
脚本保持手动曝光和增益，未锁定 ISP 白平衡；光照、ISP 白平衡或成像配置变化后应重新采样验证。
图片仍不保存；SD 卡写入失败时结果保留在控制台，采样不足设定帧数时不输出完整标定结果。

`COLOR_SAMPLE_FRAMES`、`COLOR_PERCENTILES` 和 `COLOR_THRESHOLD_MARGIN` 控制采样数量与阈值范围；
`COLOR_RESULT_PATH=""` 只输出控制台结果。设置 `COLOR_ROI=(x, y, width, height)` 可启动后自动采样，
适用于 `PREVIEW_ENABLED=False` 或固件没有 IDE 虚拟触摸的情况，坐标基于 640 × 480 输出图像。
`COLOR_SAMPLING_ENABLED=False` 恢复仅预览与出图检查。

2026-11-10 在 STOP_SAFE 固件上实测：固定 `COLOR_ROI=(288,208,64,64)` 完成 30 帧、
122880 个像素的 Lab 统计及 JSON 写入，下载后验证帧数、选区和数值范围；
本轮 8 秒共收到 349 个真实 RGB565 帧。随后 IDE 两点选区测试在 `sensor.run()`
初始化阶段返回 `VICAP_NOTREADY`，尚未进入触控采样，不能标为交互验收通过。
另一次受控整机软件重启后，直接启动 IDE 版仍为 0 帧；该次 UART 捕获到首条
`0x0103=0x01` 写入失败，尚未进入颜色统计，故障原因仍由固件侧排查。
固件的停止后重跑失败及非人工整机重启仍未解决，固定选区通过不代表这些项目通过。

启动前校验 `Sensor.get_type()`、固件元数据的 `type`、输入宽高和 RAW10 数据类型；
`firmware sensor_mode` 打印固件实际返回的模式元数据，不再仅按脚本开关拼接模式说明。
`nominal_fps` 是固件标称帧率，元数据校验成功不表示传感器已经初始化或收到真实帧。

`DIAGNOSTICS_ENABLED=True` 在启动失败时检查传感器设备节点，
并检查当前模式对应的 `/bin/sc132gs-*` ISP 文件是否存在且非空；文件存在不代表解析通过。
原生 VICAP 初始化失败路径可能已经关闭相机电源，因此启动失败后不再重新探测 ID
或读取 `0x0100`，也不修改共享 I2C 控制器的配置。清理后读取出现的 `EPERM` 不能
证明最初失败是 I2C 通路故障或权限问题；应保留 UART0 上的第一条原生 sensor / ISP 错误。
`vicap init failed(-1609203696)` 对应 `0xa0158010 / K_ERR_VICAP_NOTREADY`，
发生在初始化阶段，不是取帧超时，也不能仅据此认定四通道接线或 MCLK 有问题。
启动诊断不强制切换 GPIO、不修改传感器寄存器、不启用 TPG，并保留原始异常。
修正诊断不等于修复底层 `VICAP_NOTREADY`；脚本不会吞掉该异常或将其报告为采集成功。
原生 VICAP / ISP 的详细日志需查看 UART0（115200）；IDE 的 MicroPython 软重启不会重启 RT-Smart。

真实相机取帧失败后，诊断会打印接收端配置和只读传感器寄存器。
跳过 ID 时使用 `I2C_ADDRESS=0x30`，可按模组实际固定地址调整，诊断不读 ID 寄存器；
它还会临时启用 ISP 内部测试图（TPG）取一帧作对照，随后关闭 TPG。
脚本开关不改变固件内核的探测和初始化实现，底层 `vicap init failed` 仍保留原样。
内部测试图不显示在 IDE、不计入相机帧数，也不将原始失败转为成功；IDE 停止请求
不会触发诊断。`snapshot failed(3)` 是 CanMV 包装层统一返回的取帧失败标记，
**不是底层 MPI 原始错误码**。诊断中的 `csi_enum` / `lanes_enum` 是 SDK 枚举值，
不能直接当作 CSI 下标或实际通道数，脚本同时打印该固件的四通道枚举供比较。

固件依据本机 `/mnt/mydata/kRTOSSDK` 的 `micropython` 分支和 Dart V1 配置，
须包含 `dart_board.camera_sensor()`、SC132GS 驱动及对应 ISP 数据库。
当前适配板配置为 `CONFIG_MPP_ENABLE_SENSOR_SC132GS=y`、
`CONFIG_MPP_ENABLE_CSI_DEV_0=y`、`CONFIG_MPP_SC132GS_CSI0_4LANE=y`，
通过逻辑 CSI0 同时使用物理 CSI0/CSI1 数据组；不要另开第二个独立 CSI1 相机。
不能使用旧 SC038HGS 固件或套用其 GPIO4 PWDN / GPIO2 TRIG 控制。
I2C 失败时检查固件、供电和排线；I2C 成功但出图失败时检查四通道接线、外部 MCLK
和 ISP 文件。外部晶振的实际频率仍须根据实物或 BOM 确认，不能由脚本验证。

已离线使用 Poppler 阅读下载目录中的四份 SC132GS PDF，并渲染核对模组引脚图和应用电路：

| PDF | 与脚本相关的核对结果 |
| --- | --- |
| `3763184980SC132GS_Datasheet_V3.0.pdf` | 第 6、11 页明确 1080H × 1280V、全幅 RAW10 最高 120 fps；第 15、17 页区分 RAW/PHY 位宽与 MIPI/LVDS 选择；第 32 页支持窗口裁剪，第 18、33 页说明 PLL 分频及特殊帧率由 FAE 提供。 |
| `3763188902SC132GS_数据手册_V2.6.pdf` | 第 1 页同样列出全幅 RAW10 120 fps；第 31 页给出窗口宽高寄存器和帧率计算。其 VTS 默认值为 `0x061a`，V3.0 为 `0x0546`，不能依赖跨版本复位默认值。 |
| `3763228682SC132GS_Application_Note_V1.1_for-PLCC.pdf` | 第 5、9、10 页给出上电、复位、1.8 V I/O、2.5 V AVDD、1.2 V DVDD、6–27 MHz EXTCLK 和四对 MIPI 数据线要求，不提供 SDK 的 VGA 240 fps 初始化表。 |
| `3763268327SC132摄像头120度无畸变 镜头 仁恒2025年12月23日.pdf` | 单页模组图：22-pin 接口的 pin 1 为 3.3 V、pin 6 为 MCLK，接口包含四对 MIPI 数据线；不能将模组 3.3 V 输入当作芯片 AVDD 或 I/O 电压。 |

因此 VGA 模式不是由 Python 把尺寸写错，而是固件中的另一组传感器时序；现有 PDF 不足以
证明这组 240 fps PLL/四通道时序适合实物。默认先使用手册全幅工作点，但这也不构成硬件出图验收。
`/dev/sensor_sc132gs_csi0` 在内核启动探测成功后才注册；缺失时，修改 Python 的模式选择不能创建该设备。
模组图的传感器标注存在拼写差异，需要识别型号时可显式启用芯片 ID 校验。主机模拟测试可运行
`python3 tests/test_sc132gs_test.py`；它只验证脚本流程和异常清理，不代替板端实测。

### SC132GS 原理图核对

以下载目录 `20261107-223105.jpg` 中的 `K230D_Upper_Board_SC132GS.kicad_sch`
为适配板依据，对照 `20261108-164010.jpg` 的主芯片和 `20261108-164346.jpg` 的
主板连接器；`20261108-164352.jpg` 是旧扩展板，不能套用其 2.8 V / PWDN / TRIG 接法。
GPIO2 接新板两颗 LDO 的 EN，高电平上电；GPIO4 不接该模组复位。
新板向模组 pin 1（适配板 FPC pin 22）提供 3.3 V，I2C1 经 TXS0102 转为 1.8 V。
这不是向裸芯片 AVDD 直接送 3.3 V：SC132GS 手册要求 AVDD 2.5 V、DVDD 1.2 V、
DOVDD 1.8 V；模组内部稳压和 XSHUTDN 电路未提供，不能仅凭适配板图确认其内部电源时序。

| 信号 P / N | 模组 pin | 适配板 FPC pin | K230 接收引脚组 |
| --- | --- | --- | --- |
| Data 0 | 20 / 21 | 3 / 2 | RX0_D0_P / N |
| Data 1 | 17 / 18 | 6 / 5 | RX0_D1_P / N |
| Data 2 | 11 / 12 | 12 / 11 | RX1_D2_P / N |
| Data 3 | 8 / 9 | 15 / 14 | RX1_D3_P / N |
| Clock | 14 / 15 | 9 / 8 | RX0_CLK_P / N |

以上是按网络名核对的预期连通关系，不证明实物连接器镜像、排线接触面或 PCB 信号质量正确。
Y1 使用 1.8 V，OUT 经 R9 22 Ω 到 FPC pin 17 / 模组 pin 6 的 MCLK；不是 K230 输出时钟。
图中没有 Y1 频率或型号。V3.0 第 18、35 页给出输入时钟 6–27 MHz、占空比 45–55%，
PLL 分频由 FAE 提供，不能理解为同一寄存器表自动适配任意时钟。
当前 SDK 配置的 `CONFIG_MPP_SC132GS_EXTCLK_HZ=24000000` 只是时序元数据依据，
不会改变 Y1 或重编程 PLL。VGA 表注释含 `mclk=12`，但同段还写了与 SC132GS 手册不符的
AVDD 2800 / DVDD 1500，不能据此断言晶振应该是 12 MHz 或按其电压上电。

诊断依据 V3.0 第 15、17、20 页解码：`0x3018[7:5]` 为通道数减一，
`0x3031[3:0]` 为 RAW 位宽，`0x3037[6:5]` 为 PHY 位宽，**不是 MIPI/LVDS 选择位**；
接口选择是 `0x4b01[3]`（0 = MIPI，1 = LVDS）。脚本只读补充该寄存器及
`0x3652/0x3653/0x3654` 的 lane / clock 极性与延迟原值，不修改传感器配置。
所提供的手册未定义 `0x3019`，因此读回 `0x00` 不能作为“四通道输出已启用”的独立证据。

2026-11-08 在所连 Dart V1 板上复现：640 × 480 / 1080 × 1280 两种输入均在
约 3006 ms 后取帧失败；不启用 IDE 显示时同样失败。寄存器读回
`0x0100=0x01`（启流）、`0x3018=0x72`（四通道）、`0x3019=0x00`，
RAW10 位宽和输出尺寸与所选模式一致，接收端也报告四通道。
VGA 模式临时将接收端 PHY 档位从 800M 改为 1200M、1600M 对照也未取得真实帧，
没有把这些临时覆盖写入默认配置。ISP 内部 TPG 能收到 640 × 480 图像，说明同一输出链可处理生成输入，
但**尚未取得真实相机画面**。下一步应检查传感器外部 MCLK、模组供电、
排线的四对数据线及 CSI0 时钟对，以及传感器 PLL / MIPI 接收时序；
目前不能仅凭此结果断言哪一项硬件损坏或哪个时序参数错误。

同日针对 `vicap init failed(-1609203696)` 再次实测：
`/dev/sensor_sc132gs_csi0` 能打开，SC132GS ISP 文件存在；改用 XML/JSON 解析仍在初始化阶段失败。
GPIO2 / GPIO34 / GPIO35 的复用分别为 GPIO2 / IIC1_SCL / IIC1_SDA；
诊断时手动将 EN 拉高并等待 200 ms 后，100 kHz I2C1 扫描仍为空，`0x30..0x33` 读取均失败。
临时诊断结束已将 EN 恢复为断开状态，没有把强制上电写入测试脚本或启动文件。
这与此前已启流但收不到帧是不同的失败阶段；先完全断电后检查 FPC 连接，再核对 EN、
适配板 3.3 V / 1.8 V、电平转换及外部 MCLK，并结合 UART0 日志定位。
设备节点只证明内核曾完成注册，不能证明当前 I2C 通信正常；目前没有恢复真实相机出图。

后续冷启动后曾恢复 I2C 通信；为排除 ID 匹配，另按固定地址 `0x30` 直接执行
VGA 初始化表，不读取 `0x3107/0x3108`，也不调用固件的型号探测接口。
该轮 119 次寄存器写入全部返回成功（约 14 ms），随后普通寄存器回读在
`0x3209` 首次失败，错误为 `OSError(1)`；后续 `0x0100` 的读写也失败。
将 I2C 改为 100 kHz / 400 kHz，或将重复起始读取改为分开的 STOP/START，均未恢复通信。
这证明不依赖 ID 匹配也能复现初始化后的通信失效，但一次首次失败地址
不能证明 `0x3209` 本身有问题，也尚未确定是传感器时钟/供电、电平链路还是 I2C 控制器状态。
不能把跳过 ID 当作相机出图已修复。

2026-11-09 已直接捕获同一解释器进程的原生 SDK 日志，无需外接 UART0：
`kd_mpi_vicap_init, sensor open failed.`；同时 `/dev` 下没有任何 `sensor_*` 节点，
打开 `/dev/sensor_sc132gs_csi0` 返回 `ENODEV`。这轮 `NOTREADY` 的直接原因是
内核启动探测未完成设备注册，尚未进入传感器模式初始化或 MIPI / ISP 启流。
`camera_sensor()` 的固定模式来自编译时元数据；能构造对象不代表内核设备存在。
SDK 的 Dart 配置现已改为 `MPP_SC132GS_SKIP_CHIP_ID_CHECK=y`、
`MPP_SC132GS_I2C_ADDRESS=0x30`，注册、初始化及连接检查不再依赖芯片 ID；
普通寄存器传输和模式回读错误仍会报错。此更改需要安装重新构建的内核，
仅更新测试脚本不能补注册当前内核中缺失的设备。

## 录像逐帧播放器

[`video_player/`](video_player/README.md) 移植远程 `k230_v1` 的 Qt 播放器，
适配当前录像目录及图片、录像 CSV、检测 CSV 的精确关联：

```bash
bash .script/player records/rec_0001
# 无窗口检查图片与文字关联：
bash .script/player --check records/rec_0001
```

滑块或左右键切帧，点击查看原图像素坐标；右侧显示原始文字及文件行号，叠加有效
目标和扫描窗口。以 `image_file` 和帧身份关联，缺图/丢帧不会按行错配。每轮录像应
同时归档对应的 `records/detection.csv`，避免后续运行覆盖。详细构建、过滤和核对规则见
播放器说明。

`pngs` 容器可直接用播放器打开；需要独立 PNG 时运行
`python3 tools/rec_extract_pngs.py records/rec_NNNN --strict --output /tmp/frames`。
`python3 tools/rec_stall_report.py records/rec_NNNN --log logs/dart_app.log`
支持 PNGS、H.264、DBV、PGM/PBM，分别汇总输入间隙、成功保存源帧的间隙、
CSV `buffer_full`、文件边界及写盘/日志耗时。输入连续但录像帧间隙大时，结合
`rec_buffer_dropped`、编码与 IO 峰值区分原因；缺失 CSV 行也会造成表观输入间隙。
`--skip-windows` 按完整 `op=timing` 周期跳过启动窗口。CSV 的 `received_us` 是输入时间，
不能用来测磁盘完成间隔；报告耗时超过阈值也不能单独证明调用线程发生了文件 I/O。

## IMU 日志转 ROS 2 bag

`imu_log_to_bag.py` 读取板端 `runtime/imu/Session` 写出的 v1 CSV 格式 `.log`，
默认生成带消息定义的 MCAP bag，便于 Foxglove 直接打开和 ROS 2 回放。
先把板端 `/data/app/logs/imu_NNNNNN.log` 中需要的一轮复制到主机，例如
将 `imu_000001.log` 保存为 `records/imu.log`。IMU start/stop 状态行记录实际文件名；
当前正在写入的文件可能被板端共享锁阻止读取，可在停止或下一次上电后读取上一轮。
以下命令在仓库根目录执行：

```bash
# 本机可用的完整 bag 环境为 Rolling；bash 用户用 setup.bash。
source /opt/ros/rolling/setup.zsh

/usr/bin/python3 tools/imu_log_to_bag.py records/imu.log -o records/imu_bag
ros2 bag info records/imu_bag
```

输出目录必须不存在；不会覆盖原始日志或已有 bag。失败时清理本次创建的临时 bag。
`metadata.yaml` 和 `.mcap` 文件位于输出目录内。`--storage sqlite3` 可改为 SQLite bag。
帮助可直接运行 `python3 tools/imu_log_to_bag.py --help`，不要求 source ROS。

默认 `--position fixed`：平移为零，姿态直接使用日志中的四元数，不重新积分角速度，
也不重新执行板端估计器。日志里的四元数顺序是 w,x,y,z，输出 ROS 字段为 x,y,z,w；
表示从传感器轴到本批次参考系的旋转，不取逆、不额外转换为 ENU/NED。
输出前仅归一化四元数；原始字段完整保留在 `/imu/status`。

如需观察估算的平移：

```bash
/usr/bin/python3 tools/imu_log_to_bag.py records/imu.log \
  -o records/imu_bag_motion --position velocity
```

日志没有位置，`velocity` 模式使用日志里的参考系速度，按原始 `timestamp_ns`
做梯形积分；每个批次的位置从零开始。这是主机离线估算，受速度漂移和缺失数据影响。
拒绝样本、读取错误、序号缺口、dropped 增长和过大时间间隔均不跨越积分；
当前 `integrated=0` 时位置保持不变，从该样本建立下一个区间的基点。
原始时间戳只用于这个位置估算，不决定 bag 的事件顺序。

新日志每个文件对应一轮启动；多个输入文件按命令顺序编号为 session 1、2……。
旧版追加日志中的多轮运行仍按出现顺序编号。
可限定一个批次：

```bash
/usr/bin/python3 tools/imu_log_to_bag.py records/imu.log \
  -o records/imu_s1_g2 --session 1 --generation 2
```

`seed` 成功、失败和 `reset` 都保留在诊断话题。只有已接受、有参考系、姿态和速度
有限的 `sample`，以及成功的 `seed`，会生成位姿/TF。时间 gap 样本若仍被估计器接受，
会按上层所见输出其保持的姿态，质量位完整保留。拒绝样本仍输出原始测量，
但不把旧估计冒充新位姿；`read_error` 不伪造测量。

默认遇到残缺行、倒退的处理时间、非法序号或字段即报出文件名与行号。
断电后末行不完整等情况可用 `--skip-bad-rows` 显式跳过，终端逐条报告。
未知日志版本和损坏的列名始终报错。

## Foxglove 直接打开

在 Foxglove 的 Open local file 中选择输出目录里的 `.mcap` 文件，或直接拖入。
此方式不需要运行 Bridge 或 `ros2 bag play`。

1. 添加 **3D** 面板，打开面板设置，将 **Display frame** 设为 `world`。
2. 在 **Transforms** 中打开 `imu_reference_s1_g1` 与 `imu_link_s1_g1` 的可见性，
   显示坐标轴和标签。参考坐标轴保持不动，`imu_link` 随日志姿态旋转。
3. 播放或拖动时间轴；也可启用 `/imu/pose`，以箭头查看传感器位姿。
4. 添加 **Plot** 面板，选择 `/imu/rpy.vector.x`、`.y`、`.z`，分别观察 roll、pitch、yaw
   （rad）；`/imu/velocity.vector.x` 等可显示参考系速度（m/s）。
5. 添加 **Raw Messages** 或 **Diagnostics** 面板，查看 `/imu/status` 的事件、原始时间、
   质量位、generation、dropped 及原始字段；`/imu/generation.data` 可单独绘制批次变化。

每个 `session + generation` 使用独立的 TF 父子节点：

```text
world                         仅供显示的共同基准
  ├─ imu_reference_s1_g1       第一轮、第一次积分的参考系
  │    └─ imu_link_s1_g1       该批次的传感器位姿
  └─ imu_reference_s1_g2       重新 seed 后的新参考系
       └─ imu_link_s1_g2
```

`world -> imu_reference_*` 是人为设置的单位变换，方便在同一个 3D 面板查看各批次；
它不表示不同批次已经全局配准。重新 seed 后不会在前后两个批次之间插值，
也不会把位置连成一条连续轨迹。历史坐标轴可能继续显示，按需关闭其他批次或使用筛选参数。
转换结束时终端会列出实际生成的坐标系名称。

bag 与消息 `header.stamp` 统一采用日志的 **recorded_ns 处理顺序**，保持上层命令
与样本的真实先后。第一轮从 ROS 时间 1 秒开始，同轮保留处理时间间隔；
同一纳秒内的事件相隔 1 ns。不同轮之间间隔 1 秒，避免板子重启后的单调时钟倒退。
这条时间轴不代表 UTC，也不保留两轮间的真实停机时间。所有原始时间都保留在诊断中。
TF 与位姿在相同 bag 时间发布，便于 Foxglove 时间轴跳转。

## 通过 Foxglove Bridge 回放

本机检查时 Rolling 已有 rosbag2、MCAP 和标准消息，但尚未安装 `foxglove_bridge`。
如果需要 WebSocket 回放，先在所选 ROS 发行版安装对应主机依赖：

```bash
sudo apt install ros-${ROS_DISTRO}-foxglove-bridge
# 若使用其他尚未安装 bag 支持的 ROS 2 环境：
sudo apt install ros-${ROS_DISTRO}-rosbag2-py ros-${ROS_DISTRO}-rosbag2-storage-mcap \
  ros-${ROS_DISTRO}-ros2bag ros-${ROS_DISTRO}-launch-xml
```

启动 Bridge 和一个暂停的 bag 播放器：

```bash
ros2 launch ./tools/foxglove_launcher.xml \
  play_bag:=true bag:="$PWD/records/imu_bag"
```

在 Foxglove 选择 Open connection → Foxglove WebSocket，地址填
`ws://127.0.0.1:8765`。
启动 XML 的子进程不接收终端键盘；连接后在另一个已 source ROS 的终端执行：

```bash
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
# 暂停：
ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause '{}'
```

播放器发布 `/clock`，Bridge 使用 `use_sim_time=true`；随后按上面的 3D 步骤设置。
XML 默认监听本机，`address:=0.0.0.0` 可供其他主机连接，`port:=8766` 可改端口，
`rate:=0.5` 可半速回放。

也可单独启动 Bridge，再在另一个已 source 同一 ROS 环境的终端中回放：

```bash
ros2 launch ./tools/foxglove_launcher.xml
ros2 bag play records/imu_bag --clock 100 --loop
```

XML 启动的是 ROS Bridge 和可选播放器；Foxglove 网页或桌面应用由用户打开。
离线文件方式最适合拖动时间轴；WebSocket 方式观察 ROS 实际回放。

## 话题与验证

| 话题 | 类型 | 数据 |
|---|---|---|
| `/tf` | `tf2_msgs/msg/TFMessage` | 显示基准到批次参考系、参考系到传感器的动态变换 |
| `/imu/pose` | `geometry_msgs/msg/PoseStamped` | 日志姿态与固定/离线估算位置，批次参考系 |
| `/imu/raw` | `sensor_msgs/msg/Imu` | 传感器轴 SI 角速度和比力；orientation_covariance[0]=-1 |
| `/imu/raw_counts/acceleration`、`/imu/raw_counts/gyroscope` | `geometry_msgs/msg/Vector3Stamped` | 原始 ADC 计数，传感器轴 |
| `/imu/temperature` | `sensor_msgs/msg/Temperature` | 摄氏温度 |
| `/imu/velocity` | `geometry_msgs/msg/Vector3Stamped` | 积分速度，m/s，批次参考系 |
| `/imu/rotation`、`/imu/rpy` | `geometry_msgs/msg/Vector3Stamped` | 累计旋转、RPY，rad，批次参考系 |
| `/imu/linear_acceleration` | `geometry_msgs/msg/Vector3Stamped` | 去重力加速度，m/s²，批次参考系 |
| `/imu/generation` | `std_msgs/msg/UInt64` | 原始批次编号，跨 session 可能重复 |
| `/imu/status` | `diagnostic_msgs/msg/DiagnosticArray` | 每条原始事件全部列、session、文件行号、元数据和可视化状态 |

诊断 KeyValue 的 value 按 ROS 契约为字符串；数值曲线使用对应的标准数值话题。
协方差未记录，因此保留 unknown，不伪造标定值。

```bash
# 无 ROS 依赖时，运行解析/几何测试，跳过 ROS bag 与 launch 用例。
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s tests \
  -p test_imu_log_to_bag.py -v
```

source ROS 2 后同一命令还会验证真实 MCAP/SQLite 写入和 CDR 读回、消息定义、
四元数方向、位移积分、重积分分段、时间重建、异常数据和 launch XML。
2026-10-16 在本机 Rolling 上 7 项测试通过；另以临时 MCAP 实际执行了 bag 回放，
成功接收 5 组 TF、10 条 `/clock`，并验证了暂停/恢复服务和带空格的 bag 路径。
验证范围不包含 Foxglove GUI 的人工交互，Bridge 安装后可按上述步骤验证。

官方参考：[Foxglove ROS 2/MCAP](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)、
[3D/TF](https://docs.foxglove.dev/docs/visualization/panels/3d)、
[Bridge 启动配置](https://github.com/foxglove/foxglove-sdk/tree/main/ros/src/foxglove_bridge)。
