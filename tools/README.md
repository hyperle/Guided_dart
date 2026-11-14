# 主机工具与板端诊断探针

本目录的 Qt 播放器、主机 Python 和 ROS 2 launch XML 不参与板端应用 CMake、
交叉编译或部署。主机 Python 使用标准库组织解析、时间轴和几何计算；ROS 2 导入集中在
bag 写入边界，无需创建 ROS package 或运行 colcon。

[`canmv_tune.py`](canmv_tune.py) 在 Dart V1 的 CanMV MicroPython 上运行，
用于调整与 RT-Smart C/C++ 同量纲的曝光行数、灰度候选阈值、背景对比度和四路舵机零点。
调参项集中在文件顶部；相机需配套 RAW 接口，解释器构建及运行方法见
[`canmv_tuning/README.md`](canmv_tuning/README.md)。关闭相机后可直接在当前固件标定 PWM。
注意：其 `_dart_detect` 桥接（`canmv_tuning/detector_bridge.cpp`）仍引用已删除的灰度
`RoiDetector`，当前无法构建；颜色识别参数以 [`sc132gs_test.py`](sc132gs_test.py) 为准，
其 `DETECTOR_CONFIG` 键与 C++ `Detect::Config` 字段路径一致。

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
传感器以原生 1080 × 1280 模式运行。默认 `NATIVE_FULL_FRAME=False`：ISP 把整个视野
硬件缩放为一路 480 × 640 RGB565，不裁剪（宽高比 0.84 显示为 0.75，画面横向压缩）。
`NATIVE_FULL_FRAME=True` 输出完整 1080 × 1280 像素，需要 PITCH_FIX 固件，旧固件会直接报错提示。

IDE 预览经虚拟显示（`Display.VIRT`，`to_ide=True`）：VO 回写由硬件 JPEG 编码，CPU 只复制帧。
`PREVIEW_FPS`（默认 60）限制复制到显示的频率，两次复制之间继续取帧并计数。
`DRAW_ROI=True` 时，蓝框标“绿灯扫描区”，表示本帧提取绿灯 blob 的扫描范围，不是灯体轮廓。
绿色椭圆和 `L green 灯体` 才表示检测到的绿灯；取色选区单独标“取色选区”，采样完成后为白框。
紫框标“装甲搜索”，并显示裁剪后实际窗口的像素宽、高。
搜索框在检测和取色标注全部完成后绘制；紫框使用 3 像素紫边和黑色衬边，避免被候选标签遮挡。
画面左上固定显示搜索区状态：紫框使用当前帧传给 `find_blobs()` 的装甲搜索区，
跟踪间隔帧标“间隔”，表示该帧有绿灯锚点和候检窗口，但没有执行灯条扫描。
没有绿灯锚点时显示“装甲搜索:无合格绿灯”；窗口被图像边界完全裁掉时显示
“装甲搜索:窗口在画面外”。这两种情况均不虚构搜索框。
`DRAW_CANDIDATES=True` 时，`G1…` 用橙色显示绿灯候选，`B1…` 用蓝色显示蓝色灯条候选，
两类默认各保留评分最高的五个。没有已确认目标时，绿灯候选显示“颜色”“太小”“太大”
“圆度”“填充率”“宽高比”或“关联”等原因；等待连续帧确认的候选显示“待确认”。
被淘汰组件的矩形框是其包围框；选中的绿灯改用圆形轮廓，ISP 缩放画面上显示为相应椭圆，
并标出圆度 `C` 和灯径 `D`。灯体中心取外接框中心，灯径取外接宽、高的几何平均值
`D = sqrt(width × height)`；绿色椭圆也按同一外接范围绘制，不再用颜色命中像素面积反推大小。
绿灯圆度达标后按 `面积 × (1 + light.circularity_weight × 圆度) × 颜色质量`
排序，避免圆度五次方把灯内的小圆碎片排在整灯前面；跟踪关联同时比较命中像素面积的一致性。
正式绿灯检测先用原始绿色 Lab 配置取得颜色种子，再用一次扩展 Lab 扫描提取包含种子的灯体，
圆度、灯径、绿色椭圆和装甲搜索框均使用灯体范围，不再使用灯内绿色碎片的外接框。
灯体扫描的 L 下界向下扩 `light.body_l_margin=15`、上界到 100，a/b 各扩
`light.body_ab_margin=20` 并至少覆盖 `±light.body_neutral_chroma=8`，以连接褪色及白亮核心；
扩展值均限制在合法 Lab 范围内。灯体中原始绿色种子像素占比至少
`light.body_min_green_fraction=0.02`，颜色统计仍只使用原始绿色阈值。
种子和灯体扫描均启用 `light.merge_margin_px=2` 的相邻组件合并（设为 0 可关闭），
灯条和宽阈值诊断补查不做此合并。灯体被扫描区截断、绿色占比不足或形状不合格时拒绝锚定，
不回退到种子生成紫框；预览显示相应原因，未取得合格灯体时显示“装甲搜索:整灯范围未确定”。
跟踪局部扫描未取得灯体时，沿用原有的全帧重扫逻辑。
绿灯候选统一检查 `light.min_circularity`，包括面积小于 30 像素或最大边长
小于 8 像素的远距离候选；圆度不足时拒绝选入和跟踪，并显示“圆度 实测值<阈值”。
圆度结合 ISP 修正后的轴比及填充率接近圆形面积比 `π/4` 的程度；填充不足和过于填满
矩形框都会扣分，避免实心方块仅因长短轴接近而取得高分。
蓝色灯条按面积、细长度及颜色质量排序，绘制包围框和长轴，并显示“颜色 Lab数值”
“长度”“灯条长宽比”“灯条填充率”“无合格绿灯”“绿灯搜索区外”“未配对”或“装甲待确认”。
灯条按矩形筛选，不应用绿灯圆度门槛：所有尺寸均检查长宽比（至少 2）和旋转矩形填充率
（至少 0.5），小灯条也不能跳过这两项检查；预览保持矩形框。
蓝条补查在没有合格绿灯时也运行，将蓝色配置的 L 下界向下扩 10、上界扩到 100，
a/b 上下界各扩 20（限制在合法范围），再用原蓝色 Lab 配置检查颜色；正式装甲板识别
仍要求绿灯锚点、搜索区内的同色双灯条及配对几何通过。
正式装甲板搜索区位于灯体上方，下边缘锚在灯体外接框上边缘（再扣除配置间距），
宽、高直接取灯径 `D` 乘以
`armor.window_width_k`、`armor.window_height_k`，不设置最小边长；
窗口超出图像的部分按图像边界裁剪。左上紫字同时显示 `D`、配置倍数和裁剪前的计算尺寸，
用来区分窗口计算尺寸与画面内可见尺寸。绿灯局部扫描区不再合并装甲搜索区或装甲目标范围；
灯条扫描始终独立使用上述紫框对应的全帧坐标区域。
无目标时还会用 `DETECTION_DEBUG_GREEN_MIN_L`（默认 70）到 100 的 Lab 亮度范围补查，
不限制 a/b，最小面积为 1，种子步长为 2；微小亮点可能被跳过。补查只在需要预览时运行，
默认最多每 `DETECTION_DEBUG_INTERVAL_MS=500` ms 更新一次，等待灯目标确认时暂停绿灯补查。
补查跳过超过灯面积上限的组件及包围框，避免背景触发大范围颜色统计；最多检查工作区容量内的
原始组件，先用面积和轴比选出展示数量两倍的候选，再做颜色统计（默认最多 10 个）。
每种诊断颜色以 `DETECTION_DEBUG_BUDGET_MS=20` 限制提取完成后的候选处理，
原生 `find_blobs()` 及单次统计无法中途打断。
补查结果只供诊断，缓存数值特征并显示 `debug=年龄ms`，到期丢弃；通过补查几何和颜色检查的
组件标“待复核”，达到数量或时间限制时显示“诊断限额”。
`DETECTION_DEBUG_MAX_CANDIDATES` 控制每类候选展示数量。
2026-11-13 在 PITCH_FIX 固件上用本脚本默认配置实测：开预览 15 s 收到 1554 帧（103.6 fps），
IDE 实际收到约 29 fps 的预览帧；关预览 10 s 收到 1198 帧（119.8 fps），两次都正常释放资源。
此前的软件 JPEG 预览（544 × 640）只有约 7.4 fps，最初的全幅软件转换只有 0.9 fps。

原生 1080 宽输出的板上实测：ISP 以 16 像素对齐的行距写入（1080 宽的行间隔 1088 像素，
528、544 宽不填充），驱动报告的 `stride` 却仍是 1080；旧 `sensor.py` 按 1080 宽分配缓冲区，
每帧约越界 28 KiB，并把行距错误的数据当作致密图像，因此画面错位。PITCH_FIX 固件
（`mpp_vb_mgmt.c` / `sensor.py`）按实际行距分配、映射和刷新缓冲区，`snapshot()` 在包装图像前
原地压紧各行；`snapshot(dump_frame=True)` 仍返回硬件行距的原始帧。
原生 1080 宽输出目前不可用：在 PITCH_FIX 固件上，单路 1080 × 1280 RGB565、不开预览，
取帧也会超时，ISP 停住、`sensor.stop()` 卡死，需要复位。旧固件上 1080 宽输出叠加
第二路缩放通道或 1080 × 1280 虚拟显示预览时同样停住；只取帧、不在帧上做转换的单路
1080 宽 RGB888 多次正常。共同点是 1080 宽输出叠加 CPU 写帧或额外 DMA，推测与离线
ISP 在 120 fps 下的内存带宽有关，尚未证实。因此默认使用已验证的 480 × 640。
默认 `TEST_SECONDS=1200`，预览 20 分钟，不保存图片；每秒报告接收帧数和接收帧率。
RGB565 是 ISP 输出格式，不是传感器 MIPI 原始格式。将 `TEST_SECONDS` 设为正整数可运行指定秒数，
结束后输出接收帧数和平均接收帧率并释放资源。

需要录像时，修改脚本顶部配置后直接运行：

```python
RECORDING_ENABLED = True
RECORDING_DIR = "/sdcard/sc132gs_recordings"
RECORDING_FPS = 30
RECORDING_BITRATE_KBPS = 4000
RECORDING_QUEUE_FRAMES = 6
RECORDING_FRAME_BYTES = 256 * 1024
RECORDING_STOP_TIMEOUT_MS = 1000
```

录像默认关闭。开启后，脚本在传感器通道 1 配置 YUV420SP，绑定硬件 H.264 编码器，
将视频写入分片 MP4；通道 0 的 RGB565 仍用于识别、取色和预览。录像保存未经标注的 ISP 彩色画面，
不含识别框、取色框和音频。全幅模式录制 480 × 640 的完整视野，VGA 模式录制 640 × 480；
`NATIVE_FULL_FRAME=True` 时录像仍使用 480 × 640，原生预览的板端限制见上文。
`RECORDING_FPS` 为编码器目标帧率，范围 1～60，独立于 `PREVIEW_FPS`；
`PREVIEW_ENABLED=False` 时仍可录像。MP4 使用编码帧的真实时间戳，实际录制帧率需在板端测量。

编码缓冲在主线程取出后，仅复制到预分配的有界队列，随后立即归还编码器。
默认队列为 6 × 256 KiB = 1.5 MiB，正在写盘的帧也占用一个槽位，另有固定 64 KiB 参数集缓存。
队列允许 1～8 帧，单槽允许 32～512 KiB，两者乘积最多 4 MiB；这些数字是编码数据队列的容量，
不包含相机、硬件编码池、线程栈或解释器本身的内存。
排队已满或一帧连同参数集超过槽位容量时，直接丢弃该新帧，不扩容、不等待写盘，也不覆盖在写的帧。
H.264 丢帧后请求新的 IDR，并丢弃其前的后续 P 帧，直到有空槽接收新的关键帧；
因此拥塞可能跳过一段 GOP，不能用任意丢 P 帧的方式破坏后续解码依赖。
控制台每秒报告已写帧数、当前排队数、丢帧数、超大帧数和排队峰值。

录像目录专用于单个录像脚本实例，上级目录需已存在；工作线程自动创建末级目录，流式扫描已有条目，
然后选择递增的 `sc132gs_0001.mp4`、`sc132gs_0002.mp4` 等名称，保留已有文件。
目录内所有普通文件的逻辑大小与本轮新增 MP4 的大小累计上限为 **2,000,000,000 字节（2 GB）**，
包括 MP4 头、分片和 NAL 长度字段；不只统计 H.264 载荷。目录含子目录等非普通条目时报告错误，
避免漏算其占用。开始时已有文件达到上限则不新建录像；写入前预留整个分片的大小，放不下完整分片时
停止录像，继续采集、识别和预览。不会自动删除旧文件，也不再轮转创建新文件绕过容量限制；
已有内容超过上限时保持原样。此上限不计算文件系统自身的簇和元数据开销，目录不应由其他进程同时写入。
码率单位为 Kbit/s；默认 4000 对应约 30 MB/分钟，实际大小由画面和编码器决定。
`TEST_SECONDS` 同时控制整轮采集时长，设为 0 可持续采集；录像仍受目录上限约束。

扫描目录、打开文件、写入、最终 `flush()` 和关闭全部由独立 `_thread` 工作线程执行；
主线程不调用录像文件的 `flush()`、`fsync()` 或全局 `os.sync()`，也不跨磁盘操作持有队列锁。
这里直接构造每帧一个分片的 MP4，避免传统 MP4 随录像时长增长的样本表，也不调用当前会持有 GIL 的
`kd_mp4_write_frame()` 包装接口。配套 CanMV 的 POSIX 文件 `write/flush/close` 在系统调用期间释放 GIL，
写盘或最后同步变慢时，其他 Python 线程仍能运行。
正常结束、IDE 中断或异常时，脚本归还编码缓冲、解绑并关闭编码器，通知工作线程完成剩余写入。
主线程最多等待 `RECORDING_STOP_TIMEOUT_MS`（默认 1000 ms）；超时会报告工作线程尚未完成，
继续清理预览等资源，由工作线程保留尚在使用的缓冲，不强制释放或无限等待。
写盘、同步或关闭失败及收尾超时均不会报告整轮采集成功。断电仍可能丢失尚未落盘的数据。
录像不能与 `PROBE_ONLY=True` 或 `RAW_CHECK_ENABLED=True` 同时启用。
固件需提供 `_thread`、`os.ilistdir()`、POSIX 文件接口和 `media.vencoder` 的帧率配置、非阻塞取流及
`RequestIDR()` 接口；缺少接口或 SD 卡不可写时报告错误。
录像会增加内存带宽及 SD 写入开销，默认关闭时的既有帧率数据不能作为开启录像后的性能依据。
新增录像路径已做队列拥塞、慢写盘、同步超时、容量边界和原有功能的主机测试，
并用真实 H.264 生成 MP4，经 FFprobe 核对时间戳和 FFmpeg 解码；脚本通过 MicroPython 字节码编译。
尚未完成板端录像回放、慢 SD 卡下的线程调度及性能验证。

脚本在 `sensor.reset()` 之后、`sensor.run()` 之前调用 `sensor.auto_exposure(False)`，
关闭 ISP 自动曝光和自动传感器增益，并按 `AUTO_WHITE_BALANCE` 设置
`sensor._dev_attr.pipe_ctrl.bits.awb_enable`、`wb_enable`。默认 `True` 运行 ISP 自动白平衡。
2026-11-12 板上对比（5000 µs、10 倍增益、同一场景）：AWB 关闭时整幅偏蓝紫，
亮部 RGB 均值约 175/186/190、暗部 23/36/77；只清 `wb_enable` 与否几乎无差别，
说明 ISP 仍使用配置文件的固定增益（R 1.12、B 2.51），该增益不适合当前光照。
AWB 开启后亮部约 188/187/183，颜色正常。`False` 保留旧行为，供需要固定增益时对比。
本机固件的 `set_auto_whitebal()`、`set_auto_gain()` 仍是会抛出 `NotImplementedError` 的占位方法，
因此直接使用固件已有的 ISP 配置位，无需修改或重新烧录固件。
`sensor.run()` 启动后才有有效的传感器句柄，此时设置 `ANALOG_GAIN` 和 `EXPOSURE_US`，
随后开始取帧。仅在 `run()` 后修改 ISP 配置位不会提交给正在运行的 ISP。
`[PASS] probe` 只证明 I2C 识别成功；`[PASS] first frame` 表示收到首帧，
`[PASS] capture` 才表示本轮限时采集及清理完成。出图不代表图像质量、标定或满帧率已验收。

参数集中在脚本顶部：`PROBE_ONLY=True` 显式只读芯片 ID（不受 `CHECK_CHIP_ID` 开关影响）；`PREVIEW_ENABLED=False`
关闭 IDE 显示但仍采集；`TEST_SECONDS=0` 持续运行，正整数为限时秒数；`USE_VGA_MODE=True`
改用 SDK 的 640 × 480 传感器裁剪模式，标称 240 fps；该模式的完整时序不在所提供的四份 PDF 中。
`USE_VGA_MODE=False` 为默认全幅 1080 × 1280 输入，输出 480 × 640 或原生 1080 × 1280；`True` 时裁剪为 640 × 480。
`EXPOSURE_US` 单位为微秒，`ANALOG_GAIN` 为线性倍率；实际曝光上限及增益量化由
驱动处理。收到错误或 IDE 停止请求后清理相机和虚拟显示，不重启板子，也不无限重试。
日志中的 `received_fps` 统计每一帧；预览复制、采样和监视占用的时间会降低它，
`PREVIEW_ENABLED=False` 且不采样时才反映相机帧率。

`ISP_3DNR_ENABLED=False`（默认）在 `run()` 前关闭 ISP 时域降噪；全幅 ISP 配置默认开启它，
会把历史帧混入当前帧，使运动目标拖影、发糊。

`RAW_CHECK_ENABLED=True` 不经过 ISP，直接用 `VICAP_DUMP_RAW` 读取传感器 RAW10 全幅帧，
预热 5 帧后统计 `RAW_CHECK_FRAMES` 帧，每帧取得统计后立即释放，不使用 IDE 预览和颜色采样。
请让均匀照明、未过曝的白色或灰色目标充满画面。输出 `[RAW]` 报告：
四个 Bayer 相位的平均码值（相近即单色传感器或纯红外光照）；绿色是否位于固件声明的
BGGR 相位；帧间或帧内是否发生相位翻转（MIPI 行/像素错位，会造成整屏紫色闪烁或条带）；
各 MIPI 字节通道的同色像素粗糙度（某一通道显著偏高提示对应差分对或 FPC 接触问题）；
固定曝光和增益下的帧亮度变化（灯光频闪）。RAW 码值含传感器黑电平偏移，
数据手册未给出该值：完全遮住镜头运行一次，报告给出 `RAW_BLACK_LEVEL = N`，填入脚本顶部后，
后续比例扣除黑电平。只有已知黑电平、白色目标未过曝且信号不低于量程 10% 时，
才给出当前光照下白色所需的 R/B 白平衡增益估计；它只作参考，不是标定结果。
信号低于 10% 时报告曝光不足：ISP 会把噪声和微小的黑电平误差放大成偏色，并在降噪中抹糊。
120 fps 下曝光上限约 8200 µs，更暗时提高 `ANALOG_GAIN`。
RAW 检查只是诊断，结束后打印 `[PASS]`、`[FAIL]` 或无法判定时的 `[INFO] RAW check complete`，
不代表采集验收。

`OUTPUT_MONITOR_ENABLED=True` 在普通采集中逐帧统计 ISP 输出中央四分之一面积的 Lab 均值
（取输出图像中央四分之一，例如 480 × 640 输出取 240 × 320）
（在叠加标记之前），每秒打印范围；相邻帧任一通道变化达 8 记为跳变并打印前 3 次细节。
紫色表现为 a > 0、b < 0。测试时保持画面静止。IDE 中看到闪烁而这里没有跳变，
说明闪烁来自预览传输路径而不是 ISP。统计会降低 `received_fps`，默认关闭，不用于帧率验收。

默认 `DETECTION_ENABLED=True` 时，预览里的两次点击改为选择**检测 ROI**，取色通过 `c` 切换。
当前 IDE 的虚拟触摸只提供坐标与按下/抬起事件，不携带鼠标左右键或键盘修饰信息，
所以仍依次点击两个对角，不使用拖动框选。画面下方按钮和 IDE 终端键位对应如下：

| 操作 | 画面按钮 / 终端键 | 行为 |
| --- | --- | --- |
| 选择检测区域 | `R 检测ROI` / `r` | 后续两次点击确定至少 4 × 4 的矩形；启动时默认使用此模式 |
| 选择取色区域 | `C 取色` / `c` | 后续两次点击交给颜色采样，不改变检测区域 |
| 清除检测区域 | `F 全图` / `f` | 恢复自动全图搜索，返回检测 ROI 选择模式 |
| 取消未完成的两点选择 | 终端 `x` | 丢弃第一个角点，保留已生效的检测区域 |

字母键需在 **IDE 终端**发送，预览窗口自身不会转发键盘；按终端输入方式可能需要回车。
脚本用非阻塞轮询读取，每帧最多处理 16 个字符；终端输入不可用时仍可点击画面按钮。
检测与取色共用一个虚拟触摸实例，切换模式会取消未完成的角点选择。
仅关闭检测、保留 `COLOR_SAMPLING_ENABLED=True` 时，预览两次点击仍直接取色。

检测 ROI 以黄色边框显示，灯和装甲的实际扫描窗口仍单独标注。
请框住完整绿灯及其上方装甲，边缘留出背景；紧贴灯边框选会使整灯恢复判定为边界截断。
选区内按 `x_stride=y_stride=1` 逐像素扫描，跟踪、丢失后的重扫和补充候选诊断都受选区限制，
不会自动退回选区外。重新框选或清除选区会清空旧跟踪与诊断缓存，重新累计确认帧数。
也可在启动前设置 `DETECTION_ROI=(x, y, width, height)`，无预览时同样有效，
坐标始终对应 `output_size()`；`COLOR_ROI` 独立控制取色区域。

远距离目标没有候选框时，先用检测 ROI 缩小扫描范围：原先全图搜索步长为 2，
默认灯种子至少 3 个颜色像素，补充绿灯诊断又只检查 `L>=70` 的亮色块。
缩放为 480 × 640 后，小灯可能被步长跳过、面积不足，或因与背景混色偏离当前 Lab 阈值。
手动 ROI 内额外保留 1～2 像素的种子用于显示“太小”，补充诊断也降低亮度门槛、适度扩展色域，
用于显示颜色不符等原因；最终目标仍须通过原有颜色、最小面积、圆度、整灯恢复与多帧确认，
框选本身不会产生目标或自动更改颜色阈值。
若报告 `no_lamp_color` 或候选标“颜色”，切换 `c` 对实际灯的颜色区域取样，
再更新 `DETECTION_COLOR_THRESHOLDS`，或显式设置 `COLOR_SAMPLE_APPLY_TO_DETECTION=True` 在线应用。
取色框应尽量避开背景；若缩放后只剩一两个像素，需增加目标在输出图像中的像素数，
例如在兼容 PITCH_FIX 固件上尝试 `NATIVE_FULL_FRAME=True`，并重新测量板端处理耗时。
新交互和选区约束的主机回归为 `python3 tests/test_sc132gs_roi.py`，已接入 CTest。
板端已用合成 RGB565 图验证原生 `find_blobs()` 的选区内小灯确认、选区外大灯排除、
单像素拒绝，以及模式切换和按钮绘制；`select.poll(0)` 与 `TOUCH.DEV_IDE` 初始化、读取、
释放也已通过接口实测。该测试没有启动相机，不能代替实际远距场景的识别与性能验证。

默认启用 `COLOR_SAMPLING_ENABLED=True`：切到取色模式后，在 IDE 预览中依次点击取色区域两个对角，
确定至少 4 × 4 的矩形。黄色框表示正在采样，默认采集随后 30 帧未叠加标记的 RGB565 输出，完成后变为白色，
显示 Lab 均值，并在控制台输出 `COLOR_THRESHOLD = (L_min, L_max, a_min, a_max, b_min, b_max)`，
可作为 CanMV `find_blobs()` 的颜色阈值。再次点击两个对角会重新采样，旧区域的数据不混入新区域。
框选坐标来自预览，统计在板端绘制边框和文字之前完成；采集期间请保持目标位于选区内。
这里采集的是选区确定后的实时帧，不冻结此前在 IDE 中看到的某一帧。

结果默认写入 `/sdcard/sc132gs_color_sample.json`，每次完成覆盖上一次结果，包含选区、
采样帧数及首尾帧编号、请求的曝光/增益、Lab 均值与标准差、分位数包络和颜色阈值。
标准差合并各帧内部波动及帧间变化，精度受 CanMV 原生整数统计限制。
阈值取每帧各通道 5%～95% 分位数的整体包络，再加 `COLOR_THRESHOLD_MARGIN` 容差，
限制在 L=0～100、a/b=-128～127 内；它是目标外观特征参数，不是 ISP 颜色校正文件。
脚本保持手动曝光和增益；JSON 的 `white_balance` 字段记录 `ISP AWB` 或 AWB 关闭。
AWB 会随场景调整增益，同一目标的 Lab 值可能随背景变化；光照、曝光、增益或白平衡配置变化后
应重新采样验证，此前在 AWB 关闭时采集的阈值需要重新采样。
缩放模式在 480 × 640 硬件缩放帧上统计，JSON 另记传感器尺寸 `sensor_size`；缩放会平均邻近像素，
统计到的标准差可能略小于原生分辨率。
图片仍不保存；SD 卡写入失败时结果保留在控制台，采样不足设定帧数时不输出完整标定结果。

`COLOR_SAMPLE_FRAMES`、`COLOR_PERCENTILES` 和 `COLOR_THRESHOLD_MARGIN` 控制采样数量与阈值范围；
`COLOR_RESULT_PATH=""` 只输出控制台结果。设置 `COLOR_ROI=(x, y, width, height)` 可启动后自动采样，
适用于 `PREVIEW_ENABLED=False` 或固件没有 IDE 虚拟触摸的情况，坐标基于输出图像：
默认 480 × 640，原生模式 1080 × 1280；`USE_VGA_MODE=True` 时为 640 × 480。触摸范围、选区边界与 JSON 的
`frame_size` 使用同一尺寸。
`COLOR_SAMPLING_ENABLED=False` 关闭取色，检测与检测 ROI 交互仍由 `DETECTION_ENABLED` 控制。

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

### SC132GS 颜色扫描

[`sc132gs_color_scan.py`](sc132gs_color_scan.py) 是独立的 CanMV IDE 脚本，依次执行
固定 `FIXED_EXPOSURE_US` 扫 `GAIN_STEPS`、固定 `FIXED_ANALOG_GAIN` 扫曝光范围。
增益为线性倍率，曝光单位为微秒；默认全幅缩放到 480 × 640。VGA 模式需相应调整曝光范围。
两组均关闭 AE、AWB、WB 和 3DNR，保留其它 ISP 处理以观察当前链路的颜色响应，
不会修改 `sc132gs_test.py`。关闭 AWB 不代表其它按增益变化的 ISP 算法也已关闭。

曝光范围由 `EXPOSURE_MIN_US`、`EXPOSURE_MAX_US` 和 `EXPOSURE_STEP_US` 配置，
默认按 10 µs 步长扫描 100、110、…、300 µs，共 21 档。只扫曝光时设置：

```python
GAIN_SWEEP_ENABLED = False
EXPOSURE_SWEEP_ENABLED = True
EXPOSURE_MIN_US = 100
EXPOSURE_MAX_US = 300
EXPOSURE_STEP_US = 10
EXPOSURE_STEPS_US = None
```

从下限递增，只有步长正好到达上限时才包含上限，始终不超过上限；上下限相同则只扫一档。
范围和步长必须为正整数，下限不得大于上限，每组最多 256 档，启动相机前校验。
`EXPOSURE_STEPS_US` 设为元组或列表（如 `(100, 150, 200, 300)`）时优先使用自定义档位。
范围仅控制曝光扫描；ROI 选区参考曝光以及增益扫描的固定曝光仍由 `FIXED_EXPOSURE_US` 控制。

运行前保持目标与光照静止，在 IDE 预览中点击同一均匀色块的两个对角，选定 ROI 后自动扫描；
也可设置 `COLOR_ROI=(x,y,width,height)`，关闭预览时必须设置。`GAIN_SWEEP_ENABLED`、
`EXPOSURE_SWEEP_ENABLED` 可分别启停两组。每档重新设置曝光和增益，丢弃至少
`SETTLE_FRAMES` 帧并持续排空至少 `SETTLE_MS`，然后采集 `SAMPLE_FRAMES` 帧。
曝光/增益回读必须有效，固定参数跨档变化或采样期间回读变化会中止扫描。
`exposure()` 回读单位为微秒，`again()` 的 `k_sensor_gain` 返回值取 `gain[0]` 作为
SC132GS 线性模式的模拟增益；也接受直接返回数值的接口。读取失败会报告原始返回值，
不会用请求值冒充回读值。
回读不是逐帧生效元数据，丢帧等待是否足够仍需板端验证。

默认保存 `/sdcard/sc132gs_color_scan_0000.csv` 和同名 `.jsonl`，自动递增编号避免覆盖。
CSV 逐帧记录请求值、采样前回读值、RGB/Lab 均值和标准差、Lab 色相与色度、
RGB 通道比例及近饱和/暗像素比例；JSONL 记录配置、每档开始/汇总、结束或中断状态。
Lab 使用完整 ROI；RGB 和近饱和比例使用最多 `RGB_MAX_SAMPLES_PER_FRAME` 个均匀网格点，
记录实际步长和样本数。色相来自每帧 ROI 的 Lab 均值，低色度时留空；跨帧色相采用圆周平均。
这些均为 ISP 输出，RGB 比例未经线性化，近饱和比例不能作为传感器 RAW 饱和率。
不保存图片。`RESULT_PREFIX=""` 只输出控制台；中断或失败会保留已写出的部分结果。

主机流程与统计回归：`python3 tests/test_sc132gs_color_scan.py`，已接入 CTest；
不代表曝光实际生效时刻、ISP 配置或颜色质量已在板端验证。

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
