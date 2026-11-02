# CanMV 相机与四路舵机标定

在 CanMV VS Code 插件中打开并运行 [`../canmv_tune.py`](../canmv_tune.py)。
所有调参项都在文件最前面；修改后停止、重新运行即可。
相机灰度图显示在 IDE 的帧缓冲窗口，控制台打印 FPS、亮像素比例及 RT-Smart 参数。
舵机始终保持各自的固定零点，不做摆动或视觉闭环。

## 目标识别

`DETECTION_ENABLED=True` 使用 `_dart_detect.Detector`，直接编译本仓库
`src/detector/roi.cpp`，与 RT-Smart 应用复用同一套亮灯、装甲板配对和 ROI 跟踪。
参数键及单位与 C++ 源码一致；`DETECTOR_CONFIG` 在脚本顶部暴露所有识别配置，
其中已标定的圆度和舵机零点设置按当前脚本保留。
字典键就是 `RoiDetector::Config` 字段路径，例如 `light.min_area`、
`armor.parallel_deg`、`roi.min_width`。未知键、整数参数的小数值和无效配置会报错。

- 面积使用像素数（px²），长度与中心坐标使用像素（px），允许半像素坐标。
- `flat_deg` / `parallel_deg` 使用度，`_k` 是原生倍率，比例字段使用 0..1。
- `plate_up_lo_pct` / `plate_up_hi_pct` / `plate_offx_pct` 使用原生百分数。
- `roi.max_gap` 使用毫秒，其余确认/丢失/周期字段使用帧数。
- `GRAY_THRESHOLD` 设置灰度候选下限，对应 `--threshold`，不改变灰度图。
- `workspace.min_contrast` 设置候选与背景的最小灰度差（默认 12），对应 `--min-contrast`；
  `workspace.background_margin` 设置候选包围框外背景环的宽度（默认 3 px）。
  灯和装甲板均使用该过滤，0 灰度差可关闭这一个门限。
- `workspace.min_snr=3` 要求 `对比度 / (背景标准差+4)` 至少为 3；0 关闭门限。
- `workspace.stability_delta=8`、`workspace.min_stability=0.5` 在候选附近检查相邻阈值，
  过滤易消失、碎裂、粘连或中心漂移的亮斑。任一设为 0 关闭检查。
  允许正常亮斑随阈值平滑缩小；低阈值会避开估计的背景噪声带。
- `armor.min_brightness_ratio=0.45` 要求两条灯条的背景对比度接近；
  `armor.min_gap_contrast=8` 要求条间九点均值比较暗灯条至少低 8 个灰度级。
  若目标中部本身很亮，可先降低 gap 门限并观察漏检情况。
  这些默认值需结合实拍标定；具体计算见 [`src/detector/README.md`](../../src/detector/README.md)。

Detect/Track 分工与原生应用一致：Detect 完整筛选并连续确认；Track 使用真实接收时间
估计速度、预测 ROI，并按位置/尺寸/对比度优先关联原目标。正常跟踪仅周期性对选中的
灯和两条灯条复查多阈值稳定性（`track.validation_interval=8`），质量降到最近复查值的
`track.quality_drop_ratio=0.65` 以下立即复查。SNR 和基本几何仍逐帧检查。
`track.association_k` / `track.association_min_px` 控制关联半径，`track.min_size_ratio`
和 `track.min_contrast_ratio` 控制允许的外观变化；`track.velocity_alpha` 是速度平滑系数，
`track.prediction_max_ms` 限制外推时间。装甲板确认使用 `armor.confirm_frames=2`。
原目标失配后，全图恢复扫描也不会立即切换身份；灯连续丢失达到门限后重新 Detect，
新灯条对在旧关联解除后重新确认。预测与旧目标都不会作为本帧测量输出。

预览中 `L` 为本帧确认的灯，圆心/灯心为绿色十字；`A` 为装甲板，板心为红色十字。
包围框使用相同颜色，最终目标的十字加大显示（装甲板优先，否则灯心），蓝框显示本帧搜索 ROI。
`LIGHT_COLOR` / `ARMOR_COLOR` / `ROI_COLOR` 可在脚本顶部改为其他 RGB 颜色。
`COLOR_PREVIEW=True` 复用独立的 RGB565 预览图，额外占用约 600 KiB；
灰度识别图仍为 Y8，标注不会写入识别输入。设为 `False` 可使用原灰度预览。
`DRAW_DETECTION` / `DRAW_ROI` 分别控制标注，`PREVIEW_ENABLED=False` 可只看终端结果。
标注在识别后绘制，预览图每帧完整刷新，亮像素统计和识别不读取标注内容。

终端每个 `STATUS_INTERVAL_MS` 输出灯/板中心、最终目标、相对图像中心的像素偏移、
候选数、丢失帧数和 ROI 重扫信息。新增 `contrast`、`SNR`、`stability`，装甲板另有
`brightness_ratio`、`gap_contrast`；`quality` 是这些指标构成的相对分数，不是正确率。
`gray rejects` 按顺序统计灰度差、SNR、稳定性拒绝数。若漏检，先看拒绝项，逐个降低
相应门限；若背景反光仍易误检，先适量提高 SNR 或稳定性下限，再检查连续确认帧数。
只看合格目标的质量分不能评估真实误识别率，需要保存正、负样本比较。
无目标时输出 `none`，不沿用历史坐标。
新增 `phase=detect/track`、`full_validation` 和 `stability_checks`，便于区分阶段耗时。
`stability_checked=False` 表示本帧按快速跟踪策略跳过多阈值复查，stability 此时为 0；
它不代表本帧检查失败。质量分使用本帧实际计算的指标。
相机超时不会推进识别帧数；`Camera.frame_info()` 提供原生帧号与单调接收时间，
保留 ROI 的间隔和顺序判断口径。识别异常触发原有资源清理并关闭 PWM。

启动时还会打印完整的 `detect_config.<字段> = <值>;`，可复制到原生
`src/vision/capture_pipeline.cpp` 中 `RoiDetector detector{detect_config}` 构造前。
`roi.max_gap` 会输出 `std::chrono::milliseconds{...}`。仅调整 Python 字典不会
自动修改原生应用默认值。这些识别结果不会驱动舵机闭环，PWM 继续保持各路标定零点。
当前相机只有亮度，无法区分绿色与其他颜色的同形亮目标。

本轮灰度修改需要重新构建并更新解释器，再完整重启；只更新 `.py` 无法改变旧解释器
内的二值采集路径。脚本检查 `_dart_raw.PIXEL_FORMAT == "gray8"`，遇到旧解释器会提示更新。
新增灰度特征检查还要求 `_dart_detect.GRAY_FEATURE_VERSION >= 1`。
Detect/Track 分工要求 `_dart_detect.TRACK_FEATURE_VERSION >= 1`。
本轮同样需要更新解释器并完整重启；构建、上传和完整重启步骤见下文。

| Python 参数 | RT-Smart 对应项 | 单位/语义 |
|---|---|---|
| `EXPOSURE_LINES` | `--exp` / `Controls::initial_exposure_lines` | 整行；0 保留初始化值，否则 4..65509 |
| `GRAY_THRESHOLD` | `--threshold` / `RoiDetector::Config::threshold` | 0..255；灰度候选下限，包含等号，不修改图像 |
| `DETECTOR_CONFIG["workspace.min_contrast"]` | `--min-contrast` | 候选均值减背景均值的下限，默认 12 个灰度级 |
| `MIRROR` | `--image-reverse` / `Controls::flip180` | CanMV 可设 0..3；dart_app 只有 180°：`on` = 3，`off` = 0 |
| `SERVO_INITIAL_US` | `--servo-initial` / `Channel::initial_pulse_us` | 四路机械零点的绝对脉宽，µs，允许小数 |
| `SERVO_MIN_US` / `SERVO_MAX_US` | `--servo-min` / `--servo-max` | µs；零点超出范围时直接报错 |
| `PWM_FREQUENCY_HZ` | `dart::pwm::kFrequencyHz` | 固定 333 Hz |
| `SERVO_ENABLED` | 调试时逐路选择 | 仅 Python 标定开关；原生应用启用全部四路 |

四路数组顺序固定为 CH1～CH4：

| CH | PWM | IO |
|---|---|---|
| 1 | 1 | 61 |
| 2 | 2 | 7 |
| 3 | 3 | 8 |
| 4 | 5 | 59 |

PWM0/1/2 共用周期，PWM3/4/5 共用周期，调试期间独占这两组。
PWM 使用 `duty_ns(round(us*1000))`，与原生后端的 µs→ns 换算一致。
脚本在配置频率、脉宽期间将输出脚保持为 GPIO 低电平，全部读回通过后接通 PWM。
停止、异常或达到运行时长时尝试逐路关闭 PWM；关闭波形不会切断舵机电源。

## 当前本机固件需要的相机接口

2026-10-24 的本机 Dart V1 固件只暴露了 SC038HGS 模式号和普通 `Sensor` 接口，
没有 Python 可调用的完整 RAW 采集接口。普通 `Sensor.snapshot()` 使用 ISP，
其灰度阈值不能直接移植到当前 `dart_app` 的线性灰度阈值；该模式还需要 ISP 场景文件。

这里提供 `_dart_raw.Camera`，直接编译本仓库现有的 C 相机后端：
`SC038HGS CSI2 / 640x480 → ONLY_MCM RAW10 packed10-lsb → 原生解包为灰度 Y8（RAW10 >> 2）`。
曝光复用 `k230/exposure.c`：Group 0 同步提交 `lines*16` 与
`VTS=max(600, lines+11)`，保留增益和保留位，并读回核对。
不使用曝光微秒换算、自动曝光、ISP 或缩放。

`Camera(EXPOSURE_LINES, GRAY_THRESHOLD, MIRROR)` 的第二个参数只控制亮像素统计。
`camera.snapshot_into(pixels, timeout_ms)` 将完整灰度图写入可写的 640×480 Y8 缓冲，
返回达到阈值的像素数，超时返回 `None`；返回 0 时仍是一张有效帧。
将同一缓冲与 `camera.frame_info()` 交给 `_dart_detect.Detector.detect()`，
识别使用保留的灰度值进行背景对比度检查。

构建配套解释器：

```sh
python3 tools/canmv_tuning/build.py
```

默认读取 `/mnt/mydata/kRTOSSDK` 和已完成的 Dart V1 MicroPython 构建，
输出 [`../../build/canmv-tuning/micropython`](../../build/canmv-tuning/micropython)、
脚本副本和 `BUILD_INFO.json`。可用 `--sdk`、`--toolchain-bin`、`--build-dir` 覆盖路径。
构建前校验原生后端要求的 VICAP 用户态/内核库哈希；SDK 源码和原固件产物均不改动。
新增适配器借用 CanMV 启动时已初始化的 VB；相机仍拥有并释放自己的 RAW 池。
不要同时创建普通 `Sensor` 或运行另一个相机应用。

此解释器针对当前本机固件配套的 RT-Smart 内核，无需重烧整盘镜像。
`Missing _dart_raw` 表示当前运行的解释器没有编入该模块；单独上传 `.py` 文件无效。
**IDE 的停止、重新运行和 `MPY: soft reboot` 都不会重新加载解释器 ELF。**

通过 MTP 暂存并校验新解释器（主机需安装 GIO，桌面已挂载板端存储）：

```sh
gio mount -l
python3 tools/canmv_tuning/stage.py gphoto2://Kendryte_CanMV_001000000/store_ffff0001
```

将 URI 替换为实际包含 `boot.py`、`main.py`、`micropython` 的存储根。
`stage.py` 将原解释器备份至 `build/canmv-tuning/board-backups/`，
上传到独立的 `.new` 文件，并对原文件备份、新文件进行完整回读 SHA-256 校验。
操作不会改变启动路径，结果写入 `build/canmv-tuning/STAGED.json`。

随后在 CanMV 中运行以下脚本，将 `stage` 和 `previous` 替换为
`STAGED.json` 中的 `stage`、`board_previous`：

```python
import os

stage = "/sdcard/micropython.dart-raw-时间戳.new"
previous = "/sdcard/micropython.before-dart-raw-时间戳"
os.rename("/sdcard/micropython", previous)
try:
    os.rename(stage, "/sdcard/micropython")
except Exception:
    os.rename(previous, "/sdcard/micropython")
    raise
print("Interpreter installed; hard reboot required")
```

这里仅重命名已经关闭并校验过的文件，保留板端原文件，避免原地覆盖正在运行的解释器。
然后单独执行 `import machine; machine.reset()`，或完整断电重启，重新连接 IDE 后验证：

```python
from _dart_raw import Camera, PIXEL_FORMAT
print(Camera)
print(PIXEL_FORMAT)
assert PIXEL_FORMAT == "gray8"
```

输出 `<class 'Camera'>` 和 `gray8` 后再运行 `canmv_tune.py`。首次检查可设置
`PWM_ENABLED=False`，确认相机预览后再启用舵机。解释器沿用 SDK 版本号，
启动横幅相同不能证明更新是否生效，应检查模块的 `PIXEL_FORMAT`。
识别版本还需验证 `from _dart_detect import Detector; print(Detector)`。

回退时将 `/sdcard/micropython` 改名保存，再将对应 `previous` 恢复为
`/sdcard/micropython`，完整重启即可。主机备份也可重新上传。

在更新解释器前，将 `CAMERA_ENABLED=False` 就能先在现有固件调试四路 PWM。
相机启用而缺少 `_dart_raw` 时会明确报错。

## 参数移植与验证范围

例如调好后控制台输出：

```text
--exp 400 --threshold 50 --min-contrast 12 --servo-initial 1520.000,1520.000,1520.000,1520.000 --servo-min 1420.000,1420.000,1420.000,1420.000 --servo-max 1620.000,1620.000,1620.000,1620.000
```

可追加到 `dart_app` 的启动参数；相同参数也可填入表中 C++ 配置字段。
`MIRROR` 通过 C++ 配置设置。Python 的预览速率不代表原生应用的采集/控制速率。

已验证 Python 语法、解释器交叉编译、相机后端 ASan/UBSan 回归以及脚本异常清理。
2026-10-25 的 Detect/Track 更新已通过 32 项主机回归、10 项 Python 测试和跟踪器
ASan/UBSan/LSan 检查，并完成原生应用、MicroPython 解释器交叉编译。
配套解释器已在本机 Dart V1 实板安装、完整重启，模块特征版本检查通过。
使用脚本当前全部识别参数，在 640×480 合成场景连续验证 130 帧：快速跟踪 112 帧
平均 109 µs、最大 149 µs，周期复查 16 帧平均 170 µs、最大 177 µs；
Detect 两次调用分别为 603 / 611 µs。已检查干扰物关联、质量下降立即复查、
丢失后重新确认，以及快速帧 0 次、周期复查帧 3 次多阈值检查。
真实相机另连续取 3 帧通过（曝光 120 行、阈值 60，PWM 关闭）；现场灰度范围
10..15，属于无目标暗场。记录与有界验证脚本在 `build/canmv-tuning/INSTALLED.json`
和 `build/canmv-tuning/detect-track-smoke.py`。
以上耗时包含 MicroPython 检测包装层，未计入采集、预览、PWM、遥测和录像；
未验证带真实目标的 RT-Smart 整链路持续 240 FPS，也未标定误识别率。
2026-10-24 已在本机连接的 Dart V1 实板更新解释器，并验证 `_dart_raw.Camera` 导入、
两次打开/各连续取 3 帧/关闭，取帧均成功（曝光 400 行、阈值 50，PWM 关闭）。
IDE 预览、曝光响应和连接器处的 PWM 波形仍需确认。
PWM API 与引脚配置依据本机 SDK 源码及[官方 PWM 手册](https://www.kendryte.com/k230_canmv/v0.6/zh/api/machine/k230_canmv_pwm_api_manual.html)。
