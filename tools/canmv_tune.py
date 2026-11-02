# CanMV VS Code：修改本区参数后停止、重新运行脚本。
# 相机使用与 dart_app 相同的 RAW10 C 后端，需要配套解释器，见 canmv_tuning/README.md。

# ---------------- 可调参数（量纲与 RT-Smart C/C++ 一致） ----------------
CAMERA_ENABLED = True
EXPOSURE_LINES = 120                 # --exp：整行；0 保留初始化曝光，否则 4..65509
GRAY_THRESHOLD = 60                  # --threshold：灰度候选下限 0..255；不改变预览/识别图
MIRROR = 0                           # CanMV 软件镜像 0..3；dart_app 对应 --image-reverse（仅 180° = 3）

DETECTION_ENABLED = True             # 复用 RT-Smart 的 RoiDetector；装甲板优先，否则灯心
# 键名即 RoiDetector::Config 字段路径；面积 px²，长度/坐标 px，角度 deg。
# _k 为原生倍率，比例 0..1，_pct 为原生百分数；roi.max_gap 为毫秒。
DETECTOR_CONFIG = {
    "workspace.min_contrast": 12.0,  # --min-contrast；候选均值减周围背景均值
    "workspace.background_margin": 3,
    "workspace.min_snr": 3.0,        # 对比度 / (背景标准差 + 4)；0 关闭此门限
    "workspace.stability_delta": 0,  # 候选附近复查 T±8；0 关闭稳定性检查
    "workspace.min_stability": 0.5,  # 0..1；容许平滑缩小，抑制碎裂/粘连/中心漂移
    "light.min_area": 3,
    "light.far_area_threshold": 30,
    "light.far_size_px": 8,
    "light.search_step": 2,
    "light.confirm_frames": 2,
    "light.far_confirm_frames": 3,
    "light.far_jump_k": 1.5,
    "light.far_jump_min_px": 6,
    "light.deadband_px": 2,
    "light.min_fill": 0.5,
    "light.min_circularity": 0.9,
    "light.circularity_weight": 5.0,
    "light.aspect_min": 0.7,
    "light.aspect_max": 1.4,
    "light.max_area_fraction": 0.25,
    "armor.length_floor": 2,
    "armor.area_floor": 2,
    "armor.length_min_k": 0.2,
    "armor.length_max_k": 8.0,
    "armor.tiny_px": 8.0,
    "armor.aspect_min": 2.0,
    "armor.fill_min": 0.5,
    "armor.flat_deg": 12.0,
    "armor.parallel_deg": 15.0,
    "armor.thickness_ratio": 0.3,
    "armor.length_ratio": 0.4,
    "armor.overlap": 0.4,
    "armor.gap_max_k": 10.0,
    "armor.diagonal_min_sin": 0.30,
    "armor.border_penalty": 0.0,
    "armor.window_width_k": 3.0,
    "armor.window_height_k": 2.0,
    "armor.window_gap_k": 0.0,
    "armor.window_min_side": 24.0,
    "armor.near_k": 8.0,
    "armor.near_min_px": 32.0,
    "armor.span_min_k": 0.2,
    "armor.span_max_k": 12.0,
    "armor.plate_up_lo_pct": 0,
    "armor.plate_up_hi_pct": 1200,
    "armor.plate_offx_pct": 600,
    "armor.association_k": 0.6,
    "armor.association_min_px": 4.0,
    "armor.track_hold": 2,
    "armor.confirm_frames": 2,           # 新灯条对须连续确认，不能继承旧装甲板锁定
    "armor.min_brightness_ratio": 0.0,  # 两条灯条的较小/较大对比度；0 关闭门限
    "armor.min_gap_contrast": 8.0,       # 较暗灯条均值 - 条间采样均值；0 允许相等
    "track.association_k": 1.5,          # 预测位置附近关联半径：max(最小像素, 倍率*尺寸)
    "track.association_min_px": 6.0,
    "track.min_size_ratio": 0.5,         # 宽、高各自允许 0.5..2 倍变化
    "track.min_contrast_ratio": 0.5,     # 当前/上次对比度的较小/较大比值下限
    "track.validation_interval": 8,      # 稳定跟踪每 8 帧复查选中目标的多阈值稳定性
    "track.quality_drop_ratio": 0.65,    # 对比度/SNR 下降到最近复查值的此比例时立即复查
    "track.velocity_alpha": 0.5,         # 速度平滑系数；使用真实接收时间，不使用帧号估算时间
    "track.prediction_max_ms": 50,       # 预测时间上限，避免丢帧后无限外推
    "roi.min_width": 24,
    "roi.min_height": 64,
    "roi.size_k": 2.6,
    "roi.margin": 12.0,
    "roi.grow_per_miss": 0.5,
    "roi.full_area_fraction": 0.75,
    "roi.reset_after_misses": 3,
    "roi.full_scan_interval": 0,
    "roi.max_gap": 250,
    "workspace.max_runs": 16384,
    "workspace.max_candidates": 256,
}
DRAW_DETECTION = True                # 预览叠加灯/板包围框与中心；不影响识别输入
DRAW_ROI = True                      # 绘制本帧搜索窗口
COLOR_PREVIEW = True                 # RGB565 预览；识别继续使用独立的灰度 Y8 图
LIGHT_COLOR = (0, 255, 0)             # 圆心/灯心：绿色，RGB 各分量 0..255
ARMOR_COLOR = (255, 0, 0)             # 装甲板中心：红色
ROI_COLOR = (0, 128, 255)             # 搜索窗口：蓝色
PRINT_DETECTOR_CPP_CONFIG = True      # 启动时输出可移植的 C++ 配置赋值

PWM_ENABLED = True
# 数组顺序固定为 CH1/PWM1/IO61、CH2/PWM2/IO7、CH3/PWM3/IO8、CH4/PWM5/IO59。
SERVO_INITIAL_US = (1525.0, 1520.0, 1520.0, 1520.0)  # --servo-initial：机械零点，µs
SERVO_MIN_US = (1420.0, 1420.0, 1420.0, 1420.0)      # --servo-min：校验下限，µs
SERVO_MAX_US = (1620.0, 1620.0, 1620.0, 1620.0)      # --servo-max：校验上限，µs
SERVO_ENABLED = (True, True, True, True)              # 可只启用正在标定的一路
PWM_FREQUENCY_HZ = 333                # 与 dart::pwm::kFrequencyHz 一致

PREVIEW_ENABLED = True               # 通过 IDE 帧缓冲窗口预览完整 640x480 灰度图
ACQUIRE_TIMEOUT_MS = 200             # --acquire-ms；0..1000 ms
MAX_CONSECUTIVE_TIMEOUTS = 8          # 连续无帧达到此数则停止，并关闭 PWM
STATUS_INTERVAL_MS = 1000            # 控制台输出 FPS / 达到灰度候选阈值的像素比例
RUN_SECONDS = 0                      # 0 持续运行；IDE 停止或 Ctrl+C 后清理资源
# ---------------- 以下通常无需修改 ----------------

import math
import os
import time

PWM_CHANNELS = (1, 2, 3, 5)
PWM_PINS = (61, 7, 8, 59)
WIDTH = 640
HEIGHT = 480


def _validate():
    if not CAMERA_ENABLED and not PWM_ENABLED:
        raise ValueError("Enable CAMERA_ENABLED or PWM_ENABLED")
    if not isinstance(EXPOSURE_LINES, int) or not (
        EXPOSURE_LINES == 0 or 4 <= EXPOSURE_LINES <= 65509
    ):
        raise ValueError("EXPOSURE_LINES: 0, or integer sensor lines 4..65509")
    if not isinstance(GRAY_THRESHOLD, int) or not 0 <= GRAY_THRESHOLD <= 255:
        raise ValueError("GRAY_THRESHOLD: integer 0..255")
    if not isinstance(MIRROR, int) or not 0 <= MIRROR <= 3:
        raise ValueError("MIRROR: 0..3")
    if PWM_FREQUENCY_HZ != 333:
        raise ValueError("RT-Smart PWM frequency is fixed at 333 Hz")
    for values in (SERVO_INITIAL_US, SERVO_MIN_US, SERVO_MAX_US, SERVO_ENABLED):
        if len(values) != 4:
            raise ValueError("Servo arrays must contain CH1..CH4 in order")
    for i in range(4):
        low, initial, high = SERVO_MIN_US[i], SERVO_INITIAL_US[i], SERVO_MAX_US[i]
        if not all(math.isfinite(v) for v in (low, initial, high)) or not (
            0 < low <= initial <= high <= 1000000.0 / PWM_FREQUENCY_HZ
        ):
            raise ValueError("CH%d: require 0 < min <= initial <= max <= period (us)" % (i + 1))
    if not isinstance(ACQUIRE_TIMEOUT_MS, int) or not 0 <= ACQUIRE_TIMEOUT_MS <= 1000:
        raise ValueError("ACQUIRE_TIMEOUT_MS: integer 0..1000")
    if MAX_CONSECUTIVE_TIMEOUTS < 1 or STATUS_INTERVAL_MS < 1 or RUN_SECONDS < 0:
        raise ValueError("Invalid timeout count, status interval, or duration")


def _stop_pwm(outputs):
    # 某一路失败仍继续关闭其它输出。deinit 关闭波形，不等于切断舵机供电。
    for output in outputs:
        try:
            output.deinit()
        except Exception as exc:
            print("PWM cleanup:", exc)


def _start_pwm(outputs):
    from machine import FPIOA, PWM, Pin

    fpioa = FPIOA()
    for i, channel in enumerate(PWM_CHANNELS):
        if not SERVO_ENABLED[i]:
            continue
        pin = PWM_PINS[i]
        # 不设置 msc，保留板级 IO 电压选择；启用输入/输出缓冲，与 C++ 后端一致。
        fpioa.set_function(pin, getattr(FPIOA, "PWM%d" % channel), ie=1, oe=1)
        output = PWM(channel)
        outputs.append(output)  # 后续任一步失败，finally 都会处理已创建的通道
        # 本机 CanMV freq()/duty_ns() 自动使能：配置期间先将物理引脚保持 GPIO 低。
        fpioa.set_function(pin, getattr(FPIOA, "GPIO%d" % pin), ie=1, oe=1)
        Pin(pin, Pin.OUT, value=0)
        output.freq(PWM_FREQUENCY_HZ)
    # PWM0/1/2、PWM3/4/5 分别共用周期；全部频率配置完再写各路零点。
    index = 0
    for i, channel in enumerate(PWM_CHANNELS):
        if not SERVO_ENABLED[i]:
            continue
        output = outputs[index]
        index += 1
        pulse_ns = int(SERVO_INITIAL_US[i] * 1000.0 + 0.5)  # 与 C++ llround(µs*1000) 一致
        output.duty_ns(pulse_ns)
        if output.freq() != PWM_FREQUENCY_HZ or abs(output.duty_ns() - pulse_ns) > 1000:
            raise RuntimeError("PWM%d configuration readback mismatch" % channel)
        print("CH%d: PWM%d IO%d %.3f us @ %d Hz" % (
            i + 1, channel, PWM_PINS[i], pulse_ns / 1000.0, PWM_FREQUENCY_HZ
        ))
    # 全部脉宽读回通过后，再把固定零点波形接到各路物理引脚。
    for i, channel in enumerate(PWM_CHANNELS):
        if SERVO_ENABLED[i]:
            fpioa.set_function(PWM_PINS[i], getattr(FPIOA, "PWM%d" % channel), ie=1, oe=1)


def _print_parameters():
    def csv(values):
        return ",".join("%.3f" % value for value in values)

    print("RT-Smart arguments (zero calibration; vision gains remain zero):")
    print("--exp %d --threshold %d --min-contrast %g --servo-initial %s --servo-min %s --servo-max %s" % (
        EXPOSURE_LINES, GRAY_THRESHOLD, DETECTOR_CONFIG.get("workspace.min_contrast", 12.0), csv(SERVO_INITIAL_US),
        csv(SERVO_MIN_US), csv(SERVO_MAX_US)
    ))
    print("dart_app: --image-reverse %s (Controls::flip180 = %s; app supports 180° only, MIRROR=3)" % (
        "on" if MIRROR == 3 else "off", "true" if MIRROR == 3 else "false"
    ))
    print("C++: PWM = %d Hz; grayscale candidate threshold = %d" % (
        PWM_FREQUENCY_HZ, GRAY_THRESHOLD
    ))
    if not all(SERVO_ENABLED):
        print("SERVO_ENABLED is only a calibration switch; dart_app enables all four channels")
    if CAMERA_ENABLED and DETECTION_ENABLED and PRINT_DETECTOR_CPP_CONFIG:
        print("C++: detection::RoiDetector::Config detect_config{};")
        print("detect_config.threshold = %d;" % GRAY_THRESHOLD)
        for key in sorted(DETECTOR_CONFIG):
            value = DETECTOR_CONFIG[key]
            if key == "roi.max_gap":
                text = "std::chrono::milliseconds{%d}" % value
            elif isinstance(value, int):
                text = str(value)
            else:
                text = "%.9g" % value
                if "." not in text and "e" not in text:
                    text += ".0"
                text += "F"
            print("detect_config.%s = %s;" % (key, text))


def _draw_detection(preview, result):
    # 彩色标注只写 RGB565 预览图；灰度图用于识别与亮像素统计。
    if DRAW_ROI:
        preview.draw_rectangle(result["roi"], color=ROI_COLOR if COLOR_PREVIEW else 80, thickness=1)
    if not DRAW_DETECTION:
        return
    for name, label, color in (
        ("light", "L", LIGHT_COLOR if COLOR_PREVIEW else 128),
        ("armor", "A", ARMOR_COLOR if COLOR_PREVIEW else 200),
    ):
        target = result[name]
        if target is None:
            continue
        preview.draw_rectangle(target["bounds"], color=color, thickness=2)
        cx, cy = target["center"]
        x, y = int(cx + 0.5), int(cy + 0.5)
        preview.draw_cross(x, y, color=0, size=10, thickness=4)
        preview.draw_cross(x, y, color=color, size=10, thickness=2)
        preview.draw_string(max(0, x + 5), max(0, y - 12), label, color=color)
    target = result["target"]
    if target is not None:
        cx, cy = target["center"]
        x, y = int(cx + 0.5), int(cy + 0.5)
        color = ARMOR_COLOR if result["kind"] == "armor" else LIGHT_COLOR
        if not COLOR_PREVIEW:
            color = 255
        preview.draw_cross(x, y, color=0, size=14, thickness=4)
        preview.draw_cross(x, y, color=color, size=14, thickness=2)
    preview.draw_string(4, 4, "target=%s candidates=%d" % (
        result["kind"], result["candidates"]), color=128)


def _print_detection(result):
    for name in ("light", "armor"):
        target = result[name]
        if target is None:
            print("%s=none" % name)
        else:
            cx, cy = target["center"]
            print("%s=(%.2f,%.2f) px area=%d px2 quality=%.2f clipped=%s" % (
                name, cx, cy, target["area"], target["quality"], target["clipped"]))
            print("  contrast=%.1f SNR=%.2f stability=%.2f checked=%s" % (
                target["contrast"], target["snr"], target["stability"], target["stability_checked"]))
            if name == "armor":
                print("  brightness_ratio=%.2f gap_contrast=%.1f" % (
                    target["brightness_ratio"], target["gap_contrast"]))
    target = result["target"]
    if target is None:
        print("target=none")
    else:
        cx, cy = target["center"]
        print("target=%s (%.2f,%.2f) px offset=(%.2f,%.2f) px" % (
            result["kind"], cx, cy, cx - WIDTH * 0.5, cy - HEIGHT * 0.5))
    print("ROI=%s full=%s rescan=%s candidates=%d misses=%d runs_high=%d" % (
        result["roi"], result["full_frame"], result["rescanned"],
        result["candidates"], result["missed_frames"], result["run_high_water"]))
    print("gray rejects: contrast=%d SNR=%d stability=%d" % (
        result["rejected_contrast"], result["rejected_snr"], result["rejected_stability"]))
    print("phase=%s full_validation=%s stability_checks=%d" % (
        result["phase"], result["full_validation"], result["stability_checks"]))


def main():
    _validate()
    camera = None
    detector = None
    display = None
    outputs = []
    try:
        if CAMERA_ENABLED:
            # 当前本机固件的普通 Sensor.snapshot() 经过 ISP，亮度响应与 RAW 直采不同。
            # 必须使用配套的 _dart_raw；缺失时给出明确操作提示。
            try:
                import _dart_raw
                Camera = _dart_raw.Camera
            except ImportError:
                raise RuntimeError(
                    "Missing _dart_raw: build with tools/canmv_tuning/build.py, "
                    "update /sdcard/micropython and HARD reboot (see README). "
                    "Set CAMERA_ENABLED=False to calibrate PWM on the current firmware."
                )
            if getattr(_dart_raw, "PIXEL_FORMAT", None) != "gray8":
                raise RuntimeError(
                    "Grayscale camera requires the updated tuning interpreter; "
                    "rebuild, update /sdcard/micropython and HARD reboot (see README)."
                )
            if DETECTION_ENABLED:
                try:
                    import _dart_detect
                    Detector = _dart_detect.Detector
                except ImportError:
                    raise RuntimeError(
                        "Missing _dart_detect: update /sdcard/micropython with the latest "
                        "tuning interpreter and HARD reboot; or set DETECTION_ENABLED=False."
                    )
                if getattr(_dart_detect, "GRAY_FEATURE_VERSION", 0) < 1:
                    raise RuntimeError(
                        "Grayscale feature checks require the latest tuning interpreter; "
                        "rebuild, update /sdcard/micropython and HARD reboot (see README)."
                    )
                if getattr(_dart_detect, "TRACK_FEATURE_VERSION", 0) < 1:
                    raise RuntimeError(
                        "Detect/track requires the latest tuning interpreter; "
                        "rebuild, update /sdcard/micropython and HARD reboot (see README)."
                    )
                config = dict(DETECTOR_CONFIG)
                config["threshold"] = GRAY_THRESHOLD
                detector = Detector(config)
            import image

            preview = image.Image(WIDTH, HEIGHT, image.GRAYSCALE, alloc=image.ALLOC_HEAP)
            pixels = preview.bytearray()  # 本机 Image 的可写共享内存，保留 preview 生命周期
            color_preview = None
            if PREVIEW_ENABLED:
                if COLOR_PREVIEW and detector is not None:
                    # 复用一张 RGB565 图，避免每帧转换时申请整图内存。
                    color_preview = image.Image(WIDTH, HEIGHT, image.RGB565, alloc=image.ALLOC_HEAP)
                from media.display import Display

                display = Display  # init 部分失败时也尝试 deinit
                display.init(Display.VIRT, width=WIDTH, height=HEIGHT, to_ide=True)
            camera = Camera(EXPOSURE_LINES, GRAY_THRESHOLD, MIRROR)
            print("SC038HGS CSI2: 640x480 RAW10 packed10-lsb -> grayscale Y8; no ISP")
        _print_parameters()
        if PWM_ENABLED:
            _start_pwm(outputs)

        start = time.ticks_ms()
        report = start
        frames = 0
        timeouts = 0
        last_bright = None
        last_detection = None
        while True:
            os.exitpoint()
            if RUN_SECONDS and time.ticks_diff(time.ticks_ms(), start) >= RUN_SECONDS * 1000:
                break
            if camera is not None:
                # C 后端在返回前已归还 RAW DMA 缓冲；None 表示本次超时，0 也表示一张有效灰度图（无像素达到阈值）。
                bright = camera.snapshot_into(pixels, ACQUIRE_TIMEOUT_MS)
                if bright is None:
                    timeouts += 1
                    if timeouts >= MAX_CONSECUTIVE_TIMEOUTS:
                        raise RuntimeError("Camera consecutive frame timeouts: %d" % timeouts)
                    continue
                timeouts = 0
                last_bright = bright
                frames += 1
                if detector is not None:
                    last_detection = detector.detect(pixels, camera.frame_info())
                if PREVIEW_ENABLED:
                    rendered = preview
                    if color_preview is not None:
                        color_preview.draw_image(preview, 0, 0)
                        rendered = color_preview
                    if last_detection is not None:
                        _draw_detection(rendered, last_detection)
                    display.show_image(rendered)
            else:
                time.sleep_ms(10)  # PWM 由硬件持续输出固定零点
            now = time.ticks_ms()
            elapsed = time.ticks_diff(now, report)
            if elapsed >= STATUS_INTERVAL_MS:
                if last_bright is None:
                    print("PWM holding calibrated initial pulses")
                else:
                    print("%.1f FPS, bright %.2f%%, exp=%d lines, threshold=%d" % (
                        frames * 1000.0 / elapsed, last_bright * 100.0 / (WIDTH * HEIGHT),
                        EXPOSURE_LINES, GRAY_THRESHOLD
                    ))
                    if last_detection is not None:
                        _print_detection(last_detection)
                report = now
                frames = 0
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        _stop_pwm(outputs)
        if detector is not None:
            try:
                detector.close()
            except Exception as exc:
                print("Detector cleanup:", exc)
        if camera is not None:
            try:
                camera.close()
            except Exception as exc:
                print("Camera cleanup:", exc)
        if display is not None:
            try:
                display.deinit()
            except Exception as exc:
                print("Display cleanup:", exc)


if __name__ == "__main__":
    main()
