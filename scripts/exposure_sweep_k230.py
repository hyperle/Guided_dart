"""曝光扫描/标定工具 (CanMV K230, GC2093)。

用途:
  1) 验证预览显示的就是板端帧: 每帧在左上角叠印 "exp=... lum=...",
     文字清晰可见 → 预览=我们的帧; 看不见 → 插件显示的是另一路画面。
  2) 找"灯清楚、背景暗"的曝光档位: 依次显示 SWEEP_US 里的每档 1.2 秒。
  3) 验证曝光是否真生效: 板端 lum 随 exp 大幅变化 = 生效;
     lum 几乎不变 = 自动增益(AGC)在补偿, 此时打开 FIX_GAIN 固定增益。

用法: 灯放画面里跑, 看预览文字与亮度, 终端每档打一行 "exp=.. lum=.."。
把合适的曝光值填回 green_led_test_k230.py 的 EXPOSURE_US。
"""

import time

from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

FRAME_W, FRAME_H = 640, 480
SWEEP_US = (200, 400, 800, 1600, 3200, 6400, 11700)  # 依次试的曝光(us), 可按需改
HOLD_MS = 1200               # 每档停留时间(ms), 给 3A/增益留 settling 时间
SETTLE_MS = 300              # 切档后先等这么久再采样
FIX_GAIN = True              # 尝试把模拟增益固定到最低, 防止 AGC 补偿亮度
LUM_STEP = 16                # 平均亮度采样步长


def rgb_at(img, x, y):
    p = img.get_pixel(x, y)
    if isinstance(p, int):
        r5 = (p >> 11) & 0x1F
        g6 = (p >> 5) & 0x3F
        b5 = p & 0x1F
        return ((r5 << 3) | (r5 >> 2),
                (g6 << 2) | (g6 >> 4),
                (b5 << 3) | (b5 >> 2))
    return p[0], p[1], p[2]


def sample_lum(img, step=LUM_STEP):
    total = 0
    n = 0
    for y in range(0, img.height(), step):
        for x in range(0, img.width(), step):
            r, g, b = rgb_at(img, x, y)
            lum = r
            if g > lum:
                lum = g
            if b > lum:
                lum = b
            total += lum
            n += 1
    return total / n if n else 0.0


def main():
    sensor = None
    try:
        sensor = Sensor(width=FRAME_W, height=FRAME_H)
        sensor.reset()
        sensor.set_framesize(width=FRAME_W, height=FRAME_H)
        sensor.set_pixformat(Sensor.RGB565)

        Display.init(Display.VIRT, width=FRAME_W, height=FRAME_H, to_ide=True)
        MediaManager.init()
        sensor.run()

        # 关自动曝光(属性式; 该固件支持)
        try:
            sensor.auto_exposure = False
            print("auto-exposure off (auto_exposure=False)")
        except Exception as exc:
            print("auto_exposure=False failed: %s" % exc)

        # 固定模拟增益到最低, 防止 AGC 把亮度补偿回来
        if FIX_GAIN:
            rng = None
            f = getattr(sensor, "get_again_range", None)
            if f is not None:
                try:
                    rng = f()
                    print("again range: %s" % (rng,))
                except Exception as exc:
                    print("get_again_range failed: %s" % exc)
            if rng and len(rng) == 2:
                lo, hi = sorted(rng)
                try:
                    sensor.again = lo
                    print("gain fixed: sensor.again = %s (最低)" % lo)
                except Exception as exc:
                    print("sensor.again = %s failed: %s" % (lo, exc))

        print("sweep start. 看预览文字与亮度, 终端每档打印 exp/lum")
        while True:
            for us in SWEEP_US:
                try:
                    sensor.exposure = us
                except Exception as exc:
                    print("set exp=%d failed: %s" % (us, exc))
                    continue
                time.sleep_ms(SETTLE_MS)
                t_end = time.ticks_ms() + HOLD_MS
                lum = 0.0
                while time.ticks_ms() < t_end:
                    img = sensor.snapshot()
                    lum = sample_lum(img, LUM_STEP)
                    try:
                        img.draw_string(8, 8,
                                        "exp=%dus lum=%.0f" % (us, lum),
                                        color=(255, 255, 0))
                    except Exception:
                        pass
                    Display.show_image(img)
                print("exp=%d lum=%.0f" % (us, lum))
    except KeyboardInterrupt:
        print("sweep stopped")
    except Exception as exc:
        print("sweep error: %s" % exc)
    finally:
        try:
            if sensor is not None:
                sensor.stop()
        except Exception:
            pass
        try:
            Display.deinit()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass


if __name__ == "__main__":
    main()
