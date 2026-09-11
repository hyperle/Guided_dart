"""Green-LED detection and blob-center test for CanMV K230.

预览面板会画: 黄框 = 候选 blob; 红十字 = 最终 blob 的几何中心;
蓝圈 = 时间滤波后的稳定圆心。
终端每 PRINT_EVERY 帧打印一次 blob 数、中心、灯心像素 RGB 和 FPS。

参数怎么调:
  - 想直接跑: 打开本文件, 改"可调参数"区即可, 点三角按钮启动。
  - 先看预览里黄框/十字是否压在灯上(绿灯自发光, 背景应没有绿色目标)。
  - TH_GREEN 默认值来自实测换算: 绿 LED 各亮度档 A=-29~-82、B=+22~+75;
    暗场灰绿噪声 A≈-9、B≈-2(之前 n=几百 的假目标就是它)。
    还误检 → Amax 改 -25; 检不到 → Bmin 改 0 或 Lmin 改 8。
  - 确认能稳定锁定后再收窄 ROI、再谈帧率。
"""

import time

from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

# ---------------- 可调参数 (直接改这里) ----------------
# 成像
FRAME_W, FRAME_H = 640, 480     
EXPOSURE_US = 200              # 曝光时间(微秒 us)。None = 不固定曝光。
                                
FIX_GAIN = True                 # 固定模拟增益到最低, 防 AGC 把亮度补偿回来(该固件关不掉自动增益)
ROI = (160, 120, 320, 240)  # 灯的活动区域; 收窄 = 提速 + 降误检
                                # 例: 灯只在画面中心 → ROI = (160, 120, 320, 240)

# 运行模式
DEBUG = False                    # False = 关闭预览和图像调试绘制
PRINT_STATS = True              # 独立保留 FPS/snapshot 采集统计
PREVIEW_EVERY = 300              # 调试预览帧间隔

# 自适应 ROI
ADAPTIVE_ROI = True
TRACK_MARGIN = 80
ROI_MISS_LIMIT = 3
FULL_SCAN_EVERY = 30             # 周期性全 ROI 重定位
BLOB_STRIDE = (3, 2)              # (横向步长, 竖向步长); (1,1)=逐像素
_BLOB_STRIDE_SUPPORTED = None
BASE_ROI = tuple(int(v) for v in ROI)

# 颜色阈值 LAB: (Lmin, Lmax, Amin, Amax, Bmin, Bmax)
# 默认值依据实测换算: 绿 LED 各亮度档 A=-29~-82, B=+22~+75;
# 暗场灰绿噪声 A≈-9, B≈-2。A上限-20 和 B下限8 两个条件一起卡:
# 噪声进不来(A不够负、B为负), 暗灯也不漏(A 还能到 -29)。
TH_GREEN = (12, 100, -128, -20, 8, 100)
TH_GREEN_LIST = [TH_GREEN]
# 还误检: Amax 改 -25; 检不到: Bmin 改 0 或 Lmin 改 8。

# blob 筛选
PIX_MIN = 50                  # blob 最小像素数
FILL_LO, FILL_HI = 0.55, 1.0    # 填充率判圆: 实心圆盘≈0.785
ASPECT_LO, ASPECT_HI = 0.7, 1.4 # 长宽比判圆
FILL_LO_PCT = int(FILL_LO * 100)
ASPECT_LO_NUM = int(ASPECT_LO * 10)
ASPECT_HI_NUM = int(ASPECT_HI * 10)
# 圆心与滤波
USE_FILTER = False              # True=时间滤波(中位数, 稳但延迟约 HIST_LEN/2 帧)
HIST_LEN = 9                    # 滤波窗口; 要低延迟就改小(如 5)
PRINT_EVERY = 30                # 每多少帧打印一次统计
PROBE_EVERY = 300               # lost 时每多少帧探测一次"最绿像素"(探针慢, 别太频繁)
TIME_STAGES = True              # True=前 TIME_SHOW 帧逐帧打印各阶段耗时(测帧率/找瓶颈时开)
TIME_SHOW = 60

TARGET_FPS = 90              # 请求上限；固件会选择传感器支持的最佳档位
SENSOR_FPS = TARGET_FPS

def rgb_at(img, x, y):
    p = img.get_pixel(x, y)
    if isinstance(p, tuple):
        return p[0], p[1], p[2]
    return ((p >> 8 & 0xF8) | (p >> 13 & 0x07),
            (p >> 3 & 0xFC) | (p >> 9 & 0x03),
            (p << 3 & 0xF8) | (p >> 2 & 0x07))


def find_best_blob(img, roi):
    global _BLOB_STRIDE_SUPPORTED
    kwargs = dict(roi=roi, pixels_threshold=PIX_MIN,
                  area_threshold=PIX_MIN, merge=False)
    if _BLOB_STRIDE_SUPPORTED is not False:
        kwargs["x_stride"] = int(BLOB_STRIDE[0])
        kwargs["y_stride"] = int(BLOB_STRIDE[1])
    try:
        blobs = img.find_blobs(TH_GREEN_LIST, **kwargs)
        _BLOB_STRIDE_SUPPORTED = True
    except TypeError:
        # 某些旧固件没有 x_stride/y_stride，退回逐像素搜索。
        _BLOB_STRIDE_SUPPORTED = False
        kwargs.pop("x_stride", None)
        kwargs.pop("y_stride", None)
        blobs = img.find_blobs(TH_GREEN_LIST, **kwargs)
    best = None
    best_px = -1
    for b in blobs:
        x, y, w, h, px = b.x(), b.y(), b.w(), b.h(), b.pixels()
        if w < 2 or h < 2:
            continue
        if px < PIX_MIN:
            continue
        if (px * 100 >= FILL_LO_PCT * w * h and
                10 * w >= ASPECT_LO_NUM * h and
                10 * w <= ASPECT_HI_NUM * h):
            if px > best_px:
                best = b
                best_px = px
    return best, blobs


def clamp_roi(roi, width, height):
    x, y, w, h = [int(v) for v in roi]
    x = max(0, min(x, width - 1))
    y = max(0, min(y, height - 1))
    w = max(1, min(w, width - x))
    h = max(1, min(h, height - y))
    return (x, y, w, h)


def tracked_roi(best, img, previous_center=None):
    cx, cy = best.cx(), best.cy()
    if previous_center is not None:
        cx += cx - previous_center[0]
        cy += cy - previous_center[1]
    return clamp_roi((cx - best.w() // 2 - TRACK_MARGIN,
                      cy - best.h() // 2 - TRACK_MARGIN,
                      best.w() + 2 * TRACK_MARGIN,
                      best.h() + 2 * TRACK_MARGIN),
                     img.width(), img.height())


def med(values):
    s = sorted(values)
    n = len(s)
    if n % 2:
        return s[n // 2]
    return (s[n // 2 - 1] + s[n // 2]) * 0.5


def apply_exposure(sensor):
    """固定曝光与增益(去掉自动曝光/自动增益控制, 防 AGC 补偿)。

    K230 CanMV: 运行后设置曝光/增益使用方法式接口。
    自动曝光必须在 sensor.run() 之前关闭（见 main）。
    """
    # 固定增益到最低(防 AGC 补偿; 自动增益接口该固件不支持)
    if FIX_GAIN:
        try:
            rng = sensor.get_again_range()
            if isinstance(rng, dict):
                lo = rng["min"]
            else:
                lo = min(rng)
            sensor.again(lo)
            print("gain fixed to %s (lowest)" % (lo,))
        except Exception as exc:
            print("gain fix failed: %s" % exc)

    # 固定曝光(有量程就钳位; 注意该固件 get_exposure_time_range() 返回 (上限, 下限))
    if EXPOSURE_US is None:
        return
    us = EXPOSURE_US
    try:
        rng = sensor.get_exposure_time_range()
        lo, hi = sorted(rng)
        us = max(lo, min(us, hi))
        if us != EXPOSURE_US:
            print("clamped %d -> %d us" % (EXPOSURE_US, us))
        sensor.exposure(us)
        print("exposure set: %d us (readback %s)" % (us, sensor.exposure()))
    except Exception as exc:
        print("exposure set failed: %s" % exc)


def configure_framerate(sensor):
    getter = getattr(sensor, "get_framerate", None)
    if DEBUG and callable(getter):
        try:
            print("sensor frame rate: %s fps" % (getter(),))
        except Exception as exc:
            print("frame rate read failed: %s" % exc)
    elif DEBUG:
        print("sensor fps requested at construction: %s" % (SENSOR_FPS,))

def probe_greenest(img, step=4):
    """lost 时用: 粗扫全图找"最绿像素"(g - max(r,b) 最大), 返回
    (green, x, y, r, g, b) 或 None。用于判断是灯没进阈值还是没入画。"""
    best = None
    for y in range(0, img.height(), step):
        for x in range(0, img.width(), step):
            r, g, b = rgb_at(img, x, y)
            gr = g - (r if r > b else b)
            if best is None or gr > best[0]:
                best = (gr, x, y, r, g, b)
    return best


def sample_lum(img, step=16):
    """粗略平均亮度(0~255), 板端数字验证曝光是否真的变了(不依赖预览画面)。"""
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
    hist = []          # 时间滤波: 最近 N 个有效加权圆心
    frame = 0
    miss = 0
    search_roi = BASE_ROI
    previous_center = None
    try:
        try:
            sensor = Sensor(width=FRAME_W, height=FRAME_H, fps=SENSOR_FPS)
        except TypeError:
            # 兼容不接受 fps 参数的旧固件。
            sensor = Sensor(width=FRAME_W, height=FRAME_H)
        sensor.reset()
        sensor.set_framesize(width=FRAME_W, height=FRAME_H)
        sensor.set_pixformat(Sensor.RGB565)

        if DEBUG:
            Display.init(Display.VIRT, width=FRAME_W, height=FRAME_H, to_ide=True)
        MediaManager.init()

        # K230 要求在 run() 之前关闭自动曝光，之后才能手动调曝光。
        try:
            sensor.auto_exposure(False)
            print("AE off")
        except Exception as exc:
            print("AE off failed: %s" % exc)

        configure_framerate(sensor)
        sensor.run()

        apply_exposure(sensor)

        if DEBUG:
            print("green-led test start. TH_GREEN=%s ROI=%s" % (TH_GREEN, BASE_ROI))
        acc_proc = 0          # 一个统计窗口内的相机检测耗时(snapshot+find, 不含发IDE)
        snap_sum = 0
        snap_min = None
        snap_max = 0
        frame_gap_sum = 0
        frame_gap_min = None
        frame_gap_max = 0
        prev_snap_end = None
        t_win0 = time.ticks_us()
        while True:
            snap_start = time.ticks_us()
            img = sensor.snapshot()
            snap_end = time.ticks_us()
            snap_dt = time.ticks_diff(snap_end, snap_start)
            snap_sum += snap_dt
            snap_min = snap_dt if snap_min is None or snap_dt < snap_min else snap_min
            snap_max = max(snap_max, snap_dt)
            if prev_snap_end is not None:
                frame_gap = time.ticks_diff(snap_end, prev_snap_end)
                frame_gap_sum += frame_gap
                frame_gap_min = frame_gap if frame_gap_min is None or frame_gap < frame_gap_min else frame_gap
                frame_gap_max = max(frame_gap_max, frame_gap)
            prev_snap_end = snap_end
            if DEBUG or PRINT_STATS:
                ta = snap_start
                tb = snap_end
            if ADAPTIVE_ROI and FULL_SCAN_EVERY and frame % FULL_SCAN_EVERY == 0:
                scan_roi = BASE_ROI
            else:
                scan_roi = search_roi
            best, blobs = find_best_blob(img, scan_roi)
            if DEBUG or PRINT_STATS:
                tc = time.ticks_us()
            c = None
            ok = False

            if best is not None:
                bx = best.x()
                by = best.y()
                bw = best.w()
                bh = best.h()
                if DEBUG:
                    img.draw_rectangle(bx, by, bw, bh, color=(255, 255, 0), thickness=1)

                # 使用 K230 原生 blob 几何中心，并在预览中保留中心十字。
                c = (best.cx(), best.cy())
                if DEBUG:
                    img.draw_cross(best.cx(), best.cy(), color=(255, 0, 0), size=5)

                if USE_FILTER:
                    hist.append(c)
                    if len(hist) > HIST_LEN:
                        del hist[0]
                ok = True
                miss = 0
                if ADAPTIVE_ROI:
                    search_roi = tracked_roi(best, img, previous_center)
                    previous_center = (best.cx(), best.cy())
            else:
                miss += 1
                if ADAPTIVE_ROI and miss >= ROI_MISS_LIMIT:
                    search_roi = BASE_ROI
                    previous_center = None
                if DEBUG and miss % PROBE_EVERY == 1:
                    # 周期探测"最绿像素"(粗步长, 避免卡帧), 判断阈值太严还是灯没入画
                    p = probe_greenest(img, 8)
                    if p is not None:
                        img.draw_cross(p[1], p[2], color=(255, 0, 255), size=7)
                        print("no-blob probe: top green@(%d,%d) green=%d "
                              "rgb=(%d,%d,%d)  (发我, 我据此调 TH_GREEN)" %
                              (p[1], p[2], p[0], p[3], p[4], p[5]))

            if USE_FILTER:
                if ok and len(hist) >= 3:
                    fc = (med([p[0] for p in hist]), med([p[1] for p in hist]))
                    if DEBUG:
                        img.draw_circle(int(fc[0] + 0.5), int(fc[1] + 0.5), 3,
                                        color=(0, 0, 255), thickness=2)
                else:
                    fc = None
            else:
                fc = None        # 时间滤波已禁用, 无平滑输出

            if DEBUG or PRINT_STATS:
                td = time.ticks_us()
                acc_proc += td - ta          # 累计本帧检测耗时(检测吞吐口径)
            frame += 1
            if PRINT_STATS and frame % PRINT_EVERY == 0:
                now = time.ticks_us()
                det_hz = PRINT_EVERY * 1e6 / acc_proc         # 每秒检测次数(不含发IDE)
                loop_hz = PRINT_EVERY * 1e6 / (now - t_win0)  # 实际循环速率(含发IDE)
                snap_avg = snap_sum / PRINT_EVERY
                gap_n = PRINT_EVERY - 1
                gap_avg = frame_gap_sum / gap_n if gap_n > 0 else 0
                print("capture: snap avg/min/max=%.2f/%.2f/%.2f ms, "
                      "frame gap avg/min/max=%.2f/%.2f/%.2f ms (%.1f Hz)" %
                      (snap_avg / 1000, snap_min / 1000, snap_max / 1000,
                       gap_avg / 1000, frame_gap_min / 1000, frame_gap_max / 1000,
                       1e6 / gap_avg if gap_avg else 0))
                snap_sum = 0
                snap_min = None
                snap_max = 0
                frame_gap_sum = 0
                frame_gap_min = None
                frame_gap_max = 0
                acc_proc = 0
                t_win0 = now
                avg_lum = sample_lum(img, 16)
                if ok:
                    r, g, b = rgb_at(img, int(c[0] + 0.5), int(c[1] + 0.5))
                    print("n=%d center=(%d,%d) fc=%s rgb=(%d,%d,%d) lum=%.0f "
                          "det=%.1fHz loop=%.1fHz" %
                          (len(blobs), best.cx(), best.cy(),
                           fc and ("(%.2f,%.2f)" % fc) or "None",
                           r, g, b, avg_lum, det_hz, loop_hz))
                else:
                    print("lost (miss=%d) lum=%.0f det=%.1fHz loop=%.1fHz" %
                          (miss, avg_lum, det_hz, loop_hz))

            if DEBUG and PREVIEW_EVERY > 0 and frame % PREVIEW_EVERY == 0:
                Display.show_image(img)
            if DEBUG and TIME_STAGES and frame < TIME_SHOW:
                tg = time.ticks_us()
                print("stages: snap=%dus find=%dus proc=%dus show=%dus" %
                      (tb - ta, tc - tb, td - tc, tg - td))
    except KeyboardInterrupt:
        print("green-led test stopped")
    except Exception as exc:
        print("green-led test error: %s" % exc)
    finally:
        try:
            if sensor is not None:
                sensor.stop()
        except Exception:
            pass
        try:
            if DEBUG:
                Display.deinit()
        except Exception:
            pass
        try:
            MediaManager.deinit()
        except Exception:
            pass


if __name__ == "__main__":
    main()
