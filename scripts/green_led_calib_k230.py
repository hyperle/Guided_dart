"""Green-LED 曝光/颜色标定工具 (CanMV K230)。

和帧率测试脚本分开: 这个脚本不管帧率, 只回答两件事:
  1) 曝光对不对 (背景应该暗、只有灯是亮点);
  2) 灯的像素"绿得有多纯" (g - max(r,b) 越大越好, 噪声通常很小)。

用法:
  1. 把绿灯放进画面中央, 跑本脚本。预览里红十字会压在"全图最绿像素"上,
     先确认它压在灯心上, 而不是背景上。
  2. 终端每 PRINT_EVERY 帧打印一次:
       avg_lum  : 全图平均亮度(0~255)。背景暗(几十以下)才是对的;
                  如果 120+ 说明曝光/增益太高, 噪声会把你淹没。
       dark/bright: 暗像素/亮像素占比, 辅助判断背景与过曝。
       green_px~: 绿度 > GREEN_GATE 的像素数(粗略, 可当灯的大小看)。
       top=(x,y) green=NN rgb=(r,g,b): 最绿像素的位置/绿度/原始颜色。
  3. 把 top 的 rgb、avg_lum、ctrl 列表发我, 我据此算 LAB 阈值给你填 TH_GREEN。

曝光/白平衡: 固件版本不同, 控制接口名也不同。脚本启动时会打印
  "exposure/awb controls found: [...]", 并尝试自动关掉 AWB(失败不影响标定)。
"""

import time

from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager

FRAME_W, FRAME_H = 640, 480
ROI = (0, 0, FRAME_W, FRAME_H)
SCAN_STEP = 8        # 粗扫步长(标定不需要实时, 大一点更快)
REFINE_R = 24        # 粗扫命中点周围细扫半径(拿灯的准确像素)
GREEN_GATE = 40      # 绿度门: g - max(r,b), 噪声一般 < 30, 真绿灯应 > 60
PRINT_EVERY = 15
_SENSOR_KEY = ("expo", "gain", "ae", "awb", "wb", "white", "3a", "manual")


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


def green_of(r, g, b):
    """绿度: 绿色通道超出红/蓝通道多少。自发光绿 LED 通常 > 60~150。"""
    return g - (r if r > b else b)


def probe(img):
    """粗扫全图统计 + 在最绿点附近细扫, 返回
    (avg_lum, dark_cnt, bright_cnt, sample_n, green_px, best)"""
    x0, y0, x1, y1 = ROI[0], ROI[1], ROI[0] + ROI[2], ROI[1] + ROI[3]
    n_green = 0
    s_lum = 0
    n = 0
    dark = 0
    bright = 0
    best = None
    for y in range(y0, y1, SCAN_STEP):
        for x in range(x0, x1, SCAN_STEP):
            r, g, b = rgb_at(img, x, y)
            lum = r
            if g > lum:
                lum = g
            if b > lum:
                lum = b
            s_lum += lum
            n += 1
            if lum < 64:
                dark += 1
            elif lum > 192:
                bright += 1
            gr = green_of(r, g, b)
            if gr > GREEN_GATE:
                n_green += 1
            if best is None or gr > best[0]:
                best = (gr, x, y)
    avg = s_lum / n if n else 0.0

    # 细扫: 在最绿点附近逐像素找真正的峰(粗扫可能跳过了灯心)
    if best is not None:
        xr0 = max(x0, best[1] - REFINE_R)
        yr0 = max(y0, best[2] - REFINE_R)
        xr1 = min(x1, best[1] + REFINE_R)
        yr1 = min(y1, best[2] + REFINE_R)
        for y in range(yr0, yr1):
            for x in range(xr0, xr1):
                r, g, b = rgb_at(img, x, y)
                gr = green_of(r, g, b)
                if gr > best[0]:
                    best = (gr, x, y)
    return avg, dark, bright, n, n_green, best


def main():
    sensor = None
    frame = 0
    try:
        sensor = Sensor(width=FRAME_W, height=FRAME_H)
        sensor.reset()
        sensor.set_framesize(width=FRAME_W, height=FRAME_H)
        sensor.set_pixformat(Sensor.RGB565)

        Display.init(Display.VIRT, width=FRAME_W, height=FRAME_H, to_ide=True)
        MediaManager.init()

        # 探测固件给了哪些曝光/白平衡控制(不同固件差别大)
        ctrl = sorted(n for n in dir(sensor)
                      if any(k in n.lower() for k in _SENSOR_KEY))
        print("exposure/awb controls found: %s" % ctrl)

        # 尝试关自动白平衡(名字随固件; 没有就跳过, 不影响标定)
        for name in ("set_auto_whitebal", "set_auto_awb", "set_awb_enable"):
            f = getattr(sensor, name, None)
            if f is None:
                continue
            try:
                f(False)
                print("AWB off via %s" % name)
            except Exception as exc:
                print("AWB %s failed: %s" % (name, exc))

        sensor.run()
        print("calib start. 把绿灯放进画面, 确认红十字压在灯心上")
        while True:
            img = sensor.snapshot()
            avg, dark, bright, n, n_green, best = probe(img)
            frame += 1
            if best is not None:
                img.draw_cross(best[1], best[2], color=(255, 0, 0), size=7)

            if frame % PRINT_EVERY == 0:
                if best is not None:
                    r, g, b = rgb_at(img, best[1], best[2])
                    print("avg_lum=%.0f dark=%d%% bright=%d%% green_px~%d "
                          "top=(%d,%d) green=%d rgb=(%d,%d,%d)" %
                          (avg, dark * 100 // n, bright * 100 // n, n_green,
                           best[1], best[2], best[0], r, g, b))
                else:
                    print("no green px (gate=%d): avg_lum=%.0f dark=%d%% "
                          "bright=%d%%" %
                          (GREEN_GATE, avg, dark * 100 // n, bright * 100 // n))
                if frame % (PRINT_EVERY * 8) == 0:
                    print(">>> 检查: 1) avg_lum 高且 bright 高 => 背景太亮, "
                          "降曝光/增益; 2) 十字没压灯 => 有更绿的干扰或灯太暗; "
                          "3) 把 top rgb 发我, 我给你 LAB 阈值")

            Display.show_image(img)
    except KeyboardInterrupt:
        print("calib stopped")
    except Exception as exc:
        print("calib error: %s" % exc)
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
