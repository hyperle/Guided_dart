#!/usr/bin/env python3
"""usb_cam_viewer —— 主机端接收 usb_cam_stream 通过 USB(CDC ACM) 发来的 JPEG 帧并显示。

板端 app: apps/usb_cam_stream（VICAP -> VENC JPEG -> /dev/ttyGS0）
帧协议:
    [0..3]  magic 'L','C','J','P'
    [4..7]  JPEG 长度 (uint32 LE)
    [8..11] 图像宽   (uint32 LE)
    [12..15]图像高   (uint32 LE)
    [16..]  JPEG 数据

用法:
    python3 scripts/usb_cam_viewer.py                 # 自动找 K230 的 ACM 口
    python3 scripts/usb_cam_viewer.py --port /dev/ttyACM0
    python3 scripts/usb_cam_viewer.py --save out/     # 不弹窗, 存帧
    python3 scripts/usb_cam_viewer.py --list          # 列出候选串口

依赖: pip install pyserial opencv-python numpy
"""

import argparse
import os
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺少 pyserial: pip install pyserial")

import cv2
import numpy as np

MAGIC = b"LCJP"
HEADER_LEN = 16
# K230 RT-Smart 固件 USB 描述符默认 VID/PID (见内核 .config: VID=0x1209 PID=0xABD1)
K230_VID = 0x1209
K230_PID = 0xABD1


def find_ports():
    found = []
    for p in list_ports.comports():
        if p.vid == K230_VID and p.pid == K230_PID:
            found.append(p.device)
    # 兜底: 名称含 ACM 且厂商串里含 CanMV/Kendryte 的口
    for p in list_ports.comports():
        if p.device not in found and "ACM" in p.device.upper():
            info = (p.manufacturer or "") + (p.description or "")
            if any(k in info.lower() for k in ("k230", "canmv", "kendryte", "canaan")):
                found.append(p.device)
    return found


def read_exact(ser, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def main():
    ap = argparse.ArgumentParser(description="K230 RT-Smart USB 摄像头收流显示")
    ap.add_argument("--port", default=None, help="串口, 如 /dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=115200, help="CDC ACM 无实际波特率, 默认即可")
    ap.add_argument("--save", default=None, help="保存目录(不弹窗显示), 每帧存 jpg")
    ap.add_argument("--list", action="store_true", help="仅列出候选串口")
    args = ap.parse_args()

    if args.list:
        for p in find_ports():
            print(p)
        return

    port = args.port
    if port is None:
        cands = find_ports()
        if not cands:
            sys.exit("未找到 K230 的 USB 串口，请先插好 Type-C 并确认板端已开 usb_cam_stream；"
                     "或用 --list 查看后 --port 手动指定")
        port = cands[0]
        if len(cands) > 1:
            print(f"发现多个串口 {cands}，默认用 {port}；如无画面可换 --port {cands[1]}")

    ser = serial.Serial(port, args.baud, timeout=1.0)
    print(f"opened {port}")

    os.makedirs(args.save, exist_ok=True) if args.save else None

    buf = b""
    fps_t0 = time.time()
    fps_cnt = 0
    while True:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
        # 找帧头
        idx = buf.find(MAGIC)
        if idx < 0:
            buf = buf[-3:] if len(buf) > 3 else b""
            continue
        buf = buf[idx:]
        if len(buf) < HEADER_LEN:
            continue
        (jpeg_len,) = np.frombuffer(buf[4:8], dtype="<u4")
        (width,) = np.frombuffer(buf[8:12], dtype="<u4")
        (height,) = np.frombuffer(buf[12:16], dtype="<u4")
        if jpeg_len <= 0 or jpeg_len > 4 * 1024 * 1024:
            buf = buf[4:]
            continue
        total = HEADER_LEN + jpeg_len
        if len(buf) < total:
            rest = read_exact(ser, total - len(buf))
            if rest is None:
                continue
            buf += rest
        jpeg = buf[HEADER_LEN:total]
        buf = buf[total:]

        img = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            continue
        fps_cnt += 1
        if args.save:
            cv2.imwrite(os.path.join(args.save, f"frame_{time.time():.3f}.jpg"), img)
        else:
            cv2.imshow(f"K230 USB cam {port} ({width}x{height})", img)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        now = time.time()
        if now - fps_t0 >= 2:
            print(f"recv {fps_cnt / (now - fps_t0):.1f} fps")
            fps_t0, fps_cnt = now, 0

    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
