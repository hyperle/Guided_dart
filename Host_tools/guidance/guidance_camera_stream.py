#!/usr/bin/env python3
import argparse
import asyncio
import base64
import json
import time
import threading

import serial
import websockets

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
WS_PORT = 8765

latest_jpeg = None
latest_jpeg_time = 0.0
lock = threading.Lock()


def serial_reader(port, baudrate):
    global latest_jpeg, latest_jpeg_time
    buf = b""
    while True:
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
        except Exception:
            time.sleep(1)
            continue
        try:
            while True:
                try:
                    chunk = ser.read(max(1, ser.in_waiting or 1))
                except Exception:
                    break
                if chunk:
                    buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    line = line.strip()
                    if line.startswith(b"FRAME "):
                        try:
                            jpeg = base64.b64decode(line[6:])
                            if jpeg[:2] == b"\xff\xd8":
                                with lock:
                                    latest_jpeg = jpeg
                                    latest_jpeg_time = time.time()
                        except Exception:
                            pass
        except Exception:
            pass
        finally:
            try:
                ser.close()
            except Exception:
                pass
        time.sleep(1)


def make_handler():
    async def handler(websocket):
        sub_state = [False]
        await websocket.send(json.dumps({"op": "serverInfo", "name": "openmv", "capabilities": {}}))
        await websocket.send(json.dumps({
            "op": "advertise", "channels": [{
                "id": 1, "topic": "/camera/image/compressed",
                "encoding": "json", "schemaName": "foxglove.CompressedImage",
            }],
        }))

        async def sender():
            while True:
                await asyncio.sleep(0.05)
                if not sub_state[0]:
                    continue
                with lock:
                    jpeg = latest_jpeg
                    t = latest_jpeg_time
                if jpeg is None:
                    continue
                await websocket.send(json.dumps({
                    "op": "publish",
                    "channel": {"id": 1, "topic": "/camera/image/compressed"},
                    "data": {
                        "timestamp": {"sec": int(t), "nsec": int((t - int(t)) * 1e9)},
                        "frame_id": "camera",
                        "data": base64.b64encode(jpeg).decode("ascii"),
                        "format": "jpeg",
                    },
                }))

        async def receiver():
            try:
                async for raw in websocket:
                    msg = json.loads(raw)
                    if msg.get("op") == "subscribe":
                        sub_state[0] = True
                    elif msg.get("op") == "unsubscribe":
                        sub_state[0] = False
            except Exception:
                pass

        send_task = asyncio.create_task(sender())
        try:
            await receiver()
        finally:
            send_task.cancel()
            try:
                await send_task
            except asyncio.CancelledError:
                pass

    return handler


async def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--ws-port", type=int, default=WS_PORT)
    args = parser.parse_args()

    threading.Thread(target=serial_reader, args=(args.port, args.baudrate), daemon=True).start()

    print(f"Foxglove server: ws://localhost:{args.ws_port}")
    print(f"Topic: /camera/image/compressed")
    print(f"Waiting for OpenMV on {args.port}...")
    async with websockets.serve(make_handler(), "0.0.0.0", args.ws_port):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
