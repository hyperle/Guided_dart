"""K230 CanMV camera preview for the CanMV VS Code extension.

Run this file with ``CanMV: Run Active Python Script`` after connecting the
board.  The virtual display with ``to_ide=True`` is what sends frames to the
VS Code Preview panel.
"""

import time

from media.sensor import Sensor
from media.display import Display
from media.media import MediaManager


def main():
    sensor = None
    try:
        # QVGA keeps USB preview traffic low and works with most K230 cameras.
        sensor = Sensor(width=640, height=480)
        sensor.reset()
        sensor.set_framesize(width=640, height=480)
        sensor.set_pixformat(Sensor.RGB565)

        # VIRT is the IDE framebuffer. Do not replace it with LCD output when
        # the goal is to view frames in VS Code.
        Display.init(Display.VIRT, width=640, height=480, to_ide=True)
        MediaManager.init()
        sensor.run()

        clock = time.clock()
        while True:
            clock.tick()
            img = sensor.snapshot()
            Display.show_image(img)
            # Keep a small status signal in the CanMV terminal.
            if int(clock.fps()) % 30 == 0:
                print("camera fps: %.1f" % clock.fps())
    except KeyboardInterrupt:
        print("camera preview stopped")
    except Exception as exc:
        print("camera preview error: %s" % exc)
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
