# OpenMV Guidance

OpenMV 侧绿灯识别、板载录像与经控制板导出工程。

职责：

- 采集图像并识别绿灯圆心
- 通过 UART 向 STM32 发送 `x/y/area`
- 接收 STM32 转发来的参数热更新命令并立即生效
- 在 OpenMV 本地文件系统持续分段保存 MJPEG 录像

目录：

- `src/main.py`：OpenMV 主入口
- `src/green_light_detector.py`：绿灯识别与参数逻辑
- `src/video_recorder.py`：滚动录像管理
- `tools/upload_openmv.py`：PlatformIO 上传脚本，将 `src/main.py` 写入 OpenMV

说明：

- 这里的 `platformio.ini` 主要用于工程组织与统一上传入口，不依赖 OpenMV 官方 PlatformIO board 支持。
- 默认串口端口在 `platformio.ini` 的 `[openmv]` 段配置。
- 录像优先保存到 `/sdcard/recordings`，未插卡时回退到 `/flash/recordings`。
- 主机侧可通过仓库脚本 `./.script/record-list` 与 `./.script/record-pull` 在“只连接控制板串口”的前提下列出并下载录像。
