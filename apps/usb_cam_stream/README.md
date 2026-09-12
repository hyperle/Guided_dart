# usb_cam_stream —— Type-C USB(CDC ACM) 把相机画面发给主机

数据通路：`VICAP 采集 NV12 → VENC JPEG 编码 → 写 USB 虚拟串口 /dev/ttyGS0`；
主机端用 `scripts/usb_cam_viewer.py` 读串口、解帧、显示。

## 帧协议

```
[0..3]  magic 'L','C','J','P'
[4..7]  JPEG 长度 (uint32 LE)
[8..11] 图像宽   (uint32 LE)
[12..15]图像高   (uint32 LE)
[16..]  JPEG 数据
```

## 编译 / 部署 / 运行

```bash
bash scripts/build.sh usb_cam_stream          # 产物 build/usb_cam_stream/usb_cam_stream
bash scripts/deploy.sh usb_cam_stream <板子IP> /sdcard/app
# 板端 msh 先手动验证:
msh> /sdcard/app/usb_cam_stream -w 640 -h 480
# 主机端(另一终端):
python3 scripts/usb_cam_viewer.py             # 自动找串口并弹窗显示，q/ESC 退出
```

参数：`-c <0-2>` CSI（默认 2）、`-w/-h` 分辨率、`-q` JPEG 质量、`-f` 目标帧率、
`-d` USB 串口设备（默认 `/dev/ttyGS0`）。

采集侧固定按庐山派 ISP 的 1920x1080 取流（源码里的 `ISP_WIDTH/ISP_HEIGHT`），
VICAP 通道缩放器再输出 `-w/-h` 指定尺寸；VICAP 设备号在庐山派上固定为 0
（CSI 号只用于探测 sensor），所以换 CSI 请用 `-c`，不要改 `vicap_dev`。

## 由 launcher 开机启动

`apps/launcher/startup.list` 已加入：

```
/sdcard/app/usb_cam_stream -w 640 -h 480 &
```

launcher 会以后台方式拉起它（行尾 `&`）。注意同一时刻只能有一个程序占用
VICAP/摄像头（smart_ipc、green_led_rtos 与本程序互斥），清单里别同时启用。

## 排障

- 板端报 `open /dev/ttyGS0 failed`：msh 里 `list_device` 看 USB 串口实际名字
  （ttyGS0/ttyGS1），用 `-d` 指定。
- 主机端找不到串口：`python3 scripts/usb_cam_viewer.py --list`；板子需以 USB
  设备模式接入（Type-C 连电脑），固件需含 CDC ACM 类（庐山派 RT-Smart 默认开启）。
- 主机端有多个 ACM 口（CDC+MTP 复合设备）：用 `--port /dev/ttyACMx` 逐个试，
  有画面即正确。
- 卡顿/无实时性：分辨率或 `-q` 调低；主机没在读时 USB 会背压（写阻塞），
  属正常现象，主机恢复读取即继续。
