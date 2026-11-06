# 主机工具与板端诊断探针

本目录的 Qt 播放器、主机 Python 和 ROS 2 launch XML 不参与板端应用 CMake、
交叉编译或部署。主机 Python 使用标准库组织解析、时间轴和几何计算；ROS 2 导入集中在
bag 写入边界，无需创建 ROS package 或运行 colcon。

[`canmv_tune.py`](canmv_tune.py) 在 Dart V1 的 CanMV MicroPython 上运行，
用于调整与 RT-Smart C/C++ 同量纲的曝光行数、灰度候选阈值、背景对比度和四路舵机零点。
调参项集中在文件顶部；相机需配套 RAW 接口，解释器构建及运行方法见
[`canmv_tuning/README.md`](canmv_tuning/README.md)。关闭相机后可直接在当前固件标定 PWM。

[`sc038hgs_raw/`](sc038hgs_raw/README.md) 是单独交叉构建的板端相机诊断探针，
包含 RAW 转换器和历史 ISP 对照资源。它不参与正式应用的构建或部署，
手动运行时独占相机资源。

[`pwm_swing/`](pwm_swing/README.md) 是独立的四路舵机诊断程序，按 333 Hz、
1520 µs 中心、±100 µs 摆幅逐路和同时测试。显式运行
`bash .script/build-rtos pwm_swing` / `bash .script/deploy-rtos pwm_swing` 构建与部署，
板端手动启动后约 43 秒结束。
当前 launcher 清单启用识别与录像，PWM 测试行已注释；按需手动测试时，
开机配置与日志说明见该工具说明。

[`wifi/wifi_test.py`](wifi/wifi_test.py) 是 CanMV MicroPython 独立 Wi-Fi 测试脚本，
在 VS Code 插件中直接运行。默认检测 RTL8189 / SDIO 网卡并扫描附近热点，空结果自动重试；
填写 SSID 和密码后验证连接、IP 和链路；填写上传地址后逐帧上传相机 JPEG。
[`wifi/video_server.py`](wifi/video_server.py) 在电脑端用 FFmpeg 编码 H.264/HLS，
供手机网页播放，可通过公网隧道跨网络访问。运行方法、实测和固件要求见 [Wi-Fi 说明](wifi/README.md)。

[`wifi_receiver/`](wifi_receiver/README.md) 是原生 RT-Smart 回传配套的 Rust 主机服务。
`dart_app --wifi on --wifi-config /data/app/wifi.conf` 将同一张采集图异步 JPEG 压缩并上传；
Rust 接收后调用 FFmpeg 编码 H.264，提供本地网页/HLS，同时保存每轮 MP4。
构建、配置、网络条件和验收命令见该工具说明。

## 录像逐帧播放器

[`video_player/`](video_player/README.md) 移植远程 `k230_v1` 的 Qt 播放器，
适配当前录像目录及图片、录像 CSV、检测 CSV 的精确关联：

```bash
bash .script/player records/rec_0001
# 无窗口检查图片与文字关联：
bash .script/player --check records/rec_0001
```

滑块或左右键切帧，点击查看原图像素坐标；右侧显示原始文字及文件行号，叠加有效
目标和扫描窗口。以 `image_file` 和帧身份关联，缺图/丢帧不会按行错配。每轮录像应
同时归档对应的 `records/detection.csv`，避免后续运行覆盖。详细构建、过滤和核对规则见
播放器说明。

`pngs` 容器可直接用播放器打开；需要独立 PNG 时运行
`python3 tools/rec_extract_pngs.py records/rec_NNNN --strict --output /tmp/frames`。
`python3 tools/rec_stall_report.py records/rec_NNNN --log logs/dart_app.log`
汇总实际保存帧率、采集间隙、文件边界及写盘/日志耗时，用于核对录像卡顿。

## IMU 日志转 ROS 2 bag

`imu_log_to_bag.py` 读取板端 `runtime/imu/Session` 写出的 v1 CSV 格式 `.log`，
默认生成带消息定义的 MCAP bag，便于 Foxglove 直接打开和 ROS 2 回放。
先把板端 `/data/app/logs/imu_NNNNNN.log` 中需要的一轮复制到主机，例如
将 `imu_000001.log` 保存为 `records/imu.log`。IMU start/stop 状态行记录实际文件名；
当前正在写入的文件可能被板端共享锁阻止读取，可在停止或下一次上电后读取上一轮。
以下命令在仓库根目录执行：

```bash
# 本机可用的完整 bag 环境为 Rolling；bash 用户用 setup.bash。
source /opt/ros/rolling/setup.zsh

/usr/bin/python3 tools/imu_log_to_bag.py records/imu.log -o records/imu_bag
ros2 bag info records/imu_bag
```

输出目录必须不存在；不会覆盖原始日志或已有 bag。失败时清理本次创建的临时 bag。
`metadata.yaml` 和 `.mcap` 文件位于输出目录内。`--storage sqlite3` 可改为 SQLite bag。
帮助可直接运行 `python3 tools/imu_log_to_bag.py --help`，不要求 source ROS。

默认 `--position fixed`：平移为零，姿态直接使用日志中的四元数，不重新积分角速度，
也不重新执行板端估计器。日志里的四元数顺序是 w,x,y,z，输出 ROS 字段为 x,y,z,w；
表示从传感器轴到本批次参考系的旋转，不取逆、不额外转换为 ENU/NED。
输出前仅归一化四元数；原始字段完整保留在 `/imu/status`。

如需观察估算的平移：

```bash
/usr/bin/python3 tools/imu_log_to_bag.py records/imu.log \
  -o records/imu_bag_motion --position velocity
```

日志没有位置，`velocity` 模式使用日志里的参考系速度，按原始 `timestamp_ns`
做梯形积分；每个批次的位置从零开始。这是主机离线估算，受速度漂移和缺失数据影响。
拒绝样本、读取错误、序号缺口、dropped 增长和过大时间间隔均不跨越积分；
当前 `integrated=0` 时位置保持不变，从该样本建立下一个区间的基点。
原始时间戳只用于这个位置估算，不决定 bag 的事件顺序。

新日志每个文件对应一轮启动；多个输入文件按命令顺序编号为 session 1、2……。
旧版追加日志中的多轮运行仍按出现顺序编号。
可限定一个批次：

```bash
/usr/bin/python3 tools/imu_log_to_bag.py records/imu.log \
  -o records/imu_s1_g2 --session 1 --generation 2
```

`seed` 成功、失败和 `reset` 都保留在诊断话题。只有已接受、有参考系、姿态和速度
有限的 `sample`，以及成功的 `seed`，会生成位姿/TF。时间 gap 样本若仍被估计器接受，
会按上层所见输出其保持的姿态，质量位完整保留。拒绝样本仍输出原始测量，
但不把旧估计冒充新位姿；`read_error` 不伪造测量。

默认遇到残缺行、倒退的处理时间、非法序号或字段即报出文件名与行号。
断电后末行不完整等情况可用 `--skip-bad-rows` 显式跳过，终端逐条报告。
未知日志版本和损坏的列名始终报错。

## Foxglove 直接打开

在 Foxglove 的 Open local file 中选择输出目录里的 `.mcap` 文件，或直接拖入。
此方式不需要运行 Bridge 或 `ros2 bag play`。

1. 添加 **3D** 面板，打开面板设置，将 **Display frame** 设为 `world`。
2. 在 **Transforms** 中打开 `imu_reference_s1_g1` 与 `imu_link_s1_g1` 的可见性，
   显示坐标轴和标签。参考坐标轴保持不动，`imu_link` 随日志姿态旋转。
3. 播放或拖动时间轴；也可启用 `/imu/pose`，以箭头查看传感器位姿。
4. 添加 **Plot** 面板，选择 `/imu/rpy.vector.x`、`.y`、`.z`，分别观察 roll、pitch、yaw
   （rad）；`/imu/velocity.vector.x` 等可显示参考系速度（m/s）。
5. 添加 **Raw Messages** 或 **Diagnostics** 面板，查看 `/imu/status` 的事件、原始时间、
   质量位、generation、dropped 及原始字段；`/imu/generation.data` 可单独绘制批次变化。

每个 `session + generation` 使用独立的 TF 父子节点：

```text
world                         仅供显示的共同基准
  ├─ imu_reference_s1_g1       第一轮、第一次积分的参考系
  │    └─ imu_link_s1_g1       该批次的传感器位姿
  └─ imu_reference_s1_g2       重新 seed 后的新参考系
       └─ imu_link_s1_g2
```

`world -> imu_reference_*` 是人为设置的单位变换，方便在同一个 3D 面板查看各批次；
它不表示不同批次已经全局配准。重新 seed 后不会在前后两个批次之间插值，
也不会把位置连成一条连续轨迹。历史坐标轴可能继续显示，按需关闭其他批次或使用筛选参数。
转换结束时终端会列出实际生成的坐标系名称。

bag 与消息 `header.stamp` 统一采用日志的 **recorded_ns 处理顺序**，保持上层命令
与样本的真实先后。第一轮从 ROS 时间 1 秒开始，同轮保留处理时间间隔；
同一纳秒内的事件相隔 1 ns。不同轮之间间隔 1 秒，避免板子重启后的单调时钟倒退。
这条时间轴不代表 UTC，也不保留两轮间的真实停机时间。所有原始时间都保留在诊断中。
TF 与位姿在相同 bag 时间发布，便于 Foxglove 时间轴跳转。

## 通过 Foxglove Bridge 回放

本机检查时 Rolling 已有 rosbag2、MCAP 和标准消息，但尚未安装 `foxglove_bridge`。
如果需要 WebSocket 回放，先在所选 ROS 发行版安装对应主机依赖：

```bash
sudo apt install ros-${ROS_DISTRO}-foxglove-bridge
# 若使用其他尚未安装 bag 支持的 ROS 2 环境：
sudo apt install ros-${ROS_DISTRO}-rosbag2-py ros-${ROS_DISTRO}-rosbag2-storage-mcap \
  ros-${ROS_DISTRO}-ros2bag ros-${ROS_DISTRO}-launch-xml
```

启动 Bridge 和一个暂停的 bag 播放器：

```bash
ros2 launch ./tools/foxglove_launcher.xml \
  play_bag:=true bag:="$PWD/records/imu_bag"
```

在 Foxglove 选择 Open connection → Foxglove WebSocket，地址填
`ws://127.0.0.1:8765`。
启动 XML 的子进程不接收终端键盘；连接后在另一个已 source ROS 的终端执行：

```bash
ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume '{}'
# 暂停：
ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause '{}'
```

播放器发布 `/clock`，Bridge 使用 `use_sim_time=true`；随后按上面的 3D 步骤设置。
XML 默认监听本机，`address:=0.0.0.0` 可供其他主机连接，`port:=8766` 可改端口，
`rate:=0.5` 可半速回放。

也可单独启动 Bridge，再在另一个已 source 同一 ROS 环境的终端中回放：

```bash
ros2 launch ./tools/foxglove_launcher.xml
ros2 bag play records/imu_bag --clock 100 --loop
```

XML 启动的是 ROS Bridge 和可选播放器；Foxglove 网页或桌面应用由用户打开。
离线文件方式最适合拖动时间轴；WebSocket 方式观察 ROS 实际回放。

## 话题与验证

| 话题 | 类型 | 数据 |
|---|---|---|
| `/tf` | `tf2_msgs/msg/TFMessage` | 显示基准到批次参考系、参考系到传感器的动态变换 |
| `/imu/pose` | `geometry_msgs/msg/PoseStamped` | 日志姿态与固定/离线估算位置，批次参考系 |
| `/imu/raw` | `sensor_msgs/msg/Imu` | 传感器轴 SI 角速度和比力；orientation_covariance[0]=-1 |
| `/imu/raw_counts/acceleration`、`/imu/raw_counts/gyroscope` | `geometry_msgs/msg/Vector3Stamped` | 原始 ADC 计数，传感器轴 |
| `/imu/temperature` | `sensor_msgs/msg/Temperature` | 摄氏温度 |
| `/imu/velocity` | `geometry_msgs/msg/Vector3Stamped` | 积分速度，m/s，批次参考系 |
| `/imu/rotation`、`/imu/rpy` | `geometry_msgs/msg/Vector3Stamped` | 累计旋转、RPY，rad，批次参考系 |
| `/imu/linear_acceleration` | `geometry_msgs/msg/Vector3Stamped` | 去重力加速度，m/s²，批次参考系 |
| `/imu/generation` | `std_msgs/msg/UInt64` | 原始批次编号，跨 session 可能重复 |
| `/imu/status` | `diagnostic_msgs/msg/DiagnosticArray` | 每条原始事件全部列、session、文件行号、元数据和可视化状态 |

诊断 KeyValue 的 value 按 ROS 契约为字符串；数值曲线使用对应的标准数值话题。
协方差未记录，因此保留 unknown，不伪造标定值。

```bash
# 无 ROS 依赖时，运行解析/几何测试，跳过 ROS bag 与 launch 用例。
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -m unittest discover -s tests \
  -p test_imu_log_to_bag.py -v
```

source ROS 2 后同一命令还会验证真实 MCAP/SQLite 写入和 CDR 读回、消息定义、
四元数方向、位移积分、重积分分段、时间重建、异常数据和 launch XML。
2026-10-16 在本机 Rolling 上 7 项测试通过；另以临时 MCAP 实际执行了 bag 回放，
成功接收 5 组 TF、10 条 `/clock`，并验证了暂停/恢复服务和带空格的 bag 路径。
验证范围不包含 Foxglove GUI 的人工交互，Bridge 安装后可按上述步骤验证。

官方参考：[Foxglove ROS 2/MCAP](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)、
[3D/TF](https://docs.foxglove.dev/docs/visualization/panels/3d)、
[Bridge 启动配置](https://github.com/foxglove/foxglove-sdk/tree/main/ros/src/foxglove_bridge)。
