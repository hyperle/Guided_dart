# .script/ —— 仓库短命令

日常命令在仓库里任何位置都能跑（脚本自己会定位仓库根）。构建使用 CMake >= 3.28
和 C11 / C++23 编译器，预设生成器为 Ninja；交叉编译使用 SDK 配套 RISC-V musl 工具链。
不需要命名模块扫描工具。环境配置见 [工具链说明](../README.md#3-工具链与-sdk一次性)。

| 命令 | 作用 | 参数 |
|---|---|---|
| `build-rtos` | 交叉编译板端可执行文件 | `-r`（清 build-cross 重编）/ `--host`（host 编译检查）/ `launcher` / `dart_app` / `pwm_swing` / `imu_raw_probe` |
| `deploy-rtos` | 把产物上板到 `/sdcard/app/` | `launcher` / `dart_app` / `pwm_swing` / `imu_raw_probe` / `list`（只推清单）；可加 `--keep-logs` |
| `clean-rtos-data` | 设置暂停后清除两个分区上的空闲日志与录像，跳过写入中的项 | `--dry-run` / `--keep-paused` / `logs` / `records` |
| `stream-rtos` | 运行时关闭/恢复回传编码和 Wi-Fi 发送，不等待板端应答 | `off` 暂停 / `on` 解除暂停；仅 `--wifi on` 启动的应用会恢复 |
| `write-rtos` | 暂停或恢复录像与日志写入，不等待板端应答 | `off` 暂停 / `on` 解除暂停 |
| `clean-rtos` | 清除板端内容（**保留目录本身**；固件自带的 `examples/` 默认不动） | `launcher` / `dart_app` / `list` / `logs` / `records` / `examples` |

`stream-rtos off` 仅创建 `/sdcard/app/control/stream.pause`，由回传管理线程异步关闭
编码与发送并回收自身像素缓冲。日志 `module=wifi state=paused active=0 buffers=0`
表示完成回收；`state=stopping active=0` 表示已停止接收新帧，仍在等待旧调用结束。
`stream-rtos on` 清除此标记，重新加载回传配置并恢复；它不改变 `write.pause`，
也不控制本地录像、日志或其他任务。共享网卡关联保留，避免断开其他使用者。

清理不等待板端停写应答；成功后释放本次暂停，有跳过项或失败时保持暂停。
处理完成后可运行 `write-rtos on` 恢复。详见 [写入控制](../runtime/WRITE_CONTROL.md)。

要让上面这些名字能**直接敲**（不带 `./`、不带路径），先跑一次 `install-rtos`：

```bash
bash .script/install-rtos --both     # 一次性；详见下面"怎么直接敲 build-rtos -r"
```

`deploy-rtos` 默认调用 `deploy-mtp.sh`，将产物写入当前 USB 板子的
`/sdcard/app`，直接运行即可，无需先设置环境变量；USB 存储尚未挂载时会尝试自动挂载：

```bash
deploy-rtos
deploy-rtos dart_app   # 只部署 dart_app
```

四路舵机测试通过显式目标构建和部署：

```bash
build-rtos pwm_swing
deploy-rtos pwm_swing
# 板端 msh 手动运行 /sdcard/app/pwm_swing，约 43 秒后关闭输出。
```

接线、333 Hz / 1420～1620 µs 参数和波形验收见
[`pwm_swing` 说明](../tools/pwm_swing/README.md)。

当前启动清单为 `dart_app` 完整视觉控制：识别、全图坐标、两轴映射与四路 PWM，
启用录像，IMU 和 Wi-Fi 关闭。零点、方向、增益和增量限幅以清单参数为准，说明见 [舵机控制说明](../src/control/README.md)。

```bash
build-rtos dart_app
build-rtos launcher
deploy-rtos dart_app
deploy-rtos launcher
deploy-rtos list
# 重启后由 launcher 加载 startup_final.list。
```

坐标写入 `/data/app/records/detection.csv`，实际下发脉宽在
`/data/app/logs/dart_app.log` 的 `module=servo` 状态行。
IMU 直读程序仅保留为手动测试，读取命令及波形见 [IMU 测试说明](../runtime/imu/README.md)。

固件的开机命令必须实际指向 launcher。若仍为 `/sdcard/micropython &`，
仅部署 `app/launcher` 不会启动应用；本轮用原生 launcher ELF 补到该旧入口，
断电重上电后核对 `launcher.log` 和 `dart_app.log`，详见
[旧固件入口说明](../runtime/launcher/README.md#保留旧固件自启路径的原生入口)。

需要指定其它板子时，可通过环境变量或 `.script/board.env` 配置：

```bash
export BOARD_IP=192.168.1.100     # 网络模式：ssh/scp（需免密），优先级最高
export SDCARD=/media/$USER/SDCARD # 读卡器：/sdcard 分区
export DATA_SDCARD=/media/$USER/DATA # 读卡器：/data 分区（全量部署/数据清理时需要）
export SDCARD='gphoto2://Kendryte_CanMV_001000000/store_ffff0001'
                                  # MTP 模式：板子用 USB 挂着，这是板端 /sdcard
```

都写进 `.script/board.env` 更省事（该文件已被 `.gitignore` 忽略）：

```bash
cp .script/board.env.example .script/board.env   # 然后改里面的地址
```

### MTP（板子用 USB 挂着）注意

当前板子暴露两个存储：程序与启动清单位于 `/sdcard/app`，日志与录像位于 `/data/app`。

| MTP store | 板端路径 | 说明 |
|---|---|---|
| `store_ffff0001` | `/sdcard` | **要部署到这个**；判据：根目录有 `revision.txt`（SDK 的 genimage 只把它写进 SDCARD 分区） |
| `store_ffff0002` | `/data` | 数据分区，保存 `/data/app/logs` 与 `/data/app/records` |

MTP 部署会核对 `revision.txt`；缺少该文件时，必须存在已部署的 `app/launcher` 与
`app/startup_final.list` 才能确认目标，否则拒绝写入。地址也可以直接给
gvfs 的本地路径（含冒号，记得加引号）。

MTP 部署由 `deploy-rtos` 转交 `deploy-mtp.sh`，用 `gio copy` 直接操作
存储 URI。先完整回读备份旧文件，再删除目标名并重新上传，完整回读校验 SHA-256；
替换或校验失败时用备份恢复。当前 GIO 后端不可靠地支持覆盖和改名，脚本使用此序列。
主机上的 `/tmp/dart-mtp-backup.*` 备份会保留；板上日志和录像也会保留。
此前通过 GVfs FUSE 路径运行 `cp` 出现过 0 字节文件，但原生 GIO 上传和改名已在当前
连接上验证成功，不能将 FUSE 写入失败直接判定为所有 MTP 写入都不可用。

回读录像前先用 `write-rtos off` 关闭写入。若在录制期间已经查询或读取过文件，
GVfs/gphoto2 可能继续缓存旧大小，`gio cat` 也可能只返回旧长度；应重新挂载主机端会话后再读取：

```bash
gio mount -u gphoto2://Kendryte_CanMV_001000000/
gio mount gphoto2://Kendryte_CanMV_001000000/
```

这不会重启板子或解除写入暂停。下载后仍需将文件大小、完整帧边界与 `frames.csv`
的 `file_offset` / `file_bytes` 核对；仅凭 `gio cat` 成功退出不能证明录像完整。

## 怎么直接敲 `build-rtos -r`（一次性，不用 `./`）

`.script/` 默认不在 PATH 里。**在仓库根目录执行一次**这条即可（注意：`bash` 后面跟文件名，
不需要 `./`）：

```bash
bash .script/install-rtos --both      # 把 .script/ 追加进 ~/.bashrc 和 ~/.zshrc
source ~/.zshrc                       # 当前终端生效（bash 用户 source ~/.bashrc）
build-rtos -r                         # 之后直接敲，不再需要任何前缀
```

`install-rtos` 做的事：往 rc 里写一段带标记的 PATH 追加（`# >>> dart-rtos >>>` … `# <<< dart-rtos <<<`），
改动前备份成 `<rc>.rtos.bak`。它不接受位置参数，参数只有：

| 参数 | 作用 |
|---|---|
| 无 | 按 `$SHELL` 猜一个 rc（猜中的 rc 不存在、另一个存在时，按存在的那个写） |
| `--both` | `~/.bashrc` 与 `~/.zshrc` 都写（**不确定自己用哪个 shell 就用这个**） |
| `--bash` / `--zsh` | 只写指定的那个 |
| `-u` | 撤销：删掉本脚本写入的段落（两个 rc 都查），rc 里其它行一行不动 |
| `-h` | 帮助 |

行为特点：重复执行是幂等的（路径没变就跳过），仓库搬家后重跑一次会自动把路径改成新位置。
本脚本是"执行"用的，`source` 它会被拒绝（避免 `set -euo pipefail` 污染你的 shell）。

不想动 rc 文件时，直接用路径调用也一样（任何目录都能用）：
`/path/to/dart/.script/build-rtos -r`。

## 典型流程

```bash
bash .script/install-rtos --both   # 只做一次：之后下面这些命令直接敲，不带 ./
build-rtos                  # 增量编译全部目标（含 IMU 直读程序）
build-rtos -r               # 改了 CMake / 换工具链后全量重编
deploy-rtos --keep-logs     # 上板：launcher + dart_app + imu_raw_probe + startup_final.list
clean-rtos                  # 清空 /sdcard/app 下全部内容（目录与固件的 examples/ 保留）
clean-rtos-data --dry-run   # 预览两个分区上的日志、录像及历史归档
clean-rtos-data             # 清日志与录像，保留程序、清单和进程管理文件
clean-rtos records          # 只清两个分区的录像，保留 ELF 和清单
```

`deploy-rtos` / `clean-rtos` / `clean-rtos-data` 的地址解析顺序：

1. `BOARD_IP` 有值 → ssh/scp 模式（会先测连通性，连不上立刻报错，不会跑到一半失败）；
2. 否则 `SDCARD` 有值 → 使用指定的卡挂载点、`app/` 目录或 URI。部署时 MTP URI
   直接交给 GIO，gvfs 路径先取得 URI；清理命令同样使用原生 GIO；
3. 都没有 → 默认使用当前 USB 板子：程序分区 `store_ffff0001`、数据分区 `store_ffff0002`。

`DATA_SDCARD` 可显式覆盖数据分区的 URI、挂载点或应用目录。读卡器模式必须指定它；
USB 默认自动使用同一台板子的第二个 store。`BOARD_DIR` / `BOARD_DATA_DIR` 默认分别为
`/sdcard/app` / `/data/app`；它们只配置脚本访问路径，应用运行路径仍以启动清单和 launcher 参数为准。

## 底下的实现

构建、网络部署和读卡器部署直接在对应命令内实现；MTP 的备份、上传与恢复逻辑单独保留：

| 短命令 | 调用 |
|---|---|
| `build-rtos` | CMake host/cross 预设（`-r` 先清 `build-cross`） |
| `deploy-rtos`（网络） | SSH/scp 上传与 md5 校验 |
| `deploy-rtos`（USB/MTP） | `deploy-mtp.sh`：旧文件备份、SHA-256 回读校验与替换失败恢复 |
| `deploy-rtos`（读卡器） | 临时文件写入、大小与 md5 校验、改名替换后再次校验 |
| `clean-rtos` / `clean-rtos-data` | SSH、本地文件系统或 GIO/MTP 递归删除（见 `_rtos-common.sh`） |
| `install-rtos` | 往 `~/.bashrc` / `~/.zshrc` 写一段 PATH 追加 |
| `check-camera-pool.sh` | 用 SDK 头文件和 fake SDK 调用运行相机资源池/生命周期回归检查 |
| `player` | 独立构建并启动 `tools/video_player` 主机录像播放器 |

构建和部署使用 `DART_BUILD_DIR` 指定产物目录（默认 `build-cross`，host 默认 `build`）；
部署同时兼容旧变量 `BUILD_DIR`，优先级低于 `DART_BUILD_DIR`。相对路径以仓库根目录为基准。
例如：

```bash
DART_BUILD_DIR=/tmp/dart-cross build-rtos
DART_BUILD_DIR=/tmp/dart-cross deploy-rtos dart_app
SDCARD="/media/$USER/SDCARD" DATA_SDCARD="/media/$USER/DATA" deploy-rtos --keep-logs
bash .script/check-camera-pool.sh
```

相机检查仍需本机 C 编译器和 `K230_SDK_ROOT`（默认 `/mnt/mydata/kRTOSSDK`）的 MPP
头文件；它运行真实 K230 相机生命周期实现，覆盖范围与 `build-rtos --host` 的编译检查不同。

`_rtos-common.sh` 是共用的定位/环境/板端访问层，不被直接执行。它带的护栏：

- 只允许删绝对路径，拒绝 `/`，拒绝含 `..` 的路径；
- 删除前逐条打印将被删的项；
- 列目录包含隐藏文件、保留文件名空格，不跟随符号链接；SSH 模式需要支持 POSIX shell 的会话；
- 数据清理先扫描两个分区再删除，分区不可访问时退出，忙碌或不可读的子目录跳过；数据目标不能与程序分区相同。

## 清理日志与录像

```bash
clean-rtos-data --dry-run   # 只预览，适合先确认清理范围
clean-rtos-data             # 清理空闲日志与录像，跳过正在写入的项
clean-rtos-data logs        # 只清日志（clean-rtos logs 同义）
clean-rtos-data records     # 只清录像（clean-rtos records 同义）
```

脚本扫描 `/data/app` 与旧的 `/sdcard/app`，包括归档内的 `logs`、`logs-*`、`logs_*`、
`records`、`records-*`、`records_*`、`rec_NNNN`，以及散落的 `*.log`、`*.log.*`、
`detection.csv`、`detect.csv`、`frames.csv`、MP4/AVI/MKV。日志目录还会清除隐藏文件。
保留程序、启动清单、ISP/模型/示例目录及日志里的 `*.pid` / `*.done`，避免破坏运行中的
单实例锁与完成标记。脚本不会停止或重启板端程序。

删除前默认观察 2 秒，并在每项删除前复查大小、修改时间和目录内容；目录会包含所有
后代的状态，任一文件变化或新增帧都会跳过整段录像。本地/SSH 能读取 `/proc` 时也检查
打开的写句柄，暂时没有追加数据的写入者同样跳过。无法读取元数据、目录检查失败、
删除被拒绝时保留该项（删除过程中失败则保留剩余内容），继续处理其它目标。
输出会分别统计删除与跳过的数量，`--dry-run` 使用相同检测。

`RTOS_IDLE_CHECK_SECONDS` 可设为 1–60 的整数秒，例如：
`RTOS_IDLE_CHECK_SECONDS=5 clean-rtos-data`。MTP 只能根据设备返回的元数据判断，
无法查询板端写句柄，缓存的元数据或观察结束后才开始的写入仍可能漏检；需要完全
排除并发写入时，应先停止写入程序。

回归验证：`python3 tests/test_rtos_scripts.py`（用临时目录模拟三个传输方式，不操作真实板子）。

## 主机录像播放器

```bash
bash .script/player records/rec_0001
bash .script/player --check records/rec_0001
```

`player` 自动增量构建 Qt 主机工具，参数透传给播放器；录像相对路径以调用时的
工作目录为准。`DART_PLAYER_BUILD_DIR` 可覆盖默认构建目录 `build/video_player`。
已通过 `install-rtos` 将 `.script/` 加入 PATH 时，也可直接执行 `player`。
图片和文字关联规则及完整参数见 [播放器说明](../tools/video_player/README.md)。

## 注意

- 手工调试时不要与 launcher 同时跑 payload：两者都会独占 VICAP/VB。
- `deploy-rtos <目标>` 只动该目标，不清日志；全量读卡器部署默认调用 `clean-rtos-data` 清理两个分区的新旧日志与录像，
  用 `deploy-rtos --keep-logs` 可保留。网络和 MTP 部署均保留日志与录像。
- MTP 部署会完整回读校验文件；失败时保留主机备份并报错，请按实际错误排查连接或存储。
