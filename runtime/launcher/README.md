# launcher —— K230 RT-Smart 开机自启动器

在 RT-Smart 里，"开机自启动"不是靠文件系统里的 rc 脚本，而是：

1. 把**一个用户程序**（本启动器）注册为内核的开机自启命令；
2. 启动器再读清单文件（默认 `/sdcard/app/startup_final.list`，可用 `argv[1]`
   或位置参数覆盖），用 `fork() + execv()` 把其他 ELF **逐个拉起为独立用户
   进程**（RT-Smart 每个用户 ELF 就是一个进程，内核 lwp 实现了
   `sys_fork / sys_execve / sys_waitpid`）。

本程序**平台无关**：纯 POSIX C11，不含任何 K230/MPP 头文件，因此 host 也能构建
与运行，可以在电脑上用假清单先把解析、前后台、子日志、单实例锁验证一遍。

launcher 自身日志和子进程 console 日志服从 `/sdcard/app/control/write.pause` 开关。
子进程 stdout/stderr 通过管道交给 launcher，暂停时继续消费输出并丢弃，避免占住日志文件
或阻塞应用；恢复后重新追加文件。`-n` 留下后台应用时由独立收集进程消费到 EOF。
主机 `write-rtos off|on`、清理脚本和 runtime 控制说明见 [写入控制](../WRITE_CONTROL.md)。

## 本目录文件

| 文件 | 说明 |
|---|---|
| `main.c` | 启动器源码（读清单、fork/exec、监督回收） |
| `startup.list` | 启动清单（**源码母版**；板端运行时默认文件是 `/sdcard/app/startup_final.list`，需拷贝并改名） |
| `README.md` | 本文件 |

构建目标与产物路径在 `runtime/CMakeLists.txt`（本仓库约定：模块目录只放源码，
目标集中声明）：`build-cross/launcher/launcher`。

当前启动清单只启用 `dart_app` 的识别、坐标、两轴映射和四路 PWM 链路。
零点、两轴增益与增量限幅以 `startup.list` 当前参数为准。
IMU、Wi-Fi 关闭，录像启用并显式设置为 60 FPS；待写帧超过一帧周期（约 16.7 ms）丢弃。
详细控制参数见 [舵机控制说明](../../src/control/README.md)。

## 清单语法（每行一条）

```
绝对路径 [参数...] [&|&!]
```

- 行尾 `&` → **后台启动**，不等待，适合常驻服务；启动器随后监督回收。
- 行尾 `&!` → **后台分段运行**，退出后等待 5 秒再启动同一命令。
  `dart_app` 完成相机清理后会写入 launcher 提供的 `.done` 标记；板端
  `waitpid`/`kill(pid,0)` 可能仍把已退出的进程报为存活，因此以标记触发下一轮。
- 不带 `&` → **前台启动**，等它退出后再执行下一行，适合初始化类程序。
  前台程序也可在完成全部资源清理后，将自身 PID 写入 `DART_EXIT_MARKER`，
  用于处理 RT-Smart 无法可靠回收已退出进程的情况。
- `#` 开头或空行 → 注释。
- 每行 ≤ 511 字符，token ≤ 31 个（路径 + 最多 31 个参数）；**解析器不做引号/转义**，
  参数里不能有空格。

启动器读取完整清单后立即关闭文件，再执行其中的程序，避免前台等待或子进程
长期占用 FAT 读锁。清单最长 1 MiB。若旧进程仍占用清单，可上传同目录的
`startup_final.list.next`；下次启动会将旧清单保留为 `.previous`，再启用 `.next`。
上传后应完整回读校验，完成前不要复位；`--dry-run` 不启用暂存清单。

## 命令行

```
launcher [选项] [清单路径]
  -n, --no-supervise    启动完后台项后立即退出，不进入监督
  -t, --dry-run         只解析并 stat 每项、打印将执行的命令行，不 fork
  -L, --log-dir DIR     日志与 pid 目录（默认 /data/app/logs）
      --fg-timeout SEC  前台项最长等待秒数，0 = 不限（默认 0）
      --poll-ms MS      等待/监督轮询间隔（默认 1000）
      --heartbeat SEC   存活心跳日志间隔，0 = 关闭（默认 60）
      --force           忽略单实例锁，强制启动
  -h, --help
```

日志布局（都在 `--log-dir` 下）：

| 文件 | 内容 |
|---|---|
| `launcher.log` | 启动器自身的结构化日志（每行 open/write/fsync/close） |
| `<应用名>.console.log` | 被拉起程序的 stdout/stderr（每个应用一份，便于归因） |
| `<应用名>.pid` | 后台项 pid，退出时删除 |
| `launcher.pid` | 单实例锁，正常退出时删除 |

## 相对 v1 的四处硬化

v1 的 `apps/launcher/main.c` 机制正确，但有四个已造成实际损失的缺陷，本实现逐条处理：

1. **不把 `waitpid` 当唯一判据**。v1 注释自承"waitpid 行为不可靠(可能返回 0)"，
   结果 7 次开机、28 次前台启动全部被记成"同一瞬间 exited, code=0"，掩盖了
   进程仍在运行的事实。本实现前台项用 `WNOHANG` + `kill(pid,0)` 轮询判定，
   `waitpid` 返回 `ECHILD`/异常时退化为 `result=gone status=unknown` 并继续。
2. **每个子进程独立 console 日志**。v1 把所有子进程的 stdout/stderr 塞进同一个
   `launcher-child.log`，多应用时无法归因。
3. **单实例锁** `launcher.pid`。双 launcher 会拉起两份独占 VICAP 的 payload，
   v1 实测必死（只有 9/28 次写出日志）。已在运行则拒绝启动，`--force` 可覆盖。
4. **`--dry-run`**：上电前就能发现路径写错、文件缺失，不必等开机看日志。

## 使用步骤

```bash
# 1. 构建（交叉，产物 build-cross/launcher/launcher）
bash .script/build-rtos launcher

# 2. 部署到板子 /sdcard/app（默认 USB/MTP，也可指定网络或读卡器）
bash .script/deploy-rtos launcher
BOARD_IP=192.168.1.100 bash .script/deploy-rtos launcher
SDCARD="/media/$USER/SDCARD" bash .script/deploy-rtos --keep-logs

# 3. 先在板端 msh 手动验证
msh> /sdcard/app/launcher -t                 # 只校验清单
msh> /sdcard/app/launcher                    # 读 /sdcard/app/startup_final.list
msh> /sdcard/app/launcher /tmp/my.list       # 换一份清单测试
```

确认能拉起程序后，再把它注册成开机自启：

```bash
make menuconfig
# RT-Smart Configuration
#   Rtsmart auto execute command string = /sdcard/app/launcher
make && 烧录固件
```

之后日常更新只需替换 `/sdcard/app/` 下的 ELF 或编辑
`/sdcard/app/startup_final.list`，重启即生效。

## 保留旧固件自启路径的原生入口

2026-11-01 联调时，板端 `/sdcard` 根目录缺少 `micropython`，只有
`revision.txt` 和已部署的 `app/`；`/data/app/logs` 没有本轮的
`launcher.log`、`dart_app.log`。本机 SDK 的自启配置仍为
`CONFIG_RTT_AUTO_EXEC_CMD="/sdcard/micropython &"`。
仅部署 `/sdcard/app/launcher` 不会改变固件已经编译进去的启动路径。

本轮将交叉编译的 **原生 launcher ELF** 额外写入 `/sdcard/micropython`，
保留 `/sdcard/app/launcher` 和默认清单位置。这只是文件路径兼容，
入口内容不是 Python 解释器。回读 SHA-256 与 launcher 完全一致；
需断电重上电后，以新日志确认实际运行。更新 launcher 时应同步这个副本。
返回 CanMV 时需将此路径恢复为对应固件的真正解释器。

## 重要：不要用 @preload / fastboot 快起

新版 SDK 的启动命令默认可能是 `@preload &`（Fast Boot 快起）：此时开机跑的
是**烧在镜像固定位置**的 rtapp 备份（由 U-Boot 预载、ATAG 描述），**根本不读
文件系统**——表现为"删了 /sdcard 下的 ELF 开机照跑、替换也不生效"。
要实现"替换 SD 卡文件即更新"，启动命令必须填**文件系统绝对路径**
（如 `/sdcard/app/launcher`），而不是 `@preload`。

> 源码依据：SDK `src/rtsmart/Kconfig` 中 `RTT_AUTO_EXEC_CMD`（菜单
> "Rtsmart auto execute command string"）在 Fast Boot 开启时默认 `@preload &`；
> 内核 bsp 启动时 `msh_exec(CONFIG_RTT_AUTO_EXEC_CMD, ...)` 执行该字符串
> （`@preload` 开头则走内存预载 rtapp，否则按 msh 命令行处理，文件系统需已挂载）。
> secure boot 强制要求命令以 `@preload` 开头，此时本方案不可用。

## 常见问题

- **开机没启动 launcher**：检查启动命令是否仍是 `@preload`；确认固件非 secure boot。
- **launcher 报 already_running**：上一轮没退干净（或日志目录残留 pid 文件）。
  先确认板上没有在跑的 launcher/payload，再 `--force`。
- **某项启动失败**：看 `launcher.log` 里的 `op=precheck code=stat_failed`（路径写错/
  文件没拷到卡上）或 `op=exec code=exec_failed`（架构不符、动态链接等）。
- **前台项一直不返回**：说明它其实是常驻服务，应当在清单里加 `&`；临时可用
  `--fg-timeout` 观察。
- **顺序依赖**：依赖项用前台（不带 `&`）保证先后；互相独立的常驻服务用 `&`。
- **想要崩溃自动重启**：当前只监督回收、不重启（与 v1 一致）。扩展点就是
  `supervise()` 里对指定路径重新 `fork+execv`；注意任何独占 VICAP 的 payload
  重启前必须确认上一实例已彻底退出。

## 验证状态

- 已通过（2026-10-13，本机）：host 构建 `build/launcher/launcher`；host 实跑
  验证了清单解析、`#`/空行、前后台语义、`--dry-run`、缺失路径跳过、
  `<name>.console.log`/`.pid` 生成、`--no-supervise`、单实例锁与 `--force`。
- 已通过：交叉构建产物为 RT-Smart 可加载 ELF（静态链接、入口 `0x200000000`，
  与 `-T link.lds` 一致）。
- **待板端验证**：真实 fork/execv/waitpid 语义、`/sdcard` 挂载、开机自启注册。
