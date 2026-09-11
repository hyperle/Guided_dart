# launcher —— 开机自启动器

在 RT-Smart 中，"开机自启动"不是靠文件系统里的 rc 脚本，而是：

1. 把**一个用户程序**（本启动器）注册为内核的开机自启命令；
2. 启动器再去读取清单文件（默认 `/sdcard/app/startup_final.list`，可用
   `argv[1]` 覆盖），用 `fork() + execv()` 把其他 ELF **逐个拉起为独立用户
   进程**（RT-Smart 每个用户 ELF 就是一个进程，内核 lwp 实现了
   `sys_fork / sys_execve / sys_waitpid`）。

## 本目录文件

| 文件 | 说明 |
|---|---|
| `main.c` | 启动器源码（读清单、fork/exec/waitpid；默认清单见 `DEFAULT_LIST`） |
| `CMakeLists.txt` | 构建定义，产物 `build/launcher/launcher` |
| `startup.list` | 启动清单（**源码母版**；板端运行时默认文件是 `/sdcard/app/startup_final.list`，需拷贝并改名） |

## 清单语法（每行一条）

```
绝对路径 [参数...] [&]
```

- 行尾 `&` → **后台启动**，不等待，适合常驻服务；启动器随后统一监督回收。
- 不带 `&` → **前台启动**，等它退出后再执行下一行，适合初始化类程序。
- `#` 开头或空行 → 注释。
- 每行 ≤ 511 字符，token ≤ 16 个。

## 使用步骤

```bash
# 1. 构建
bash scripts/build.sh launcher

# 2. 部署启动器 + 清单到板子 /sdcard/app
bash scripts/deploy.sh launcher <板子IP> /sdcard/app
scp apps/launcher/startup.list root@<板子IP>:/sdcard/app/startup_final.list

# 3. 先在板端 msh 手动验证（默认读 /sdcard/app/startup_final.list）
msh> /sdcard/app/launcher
# 想换一份清单测试:
msh> /sdcard/app/launcher /tmp/my.list
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

## 重要：不要用 @preload / fastboot 快起

新版 SDK 的启动命令默认可能是 `@preload &`（Fast Boot 快起）：此时开机跑的
是**烧在镜像固定位置**的 rtapp 备份（由 U-Boot 预载、ATAG 描述），**根本不读
文件系统**——表现为"删了 /sdcard 下的 ELF 开机照跑、替换也不生效"。
要实现"替换 SD 卡文件即更新"，启动命令必须填**文件系统绝对路径**
（如 `/sdcard/app/launcher`），而不是 `@preload`。

> 源码依据：SDK `src/rtsmart/Kconfig` 中
> `RTT_AUTO_EXEC_CMD`（菜单 "Rtsmart auto execute command string"）
> 在 Fast Boot 开启时默认 `@preload &`；
> 内核 bsp 启动时 `msh_exec(CONFIG_RTT_AUTO_EXEC_CMD, ...)` 执行该字符串
> （`@preload` 开头则走内存预载 rtapp，否则按 msh 命令行处理，文件系统需已挂载）。

## 常见问题

- **开机没启动 launcher**：检查启动命令是否仍是 `@preload`；确认固件非 secure
  boot（secure boot 强制要求命令以 `@preload` 开头，不能用文件路径自启）。
- **某项启动失败**：串口看 `launcher: line N: cannot access ...` 日志，通常是
  路径写错或文件没拷到 SD 卡。
- **顺序依赖**：依赖项用前台（不带 `&`）保证先后；互相独立的常驻服务用 `&`。
- **想要崩溃自动重启**：后台监督循环里对指定路径重新 `fork+execv` 即可扩展。
