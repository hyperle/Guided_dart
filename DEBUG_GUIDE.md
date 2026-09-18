# 板端调试手册（self_guiding_dart）

> 这份文档是给**下一次调试**用的：先看「一分钟速查」，再顺着「环节框架」逐环节核对；
> 每个环节都挂着我们**实际踩过的坑**（症状 → 定位证据 → 根因 → 修法）。
> 文中结论全部来自板端日志（各轮日志归档在 `logs/board/`），凡属推测都会写明「待验证」。

---

## 0. 一分钟速查

| 你看到的现象 | 最可能是哪个环节 | 直接去看 |
|---|---|---|
| 卡上什么都没有（连 `launcher.log` 都没有） | **环节 0/1**：自启没跑，或板子把 `/sdcard` 挂成只读 | `app/logs/launcher.log` 是否存在 |
| 有 `launcher.log`，没有 `self_guiding_dart.log` | 环节 0/1：exec 失败，或日志目录不可写 | `launcher.log` 里的 `exec … failed` |
| 日志有，但没有状态行（`xx.x fps \|`） | 环节 2/3：卡在初始化或取帧 | 日志最后一行停在哪 |
| 状态行 `无帧 > 0`、fps 很低 | 环节 3：VICAP 取帧失败 | `vicap: 取帧失败 ret=0x…` |
| 二值化图全黑 / 没有白像素 | 环节 4 正常，场景里没有光源（见 §4.1） | `取证: raw… 亮度 均值X maxY` |
| `dart/` 里没有 PBM/PGM/CSV | 环节 6：取证通道没开或写失败 | `取证: …` 那几行 |
| `编码器录像 0 帧` | 环节 7：本板固件上 VENC 不可用（见 §7） | `录像: 自检[…] … 字节 0` |
| 板子重启（`launcher.log` 多次 `boot launcher start`） | 环节 7/6：给卡死通道拆链 / 写线程栈溢出（历史两种） | 日志最后一行 |

**两条最快的健康判据**

- 采集侧：状态行 `无帧 0` + fps ≈ 90（至少 ≥80）；
- 取证侧：`取证: PBM N 张 / PGM M 张，队列丢 0，写失败 0`。

---

## 1. 环节框架（顺着这个顺序排查）

```
上电
 └─ 环节0 自启       固件 auto-exec → /sdcard/app/launcher → 读 startup_final.list → fork/exec 我们的程序
     └─ 环节1 日志落地   log_line 四级兜底（/sdcard/app/logs → /sdcard/app → /sdcard → /tmp）
         └─ 环节2 MPP 初始化  sensor 探测 → VB → VICAP dev/CHN0 → init → 启动取流
             └─ 环节3 取帧      dump_frame → cached 映射 + invalidate → RawFrame 作用域归还
                 └─ 环节4 二值化  RVV 阈值 → 写进池帧（cached）
                     └─ 环节5 识别    IDetector::detect（当前是 DetectorStub 占位）
                         ├─ 环节6 取证通道  PBM(每9帧) / PGM(每90帧) / CSV(每帧)，写线程负责落盘
                         └─ 环节7 编码器录像 VENC（本板固件不可用，用 --no-record 关闭）
                             └─ 环节8 收尾  --seconds 到点 → 写线程收尾 → 主循环退出 → 拆链（有安全阀）
```

---

## 2. 环节 0：自启与启动

**职责**：固件里烧死的 auto-exec 命令 = `/sdcard/app/launcher`；launcher 读
`/sdcard/app/startup_final.list` 逐行 fork/exec，并把子进程 stdout 重定向到
`/sdcard/app/logs/launcher-child.log`。

**成功判据**（`app/logs/launcher.log`）：

```
launcher: boot launcher start, list = /sdcard/app/startup_final.list
launcher: [bg ] line 32: started /sdcard/app/self_guiding_dart (pid=2)
launcher: supervising 1 background app(s)...
```

| 现象 | 含义 / 动作 |
|---|---|
| 整张卡没有任何新文件 | 自启没跑 **或** 板子把卡挂成只读。前者查固件 auto-exec 是否仍指向 launcher；后者检查 **microSD 转接卡套侧面的写保护滑块**（板子看的是卡套，不是 microSD 本体；USB 读卡器通常不接这个引脚，所以"电脑上能写"不代表板子能写） |
| `launcher.log` 里有 `exec … failed` | 目标 ELF 不存在/不可执行：确认 `app/self_guiding_dart` 已部署且 md5 与本地一致 |
| `launcher.log` 出现多次 `boot launcher start` | **板子重启过**：去环节 6/7 看最后一行日志 |
| 清单似乎不生效 | 一次上电**只放一条** `self_guiding_dart`；launcher 的前台项 waitpid 在 RT-Smart 上不可靠 |

**病历 0-A（卡写保护）**：连续多轮"卡上什么都没有、镜头也不热"。根因是 **microSD 装在 SD 转接卡套里、
卡套的写保护滑块在锁位** —— 读卡器不看 WP 引脚，PC 上读写正常，板子却只能只读挂载，于是 mkdir/open 全失败、
一行日志都写不出来。换硬件（换卡/换卡套）后立刻恢复正常。

**病历 0-B（MTP 可用）**：卡不在身边时板子会以 MTP 暴露存储（`gphoto2://Kendryte_CanMV_.../store_ffff0001`），
可直接建目录/放文件；**新增文件可以，覆盖已有文件别用 MTP**（gvfs 会先删后写，历史上把文件弄丢过）。

---

## 3. 环节 1：日志落地

**职责**：`log_line()` 同时写日志文件（每行 `write + fsync`）与 stdout；日志文件四级兜底：
`/sdcard/app/logs/self_guiding_dart.log` → `/sdcard/app/…` → `/sdcard/…` → `/tmp/…`。

**成功判据**：日志第二行直接写明实际生效路径。

```
self_guiding_dart: 日志开始
self_guiding_dart: 日志文件 /sdcard/app/logs/self_guiding_dart.log
```

**注意**

- 看门狗（fork 出的子进程）会重新打开自己的 fd，**可能落到兜底路径**，例如
  `/sdcard/app/self_guiding_dart.log`（内容是 `看门狗: 心跳`，每 2 秒一行）。
  **这个文件也是证据**：它继续写 = 进程与内核都活着；它突然停 = 整机冻住或断电。
- 崩溃留痕：`FAULT: signal NN` 由信号处理器用 `write()` 直接落盘。

---

## 4. 环节 2~4：采集与二值化

**职责**：sensor 探测（回读实际模式）→ VB 就绪 → VICAP dev/CHN0 配置 → init → 启动取流 →
每拍 `dump_frame`（**有界 200ms**）→ cached 映射 + `invalidate` → RVV 二值化写进池帧。

**成功判据**（按顺序出现）：

```
vicap: 实际模式 1280x720@90 type=26      ← 回读值，不是请求值
vicap: VB 就绪
vicap: dev 配置完成（缓冲 6 x 1843200B）
vicap: CHN0 配置 640x360（缩放器开）
vicap: init 完成 (sensor fd=9)
vicap: 启动取流 ...
vicap: 第一帧到手 640x360 stride=640
90.0 fps | 无帧 0 | 丢拍 0 | …            ← 1Hz 状态行
```

### 4.1 二值化图全黑 ≠ 坏了

**判据是亮度统计**（每存一张原始 PGM 打一行）：

```
取证: raw000900.pgm 亮度 均值4.8 min0 max20（曝光 250 us）     ← 场景里没有光源
取证: raw007290.pgm 亮度 均值5.9 min0 max189（曝光 250 us）    ← 光源进画面了
```

**本项目实测**：固定曝光 250µs + 阈值 128 下，背景亮度 ~3~6、绿 LED ~144~189，分离干净；
放上目标后 845 张 PBM 里 **641 张出现白斑**（稳定期质心固定 (312,206)，单帧最多 6725 个白像素）。
⇒ **曝光不要改**；没有目标时"全黑"是正常的，不是故障。

**注意**

- `binarize` 只收 `width`（要求 stride == width）；VICAP 一旦加行填充会被断言拦住。
- VICAP 源帧与池帧都是 **cached 持久映射**：VICAP 那侧**每帧读前必须 invalidate**（ISP 是 DMA 写入方）。
- 池帧目前**没有 DMA 消费者**（录像走"整块 memcpy 进非 cache 输入块"再喂编码器），所以不需要 flush；
  将来若把池帧直接交给 VENC/KPU，送帧前必须 flush/invalidate。

**病历 4-A（非 cache 映射的性能坑）**：源帧+池帧原来是 non-cache 映射，逐字节读是每字节十几次总线周期。
取证打包一次要 **~12ms**（230K 次迭代），把 90fps 拖到 72.9fps；改成 cached（VICAP 侧每帧 invalidate）后
打包降到 **1.4ms**，帧率回到 84~88fps，且"存图帧 vs 其余帧"从 24.4ms/12.4ms 变成 **11.5ms/11.9ms（已无差别）**。

---

## 5. 环节 5：识别

**职责**：`IDetector::detect(GrayFrame&) → DetectResult{cx, cy, roi, cost_us}`。
`main.cpp` 里当前是 `DetectorStub`（永远 `cx=cy=-1`、`cost_us=0`），所以状态行的 `识别 0 us`、
CSV 里的 `cx=-1 cy=-1` 都是**占位值**，不是结论。

**接真识别器**：在 `vision/detector.hpp` 后实现一个类，替换 `main.cpp` 的 `DetectorStub` 即可；
流程、内存、取证、录像都不用动。注意耗时字段是毫秒粒度（`<1ms` 会显示 0），要微秒精度需改 `clock_gettime`。

---

## 6. 环节 6：取证通道（当前主力产出）

**为什么有它**：本板固件上 VENC 不可用（见 §7），而"识别器看到的图 + 每帧识别结果"才是标定真正需要的。
这条通道**完全不碰编码器**。

**产出**（`/sdcard/dart/`）

| 文件 | 内容 | 默认节奏 |
|---|---|---|
| `f%06lu.pbm` | 二值化图（1bit/像素，bit=1 黑） | `--pbm 9` = 每 9 帧一张（≈10fps） |
| `raw%06lu.pgm` | 原始 Y 平面（8bit，无损） | `--raw 90` = 每秒一张 |
| `frames.csv` | 每帧一行 | 每帧 |

CSV 列：`seq, mono_ms, src_pts, d_pts_us, exp_us, cx, cy, roi, cost_us, captured_fps, pbm`

- `d_pts_us` 来自源帧 `pts` —— **真实帧间隔**，用它算出的才是真帧率（"自己数循环次数"只能证明消费了多少帧）。
- 判重复帧要用 `src_pts` 是否与上一帧相同，**不要用图像哈希**（二值图内容恒定时哈希必然相同，我们误报过 2797 处）。

**健康判据**

```
取证: /sdcard/dart/frames.csv（写线程负责落盘）
取证: PBM 845 张 / PGM 84 张，队列丢 0，写失败 0
状态行 … | PBM 840 丢 0 打包max3154us avg1604us | 写线程忙 NNNus/s
```

- `队列丢 > 0`：写线程跟不上（SD 卡慢）——丢的是样本，不影响视觉链路（设计取舍）。
- `打包max/avg`：视觉线程"打包+入队"耗时；`写线程忙`：写线程每秒实际 I/O 耗时（判断是否值得改写入方式）。

**主机侧工具**（`scripts/pbm_review.py`，自带自测）

```bash
python3 scripts/pbm_review.py <目录> --check        # 报告：真实帧率/抖动、重复帧、断档、PBM↔CSV 配对
python3 scripts/pbm_review.py <目录> --mp4 out.mp4 --dump-overlay first.png
python3 scripts/pbm_review.py --selftest            # 合成数据自测整条管线（极性/帧率/叠加）
```

**病历 6-A（写线程栈溢出）**：写线程曾在栈上放 2MB 对象（对象内缓冲尺寸算错成 2MB），RT-Smart 线程栈撑不住
→ `dir_` 被冲成空串（路径变成 `/f000001.pbm`）、**整机重启两次**。修法：元素留在环形槽里、写线程只拿索引
直接写文件；并用 `pthread_attr_setstacksize` 把写线程栈限到 **256KB**，主机测试就在该上限下跑 ——
**以后谁再往写线程里放大对象，测试会当场炸**。

**病历 6-B（CSV 头没 fsync）**：断电后 `frames.csv` 只剩 0 字节，看起来像"通道没工作"，其实是头没落盘。

---

## 7. 环节 7：编码器录像（本板固件现状：不可用）

**设计职责**：录像线程把池帧拷进自己的 VB 输入块 → `send_frame` → `get_stream` → 落盘 `.h264`，
左上角烧入 帧号/时刻/中心/ROI/耗时。

**板端实测结论（这块板 + 这份固件）**

| 观察 | 数值 |
|---|---|
| 通道建立 | chn0 能 attach / create / start |
| 送帧 | `ret=0` 成功（最多 29~4 帧） |
| 码流 | **一个字节都没有**：`start_chn 首批码流 0 字节`、`cur_packs=0 PicCnt=0`（H.264 与 MJPEG 都一样） |
| intbuf | `set_intbuf_size(16MB) ret=0x0` 也无效 |
| 第二个通道 | `create_chn` 一律 `ret=0xa009800d` = **`K_ERR_NOBUF`(13)** ⇒ 本固件 VENC **只容得下一个通道** |
| 拆链 | 给卡死的通道做 `stop/destroy/detach` → **整机重启**（4 次重启都打在这里） |
| MMZ | 开机基线 **511MB**（不是 `config.hpp` 里写的 60MB），内存不是瓶颈 |

⇒ 结论：**编码器子系统在本板固件层面没工作**（官方 sample 的 H.264 走 VICAP→VENC `kd_mpi_sys_bind`，
而老工程 README 记着"CHN1 bind 会拖死 CHN0"，所以没在验证轮里碰）。可行路线：
① 修固件里 VPU/VENC 内部缓冲（intbuf / master save area）配置；② 试 bind 模式（有拖死 CHN0 的风险，需单独一轮）；
③ 用取证通道顶着（当前选择）。

**为这条路留的开关**：`--venc-mode {m|h|i}`（只试 MJPEG / H.264 / H.264+intbuf）、`--venc-chn N`、
`--venc-intbuf MB`、`--venc-probe`。**每个候选独占一个通道、失败了不拆**；一轮只试一档。

**病历 7-A（VB 输入块泄漏）**：早期版本每帧 `get_block` 却从不 `release`，6 块输入池 7 帧漏光 → 录像静默停止。
修法：启动时**一次取满** + 状态机 `free → inflight → free`，**只有对应的码流取回来才允许复用**。

**病历 7-B（逐包 mmap/munmap）**：早期版本对每个码流包做一次 `mmap+munmap`（老工程实测这条会把内核带挂）。
修法：归一化到块基址、按**块大小**做持久 cached 映射（运行期零 mmap/munmap）。

**病历 7-C（无界阻塞送帧）**：`send_frame(-1)` 会把录像线程永远钉死。修法：有界超时 + 失败计数 + 立刻收回块。

**病历 7-D（送过帧却拆链）**：见上表。修法是安全阀：**送过帧但一字节码流都没出 → 退出时不拆链**，
只打日志，把通道/池留给内核，下次上电自清。

---

## 8. 环节 8：收尾

**成功判据**

```
self_guiding_dart: 到达 --seconds 90，开始干净退出
取证: PBM 845 张 / PGM 84 张，队列丢 0，写失败 0
退出(拆链前)：采集 7601 帧，…，编码器录像 0 帧 0.00 MB（…），丢拍 0
退出：采集 7601 帧，…，编码器录像 0 帧 0.00 MB（…）；取证通道另计（见上一行 '取证: PBM …'）
拆链: stop_chn(0) 前 / stop_chn 返回，destroy_chn 前 / … / 输出池已销毁
```

- `--seconds` 是**主循环里的单调钟判断**：板端 `alarm()` 不会送 SIGALRM（实测），SIGALRM 只当额外保险。
- 总结行刻意打在 `stop()` **之前**：拆链一旦把板子带走，这轮统计也不会丢。
- 拆链四步（`stop_chn → destroy_chn → detach_vb_pool → 销毁池`）每步前后各一行，**日志最后一行就是凶手**。
- `编码器录像 0 帧` ≠ "没有产出"：录像指 VENC 那一路（`--no-record` 时为 0），图/CSV 来自取证通道。

**病历 8-A（--mem-slim 回归）**：帧池被压到 6 槽，而 `submit()` 的预留门限硬编码"空闲 > 4"，
主循环+录像线程各占一帧后永远只剩 4 → **每帧都被丢**（录像 0 帧、队列丢 7947）。
修法：预留数按帧池大小算（`frame_slots/4`，最小 1）。

**病历 8-B（看门狗自杀）**：看门狗用 `kill(pid,0)` 判父进程存活，而 RT-Smart 上该调用可能返回"不支持"，
子进程于是秒退、心跳只剩一行。修法：只有 `errno == ESRCH` 才算父进程没了（另加 `getppid()` 判据），
并在 `atexit`/信号处理器里回收子进程。

---

## 9. 目录与路径速查

| 路径 | 内容 |
|---|---|
| `/sdcard/app/self_guiding_dart` | 业务程序（本仓 `src/` 编译产物） |
| `/sdcard/app/launcher` | 开机自启器（本仓 `apps/launcher/`） |
| `/sdcard/app/startup_final.list` | 启动清单（母版 `src/startup.list`） |
| `/sdcard/app/logs/self_guiding_dart.log` | 主日志（每行 fsync） |
| `/sdcard/app/logs/launcher.log` / `launcher-child.log` | 自启器日志 / 子进程 stdout |
| `/sdcard/app/self_guiding_dart.log` | **兜底路径**，常出现看门狗心跳 |
| `/sdcard/app/self_guiding_dart.lock` | 单实例守卫锁文件（fcntl 写锁，进程死自动释放） |
| `/sdcard/dart/` | 取证产出：`f*.pbm` / `raw*.pgm` / `frames.csv`（编码器可用时另有 `rec_*.h264`） |
| `logs/board/round*` | 本仓归档的各轮真实日志与样本图 |

---

## 10. 常用命令

```bash
# 编译 + 主机侧回归（7 个场景，ASan/UBSan：块状态机 / 编码器自检 / 取证写线程 / 非致命初始化 …）
bash scripts/build.sh self_guiding_dart
bash tests/host/run.sh

# 部署：卡在读卡器上（推荐，覆盖文件安全）
cp build/self_guiding_dart/self_guiding_dart /media/$USER/<卡>/app/
cp src/startup.list /media/$USER/<卡>/app/startup_final.list
sync

# 读结果（卡插回来后）
python3 scripts/pbm_review.py /media/$USER/<卡>/dart --check
python3 scripts/pbm_review.py /media/$USER/<卡>/dart --mp4 review.mp4 --dump-overlay first.png
```

**CLI 旋钮**（改 `startup_final.list` 一行即生效，不必重编）

| 参数 | 作用 |
|---|---|
| `--seconds N` | 跑 N 秒后干净退出（建议每轮都带） |
| `--no-record` | 完全不碰 VENC（只跑采集+二值化+识别+取证） |
| `--pbm N` / `--csv` / `--raw N` | 取证：每 N 帧一张 PBM / 每帧一行 CSV / 每 N 帧一张原始 PGM |
| `--acq WxH --fps N` | 传感器档位（gc2093/CSI2 只有 1280x720@90、1280x960@90、1920x1080@30、1920x1080@60） |
| `--frame WxH` | 识别/编码画幅（默认 640x360） |
| `--vision N` | 先纯视觉跑 N 秒再开录像 |
| `--venc-mode {m,h,i}` / `--venc-chn N` / `--venc-intbuf MB` / `--venc-probe` | 编码器排障（一轮只试一档） |
| `--mem-slim` | 收紧我方 VB 池（帧池太小会被预留门限挡住，见病历 8-A） |

---

## 11. 不要做的事（都踩过）

1. **不要**在 `startup_final.list` 里放多条 `self_guiding_dart`（会同时抢 VB/VICAP）。
2. **不要**给"送过帧但没出码流"的 VENC 通道做拆链（会重启整机）。
3. **不要**在写线程/取证线程里放大对象（栈上限 256KB，主机测试会炸）。
4. **不要**用图像哈希判重复帧（用 `src_pts`）。
5. **不要**在视觉线程里直接做文件 I/O（SD 卡会吃光帧预算；I/O 一律交给写线程）。
6. **不要**逐帧 `mmap/munmap`，也不要对同一物理块用不同长度 `munmap`（这块板的老规矩：映射一次，永不解映射）。
7. **不要**一边把 `--pbm` 设得很密一边看 fps：取证密度与帧率是**可调取舍**
   （`--pbm 9` ≈ 84~88fps；去掉 `--pbm` 就是 90fps；更省可 `--pbm 15/30`）。

---

## 12. 下一轮调试的推荐动作（模板）

1. 改 `src/startup.list` 那一行的参数（曝光不动；需要看真实画面就带 `--raw 90`）；
2. `bash scripts/build.sh self_guiding_dart` → `bash tests/host/run.sh`（主机侧先绿）；
3. 卡插读卡器 → 拷 `self_guiding_dart` 与 `startup_final.list` → `sync` → 清空 `app/logs/*` 与 `dart/*`；
4. 卡插板子 → 上电 → 等 `--seconds` 到点自动退出（约 2 分钟）→ 断电 → 卡回读卡器；
5. `python3 scripts/pbm_review.py <卡>/dart --check`，再按「一分钟速查」对着环节表逐条核对；
6. 有新现象就补进本文件的对应环节（症状 → 证据 → 根因 → 修法），下次就不用重新推。
