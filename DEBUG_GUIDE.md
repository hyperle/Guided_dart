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
| 识别结果一直是 `cx=-1` | 环节 5：启动态没确认出目标（先看 `识别自检` 是否 PASS） | 状态行的 `识别: 状态=启动 …` 与 `全图候选` |
| 状态在「跟踪/丢失」间反复跳 | 环节 5：ROI 太小 / 测量被拒（看 `超框`、`KF拒`） | `识别: 状态 …→…` 那几行 |
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
                     └─ 环节5 识别    DetectionPipeline：启动态全图 RVV 粗筛 + 3 帧滑窗确认 /
                                     跟踪态动态 ROI + 尺度自适应卡尔曼（见 DETECTION_DESIGN.md）
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
⇒ **曝光的位置只有一个**：`include/core/config.hpp::exposure_us`（当前 **500us**），
或命令行 `--exp US` 临时覆盖一档。**改 `apps/green_led_ai/` 里的 `.exposure_us` 与本程序无关**
（那是另一个应用，板子跑的是 `self_guiding_dart`）—— 这里踩过：反复上调那个值，板端日志一直是 250us。

**"全黑"要先看数不看图**：`取证: raw*.pgm 亮度 均值X maxY`。`max` 上不到 **128**（二值化阈值）就必然
"无目标"，即整轮 PBM 全黑。板端实测 250us 时 `max` 只有 1~57、842 张 PBM 里 840 张全黑 —— 那不是故障，
是**进光不够**：先确认光源亮着且在画面里，仍不够就调大曝光（该模式允许 13~11096us；调大会增加运动模糊）。

**注意**

- `binarize` 只收 `width`（要求 stride == width）；VICAP 一旦加行填充会被断言拦住。
- VICAP 源帧与池帧都是 **cached 持久映射**：VICAP 那侧**每帧读前必须 invalidate**（ISP 是 DMA 写入方）。
- 池帧目前**没有 DMA 消费者**（录像走"整块 memcpy 进非 cache 输入块"再喂编码器），所以不需要 flush；
  将来若把池帧直接交给 VENC/KPU，送帧前必须 flush/invalidate。

**病历 4-A（非 cache 映射的性能坑）**：源帧+池帧原来是 non-cache 映射，逐字节读是每字节十几次总线周期。
取证打包一次要 **~12ms**（230K 次迭代），把 90fps 拖到 72.9fps；改成 cached（VICAP 侧每帧 invalidate）后
打包降到 **1.4ms**，帧率回到 84~88fps，且"存图帧 vs 其余帧"从 24.4ms/12.4ms 变成 **11.5ms/11.9ms（已无差别）**。

---

## 5. 环节 5：识别与跟踪

**职责**：`IDetector::detect(GrayFrame&) → DetectResult{cx, cy, roi, cost_us, radius, state, roi_x..roi_h}`。
实现在 `src/detection/`（`DetectionPipeline`），**设计/公式/参数表见 [`DETECTION_DESIGN.md`](DETECTION_DESIGN.md)**。
两句话版本：

- **启动态**：全图 RVV 瓦片粗筛取最亮 Top-K，再用 3 帧滑窗验证位移矢量平滑性（剔除随机闪烁坏点）→ 确认目标；
- **跟踪态**：只在预测 ROI 内扫描（动态 ROI = `kp·ŝ + B_margin + k_σ·σ_pred`），
  测量进 6 维卡尔曼 `[x y vx vy s vs]`（`s = sqrt(A/π)` 是目标等效半径，`vs` 就是膨胀速率），
  `R_scale` 远档大（靠模型压住尺寸跳动）/ 近档小（贴合轮廓变化）。

**成功判据**（日志）

```
self_guiding_dart: 识别自检 PASS（RVV 粗筛 vs 标量 + ROI 测量解析）    ← PASS 之前先别信任何识别结果
self_guiding_dart: 识别/跟踪参数 扫描=rvv-tile(瓦片16 …) ROI: W=kp*s+margin+kσ*σ …
识别: 状态 启动 → 跟踪（第3帧，距上次迁移3帧）：中心(312,206) r=4.2px ROI 55x55@(285,178) …
识别: 状态=跟踪 | 全图帧 3 ROI帧 417 | 命中 417 未命中 0 | 扫描 avg38us max61us 整链 max82us | …
识别总结: 处理 420 帧，命中 417 未命中 3 | 全图扫描 3 帧 ROI 扫描 417 帧 | 确认 1 丢失 0 重捕(软0/硬0) 复位 0 | …
```

| 现象 | 含义 / 动作 |
|---|---|
| `识别自检 FAIL` | RVV 粗筛的向量路径与标量参考不一致（或 ROI 测量解析错）→ **别继续**，先查 `src/detection/scanner/tile_scanner.cpp` 的 RVV 段 |
| 状态一直在「启动」 | 没确认出目标：看 `全图候选`（0 = 阈值/曝光/`--det-min-area` 的问题）；有候选但 `拒闪` 持续涨 = 平滑性门太严（`--confirm-accel`）或目标真的在乱跳 |
| 状态在「跟踪/丢失」间反复跳 | ROI 太小或测量不稳定：看 `超框` 计数（>0 说明目标贴着窗口边）、`KF拒`（>0 说明被马氏门限拒收） |
| `KF拒` 持续增长 | `R_scale` 配得比真实测量噪声小，或目标真的在急机动（发散保护会兜住，但属"正在挣扎"） |
| `贴边跳` 持续增长而没目标 | 正常：镜头前有东西扫过（手/反光）被贴边门挡住了 —— 它**不是**坏点，所以单列不混进 `拒闪` |
| **`全图确认` 持续增长 / `全图帧` 与帧数齐涨** | **ROI 退化成整幅了**（跟踪态本该只在 ROI 内扫）。round13 就是这样抓出来的：近距离目标 r≈40px 时旧的回退判据让它永远走全图。修好后跟踪态 `全图确认` 应恒为 0 |
| `frames_prev.csv` 出现 | 正常：这一轮启动时把上一轮的 CSV 改名保留了（板上多上一次电就多跑一轮，避免静默覆盖）。它的图像已清空，别和本轮 img/ 配对 |
| 参数行与下一行粘在一起 | 已修（`log_line` 缓冲 256→512，且截断时强制保住换行）。若再出现，看 `src/core/log.cpp` |
| `扫描 avg/max` 吃掉帧预算 | 启动态粗筛每帧都跑全图；跟踪态若 ROI 常态很大（`roi_w×roi_h` 接近整幅），查 `roi.kp/margin` |
| CSV 里 `cx=-1` | 这一帧确实没目标（不再像 DetectorStub 时代那样是占位值） |

**耗时字段**：`cost_us` 已改成**微秒**粒度（原来毫秒粒度下"启动阶段粗筛 ~0.15ms"一律显示 0，见旧版此处的记录）。
整链耗时由 `main` 测量，扫描本身的耗时在 1Hz 状态行的 `扫描 avg/max`。

**round13（第一次有目标）实测基线**：目标 r≈40px 时命中 4443/7546 帧、连续跟踪 481 帧（≈5.3s），
但 ROI 几乎没用上（全图 7350 / ROI 196）→ 已修（见上表"整幅回退按面积判"）。

**round12（识别层第一次上板）实测基线**：`识别自检 PASS`；90.0 fps 不掉；全图 RVV 粗筛
`扫描 avg ≈ 505~570us`、整链 p50=528 / p95=631 / max=2713us（帧预算 11109us）。
那一轮画面里没目标（826 张二值图 91% 全黑），暴露并修掉了两个缺陷：贴边候选会被确认为目标、
`log_line` 的 256 字节缓冲截断了参数行（详见 `logs/board/round12_notes.md`）。

**改识别层之后**：三道闸，按顺序过，别跳级。

```bash
bash tests/host/run.sh detection    # ① 逻辑：331 项检查，合成图 + 假时钟，不用板子（标量路径）
bash scripts/rvv_qemu.sh            # ② 向量：交叉编译后在 qemu-riscv64 上跑 RVV 路径（需装 qemu-user）
bash scripts/build.sh self_guiding_dart   # ③ 上板：只剩"真 RVV 时序 + 内存带宽 + 真实曝光"三类
```

① 用的是标量参考路径；② 用 `-cpu rv64,v=true,vlen=128`（对应 C908 的 VLEN=128）把**向量路径**
按板端同一个自检函数跑一遍（`TileScanner::selftest()` 逐位比对 + 解析校验 + 整链 ROI 门控）；
③ 才需要拔卡/上电。装 qemu：`sudo apt install -y qemu-user`（脚本没找到 qemu 会明确提示，
`--build-only` 可只做编译检查）。qemu 的耗时不是板端耗时，性能判据永远看板端日志的
`识别: 状态=… 扫描 avg/max=…us`。

**卡插回来之后的复盘**（一条命令出判据表，不用再翻日志）：

```bash
python3 scripts/detect_review.py /media/$USER/<卡>/dart
```

它读 `frames.csv`，给出：状态时间线（启动/跟踪/丢失 各段帧数与秒数）、跟踪态命中率、
位置轨迹连续性、**等效半径 r 的膨胀速率与线性拟合 R²**（验证"均匀变大"）、ROI 尺寸分布与
占整幅比例、目标是否始终落在窗口内、帧率/断档/重复帧、cost_us 分位数，
最后一张判据表（`[ ok ]/[注意]/[FAIL]`）。`--selftest` 用合成数据自测整条分析链。

---

## 6. 环节 6：取证通道（当前主力产出）

**为什么有它**：本板固件上 VENC 不可用（见 §7），而"识别器看到的图 + 每帧识别结果"才是标定真正需要的。
这条通道**完全不碰编码器**。

**产出**（`/sdcard/dart/`）

**目录布局**（2025-09 起）——图像与 CSV **分层放**：一轮 90s 会产生 800+ 张 PBM，
全堆在一层时 `frames.csv` 会被埋掉，而且板上 RTC 没设、vfat 把所有文件写成同一个时间戳，
按时间排序同样找不到。

| 文件 | 内容 | 默认节奏 |
|---|---|---|
| `dart/frames.csv` | 每帧一行（**放根目录，一眼可见**） | 每帧 |
| `dart/img/f%06lu.pbm` | 二值化图（1bit/像素，bit=1 黑） | `--pbm 9` = 每 9 帧一张（≈10fps） |
| `dart/img/raw%06lu.pgm` | 原始 Y 平面（8bit，无损） | `--raw 90` = 每秒一张 |

> 两个复盘脚本（`pbm_review.py` / `detect_review.py`）都能自动识别新布局，也兼容旧布局
> （图像与 CSV 同层）——归档里的老数据照样能读。

CSV 列：`seq, mono_ms, src_pts, d_pts_us, exp_us, cx, cy, roi, cost_us, captured_fps, pbm, radius, circ, state, roi_x0, roi_y0, roi_x1, roi_y1`

- `d_pts_us` 来自源帧 `pts` —— **真实帧间隔**，用它算出的才是真帧率（"自己数循环次数"只能证明消费了多少帧）。
- 判重复帧要用 `src_pts` 是否与上一帧相同，**不要用图像哈希**（二值图内容恒定时哈希必然相同，我们误报过 2797 处）。
- 后 7 列是识别/跟踪层加的：`radius` 是等效半径 `r=sqrt(A/π)`，`circ` 是目标块的**圆度**（0~1，
  圆目标 0.75~0.95、细长反光 <0.3；`--circ-weight` / `--min-circ` 就是按这一列定的），
  `state` 是 `0=启动 1=跟踪 2=丢失`，
  `roi_x0/roi_y0/roi_x1/roi_y1` 是**本帧实际扫描窗口的四个角像素坐标**（全图扫描时就是整幅
  `0,0,639,359`）。角坐标是**闭区间**：`roi_x1/roi_y1` 是框内最后一个像素 —— 画框、和 PBM
  逐像素对照时可以直接用，不必再做一次 `x+w-1` 的换算（那个 -1 正是最容易记错的地方）。
  标定"确认要几帧、ROI 开多大、尺度跟不跟得住"就靠这几列（见 `DETECTION_DESIGN.md` §6）。

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
| `/sdcard/dart/` | 取证根目录：`frames.csv`（每帧一行）+ 编码器可用时的 `rec_*.h264` |
| `/sdcard/dart/img/` | 图像：`f*.pbm`（二值图）/ `raw*.pgm`（原始 Y 平面） |
| `logs/board/round*` | 本仓归档的各轮真实日志与样本图 |

---

## 10. 常用命令

```bash
# 编译 + 主机侧回归（ASan/UBSan：块状态机 / 编码器自检 / 取证写线程 / 非致命初始化 …）
bash scripts/build.sh self_guiding_dart
bash tests/host/run.sh              # 录像/取证 + 识别/跟踪（331 项检查）
bash tests/host/run.sh detection    # 只跑识别/跟踪（改动 src/detection/ 之后先跑这个）

# 部署：卡在读卡器上（推荐，覆盖文件安全）
cp build/self_guiding_dart/self_guiding_dart /media/$USER/<卡>/app/
cp src/startup.list /media/$USER/<卡>/app/startup_final.list
sync

# 读结果（卡插回来后）
python3 scripts/pbm_review.py /media/$USER/<卡>/dart --check
python3 scripts/pbm_review.py /media/$USER/<卡>/dart --mp4 review.mp4 --dump-overlay first.png
python3 scripts/detect_review.py /media/$USER/<卡>/dart      # 识别/跟踪判据表（状态/ROI/膨胀速率/耗时）
# 两者都会自动找到 dart/frames.csv 与 dart/img/（旧布局也能读）
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

**识别/跟踪旋钮**（同理，改清单一行即生效；完整含义与调参后果见 `DETECTION_DESIGN.md` §8）

| 参数 | 作用 |
|---|---|
| `--det off` | 换回占位识别器（恒无目标），量"识别链路本身的开销"基线 |
| `--no-track` | 关掉 ROI/滤波，只跑"全图粗筛 + 3 帧滑窗确认"（回归对照：现象是否由 ROI 门控引起） |
| `--thr N` | 二值化阈值（默认 128） |
| `--det-topk N` / `--det-min-area N` / `--det-tile N` | 启动态粗筛：Top-K 个数 / 最小面积 / 瓦片边长 |
| `--confirm-window N` / `--confirm-hits N` / `--confirm-accel F` / `--confirm-gate F` | 启动确认：滑窗帧数 / 最少命中 / 位移平滑性门(px) / 关联门(px) |
| `--roi-kp F` / `--roi-margin F` / `--roi-ksigma F` | 动态 ROI：`W=kp·ŝ+margin+kσ·σ_pred`（`--roi-ksigma 0` = 严格规格公式） |
| `--kf-rfar F` / `--kf-rnear F` / `--kf-sfar F` / `--kf-snear F` | 尺度自适应噪声：远/近档 R_scale 与分界尺度（px） |
| `--lost-after N` / `--rescan-after N` | 状态机：几帧无测量算丢失 / 丢失几帧后回头全图重扫 |

> 一轮上电**只测一档参数**：状态迁移与 `扫描 avg/max` 是这一轮唯一的证据来源，
> 同时改两个旋钮就分不清是谁的效果（与 `--venc-mode` 同一条纪律）。

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
