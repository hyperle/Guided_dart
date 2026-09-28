#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
K230 CanMV(MicroPython) 调参脚本：绿色 LED + 装甲板（两片平行细长发光灯条）

一帧里做两件事：
  1) 绿色发光 LED —— 颜色口径照搬 C++（apps/green_led_ai/detect_color.c）：
     LAB 阈值 (L>=12, A<=-20, B>=8) → 连通域 → 填充率>=55% 且 0.7<=宽高比<=1.4 → 取最大
  2) 装甲板 —— **黑白口径**，不碰灯条颜色：按亮度把窗口二值化（BIN_TH），
     挑出"细长"的亮条（长边 >= 3*短边），两条同向灯条配对成功后：
          A 上角点 ──┐        ┌── B 下角点        这两条连线就是装甲板
          A 下角点 ──┘        └── B 上角点        四边形的两条对角线，
     两连线的交点 = 装甲板中心。
     窗口 = 绿灯**上方**的一块（尺寸随绿灯大小缩放，越近窗口越大）。
     "上/下角点"取灯条**长轴的端点**（竖条=上/下边的中点）。只有取长轴端点，
     两条连线才正好是这个四边形的对角线、交点才是板心；取包围盒外角点则不是。

按"最终要封装成 hpp"的主线分层，本文件就是那个 hpp 的第一个实现：
  §1 参数注入区          → hpp：一组默认值
  §2 结构 / 配置         → hpp：struct LightBar / struct ArmorResult / struct Config
  §3 几何（不碰图像）    → hpp：bar_ends / line_intersection / pair_score /
                                armor_center / pick_armor（可单元测试）
  §4 图像层              → hpp：find_bars / is_bar / find_led（唯一碰图像的地方）
  §5 锁 + 跟踪           → hpp：class Lock / class ArmorTracker（帧间状态，可单测）
  §6 主循环              → hpp 之外（板端胶水）

★ 你要调的两个参数：
  EXPOSURE_US  手动曝光时间(us)。太亮 → 灯条过曝膨胀、长宽比掉到 3 以下；
               太暗 → 灯条断成几截、两端被吃掉、A/B 长度比不合格。
  BIN_TH       二值化阈值（L，0~100）。只看亮度不看颜色。
               调高 → 灯条变细（长宽比容易满足）但变短；调低 → 灯条膨胀变胖，
               还会和周围的亮块粘成一片（包围盒一撑大，长宽比判据立刻失效）。
               怎么定：看着预览窗把 BIN_TH 慢慢往下调，直到灯条被清晰地切出来
               （板体/背景还是黑的），再对着打印行里每条灯条的 aspect / fill
               核一遍（每一条判据的**定义式**见 §1.0 符号表与各参数那行注释）：
               反解长边 L >= BAR_ASPECT_MIN × 反解厚度 t（默认 2）；
               且基本正放时 fill = px/(w×h) >= BAR_FILL_PCT/100（默认 0.50）。

两个参数的生效顺序是硬约束（写反了手动曝光完全不生效）：
  auto_exposure(False) 必须在 sensor.run() **之前**；
  exposure() / again()  必须在 sensor.run() **之后**。

装甲板走 detect-track，**门限只在 detect 需要**（详见 ArmorTracker.plan/update）：
  plan() 给出这一帧走哪条路（跟踪器不认识像素，图像层由主循环执行 ——
  与仓库 C++ 里 TargetTracker::begin_frame → Plan → end_frame 同一套路）：
    detect（未跟踪/重捕）：整块窗 + **全门限**（is_bar 长度/长宽比/填充率/圆度）
                           + O(n²) 配对 + 刚性先验 → 用 find_bars
    track （已跟踪）      ：每条灯条一个窗（或合窗）+ **只认亲**
                           （位置 + 尺寸容差，形状判据一条都不做）→ 用 find_blobs_raw
  判定"还是同一个目标"靠的就是**在它自己的窗里连续变化**；反解只对认到的
  这两条算（端点要用 L 与主轴）。认不到 → 记 miss、标 held → 超 ARMOR_HOLD 退回 detect。
  日志的 `模式=` 就是这一栏（detect/track）。
  日志里 armor 一栏：OK = 本帧实测；HOLDn = 保持了 n 帧的旧中心（不是新测量）；
  MISS = 这一帧没有；无窗口 = 绿灯没锁住（绿灯是装甲板窗口的锚，没锚就不猜）。

工况是**逐渐接近**，所以门限不能是固定像素（详见 gate_bar / ScaleModel）：
  · 尺子 = 绿灯尺度 s（与装甲板刚性连接）：灯条长度门限 = s 的倍数 + 硬地板；
    像素数下限由长度推出来。远→近自动缩放，近距离还会顺带把噪声小亮点挡掉。
  · 尺度是状态：线性一步外推 s_pred = s + K*ds（仓库那套"均匀变大"），
    门限用包络 [min(s,s_pred), max(s,s_pred)] —— 远处目标长得快，单点尺度会把它判掉。
  · 状态机 远档/近档（尺度 < TINY_PX = 远档）：远档**整条跳过形状判据**
    （3px 光的"长宽比"是量化噪声，不是特征），只留像素地板 + FAR_CONFIRM 帧确认
    + 位移平滑；近档形状判据全开，CONFIRM_N 帧就锁。
  · 日志里的 尺= 就是当前尺度，直接告诉你处在哪一档。

斜灯条（"两条灯条平行但不竖直"）—— 包围盒口径会同时废掉两条门限：
  一条 5x40 的灯条转 30°，包围盒变成 24x37 → 长宽比 8.0→1.5、填充率 1.00→0.22，
  于是 BAR_ASPECT_MIN(2) 与 BAR_FILL_PCT(50) **同时**判死它，端点也从"长轴端点"
  退化成"包围盒边中点"（偏出半个包围盒宽）→ 板心整体偏且逐帧抖。
  对策：数据 (w,h,area) 三个数正好对应未知数 (L,t,θ) 三个，**解析反解**（bar_shape）：
      L ∈ [门限] 与 L >= ASPECT_MIN·t 都用反解值（与倾斜无关）；
      倾角 < BAR_FLAT_DEG 时才额外用填充率（斜的时候 fill 天然变低，筛它=筛真灯条）；
      端点改为"中心 ± (L/2)·轴向"（paired_ends），轴向由"两条平行"定分支；
      配对再加两条：倾角差 <= BAR_PARALLEL_DEG（平行的直接用法）、两条一样粗。
  反解失败（不是干净矩形）时退回包围盒口径 = 改造前的行为，宁少精度不丢目标。

刚性几何（已确认：绿灯与装甲板固定在同一块板上）多给两条**与距离无关**的判据
（plate_prior —— 远档形状判据全跳过时，它是唯一还在工作的判据）：
  · 两灯条间距 / 灯尺 ∈ [PLATE_SPAN_LO_PCT, PLATE_SPAN_HI_PCT]
  · 板心到灯心的**径向**距离 < ARMOR_NEAR_K × 绿灯半径（= 同一个整体）
  这些比值是板上固定几何的常数。用日志 OK 行里打印的 "比值(…)" 填 §1 那几个数；
  默认给得很宽（等于不启用），填了才收紧。**同一套比值在 3px 和 300px 时都成立。**

跑法：
  1) 板子：CanMV IDE 打开本文件直接运行。预览窗里 黄框=绿灯、蓝框=装甲板窗口、
     品红框=灯条、青线=两条对角线、红十字=装甲板中心。
     PREVIEW_BIN=1（默认）时画面本身是二值图 —— 就是装甲板检测真正跑的那张，
     调 BIN_TH 时"看着这张图把灯条切出来"最直接；想同时看原始画面用 PREVIEW_BIN=2。
  2) 主机：python3 tools/armor_detect_k230.py —— 只跑几何自测（不需要板子/相机），
     验证"配对 + 对角线交点"这套算法本身没写错。
"""

import math
import time

try:
    from media.sensor import Sensor
    from media.display import Display
    from media.media import MediaManager
    _HAVE_MEDIA = True
except ImportError:
    # 主机侧（CPython）没有 media 模块：只跑 §7 的几何自测
    Sensor = None
    Display = None
    MediaManager = None
    _HAVE_MEDIA = False


# ============================================================================
# §1 参数注入区 —— 改这里即可，改完直接重新运行
# ============================================================================

# ============================ §1.0 符号表 ============================
# 下面每个门限的注释都用这几个量写，**量就是代码里那个量**（改代码时请同步改这里）。
#
#   led        绿灯 blob 的包围盒 (led.x, led.y, led.w, led.h, led.px)
#   s          灯尺 = max(led.w, led.h)                ← py 侧的"目标尺度"
#              C++ 侧用跟踪层给的 s = 2r（等效直径），两者量级一致但不完全相等
#   r          绿灯半径 = s / 2                        ← 只有 *_R / NEAR_K 用半径
#   b          任一候选 blob（包围盒 + b.px 像素数）
#   L,t,tilt   bar_shape(b.w, b.h, b.px) **反解**出的真实长边 / 厚度 / 倾角(0~45°)
#              斜灯条必须用它：5x40 的灯条转 30°，包围盒是 24x37（长宽比 1.5、填充 0.22），
#              而 (L,t,tilt) 仍然是 (39.7, 5.0, 29.6°)
#   fill       = b.px / (b.w × b.h)                    包围盒填充率（无量纲 0~1）
#   asp        = b.w / b.h                             包围盒宽高比
#   ds_ema     尺度逐帧增量 s−s_prev 的 EMA（0.5/0.5）
#   s_pred     = s + clamp(SCALE_V_K × ds_ema, ±SCALE_V_MAX)   线性一步外推
#   s_lo,s_hi  尺度包络 = (min(s, s_pred), max(s, s_pred))     ← 门限用它而不是单点 s
#   span       pair_span(a,b) = 两条灯条中心在**垂直于长轴**方向的距离
#              （竖条取 |a.cx − b.cx|，横条取 |a.cy − b.cy|）
#   score      配对分数（越小越像一对）= 长度差% + 沿长轴错位% + BAR_BORDER_PENALTY×贴边数
#   win        本帧实际扫的窗：整块 / 合窗 / 单条窗（由 ARMOR_PER_BAR_R 决定）
#
# 日志里跟这些量对应的字段：`尺=`(s) `半径=`(r) `L= t= tilt=`(L,t,tilt)
# `条长/尺=`(L/s) `间距/尺=`(span/s) `离心/半径=`(板心到灯心距离/r) `窗=单条x2|合窗`

# ---- ★ 手动曝光 / 二值化（你要调的两个）------------------------------------
EXPOSURE_US = 500       # 手动曝光时间(us)。None = 不设，保持固件默认。
                        # 必须在 sensor.run() **之后**设（之前固件不收）
GAIN_DB = None          # 模拟增益(dB)。None = 不设（保持固件的自动增益）
BIN_TH = 20             # ★装甲板二值化阈值：命中条件 = 像素的 LAB 亮度 L >= BIN_TH
                        # 范围 0~100（LAB 的 L）；A、B 两个色度通道全放开（-128~127）
                        # 所以它是"只看亮度、不看颜色"的黑白口径。作用范围 = win（只在窗口内二值化）
                        # 日志对应：MISS 行里每条候选的 `fill=`、预览二值图（PREVIEW_BIN=1）

# ---- 绿灯颜色阈值（照搬 C++ / 旧脚本，已经调好，一般不用动）------------------
TH_GREEN = (12, 100, -128, -20, 8, 100)
                        # CanMV find_blobs 的 LAB 六元组，命中条件（四个都要满足）：
                        #   12 <= L <= 100   且   -128 <= A <= -20   且   8 <= B <= 100
                        # 即"够亮 + 足够绿(A<=-20) + 不太蓝(B>=8)"。A 越往 -10 放越宽松

# ---- 采集（不要求高帧率，30fps 跑满即可）------------------------------------
FRAME_W = 640           # 传感器输出宽(px)
FRAME_H = 480           # 传感器输出高(px)
SENSOR_FPS = 80         # 给传感器的目标帧率。循环再快也只能跑到它（实测 snap≈33ms 就是等帧）
PREVIEW = True          # True = 走 IDE 预览窗（show_image）；False = 不显示（省 ~17ms/帧）
PRINT_EVERY = 30        # 每处理多少帧打一行统计（30fps 下 ≈ 1 行/秒）
RUN_SELFTEST = False    # True = 板上只跑几何自测、不开相机

# ---- 绿灯 blob 判据（只在 s >= TINY_PX 即"近档"时生效；远档形状是噪声，整条跳过）--
GREEN_PIX_MIN = 30      # 远近档分界之一：led.px < 它 → 远档（要 FAR_CONFIRM 帧确认）
                        # led.px = find_blobs 数到的点数 × GREEN_STEP²（隔点采样换算回真实像素）
GREEN_FILL_PCT = 50     # 判据：led.px×100 >= GREEN_FILL_PCT × led.w×led.h
                        # 即 fill = led.px/(led.w×led.h) >= 0.50（实心光斑，不是散点）
GREEN_ASPECT_LO_NUM = 5 # 判据：led.w×10 >= 5 × led.h → 包围盒宽高比 asp = w/h >= 0.50
GREEN_ASPECT_HI_NUM = 20# 判据：led.w×10 <= 20 × led.h → asp = w/h <= 2.00
                        # 两条一起把绿灯限制成"大致方形/圆形"，长条反光被挡掉
GREEN_STEP = 2          # find_blobs 的 x_stride = y_stride（只在**未锁定**的全图搜索时用；
                        # 锁定后走小窗用 1，免得包围盒被量化 → 打印的 px 也×4 换算过）

# ---- 预览：让 IDE 里看到"检测器实际看到的那张图"（调 BIN_TH 靠它）----------
PREVIEW_BIN = 1         # 0 = 原始帧 + 标注
                        # 1 = **整帧二值图**(img.binary([(BIN_TH,100,-128,127,-128,127)])) + 标注
                        # 2 = 原始帧 + 标注，左上角贴一块放大后的"窗口二值图"
PREVIEW_BIN_ZOOM = 2    # 仅模式 2：窗口二值图的放大倍数（放不下自动退回 1 倍）

# ---- 阶段计时：回答"33.3ms 花在哪一段"（调帧率时开，平时可以关）-------------
TIME_STAGES = True      # 每 PRINT_EVERY 帧打一次 snap/led/armor/show 四段平均 ms

# ---- 尺度自适应：工况是"逐渐接近"，固定像素门限在物理上就不成立 ------------
#   s < TINY_PX（远档）：跳过形状判据，只留像素floor + FAR_CONFIRM 帧确认 + 位移平滑
#   s >= TINY_PX（近档）：形状判据全开，CONFIRM_N 帧就锁
# 注意 TINY_PX 被用在**两处**，同一把尺子量两个对象：
#   ① Lock：led 的长边 max(led.w,led.h) < 它 → 这个测量算"远档"
#   ② is_bar：候选灯条的包围盒长边 max(b.w,b.h) < 它 → 这条灯条算"远档"
TINY_PX = 20            # 远/近档分界(px)，见上面两处用法
GREEN_PIX_FLOOR = 3     # 绿灯像素数**地板**：led.px < 它直接丢（再小不可能是光斑）
                        # 与 GREEN_PIX_MIN 的区别：它是"收不收"，MIN 是"算不算远档"
FAR_CONFIRM = 3         # 远档：连续命中几帧才进 LOCK（近档用 CONFIRM_N）
FAR_JUMP_K = 2.5        # 远档位移平滑门限 = max(FAR_JUMP_MIN, FAR_JUMP_K × 上一帧灯尺)
FAR_JUMP_MIN = 10       # 位移门限下限(px)。判据：|Δcx| > 门限 或 |Δcy| > 门限 → hits 归零重数
                        # Δ 是"本帧测量中心 − 上一帧**报出去**的中心"
SCALE_V_K = 1.0         # 尺度线性外推的系数：s_pred = s + SCALE_V_K × ds_ema（只外推一帧）
SCALE_V_MAX = 4         # 单帧尺度增量上限(px)：防止 ds_ema 被一个坏测量带飞
# 灯条门限 = 灯尺的倍数 + 硬地板（下面的 L 都是反解出来的真实长边）
BAR_LEN_MIN_K = 0.6     # 灯条最短 = BAR_LEN_MIN_K × s_lo（s_lo 是包络下界 → 取宽松的那侧）
BAR_LEN_MAX_K = 1.5     # 灯条最长 = BAR_LEN_MAX_K × s_hi（s_hi 是包络上界）
                        # 两条合起来的实际判据：max(BAR_LEN_FLOOR, 0.6×s_lo) <= L <= max(BAR_LEN_FLOOR, 1.5×s_hi)
BAR_LEN_FLOOR = 2       # 上面两条的硬地板(px)：L < 2px 不成"条"
BAR_PX_FLOOR = 2        # 像素数地板：b.px >= max(BAR_PX_FLOOR, 灯条最短长度 // 2)
                        # （1px 宽的实心条长 L 就有 L 个像素，所以 L/2 是"成条"的下限）

# ---- 刚性几何先验（已确认：绿灯与装甲板固定在同一块板上）--------------------
# 两条判据都与距离无关（比值 / 半径倍数），所以远档形状判据全不可信时它们还在工作。
# 数值用日志里的 `间距/尺=` 与 `离心/半径=` 实测值收紧；现在给得很宽 ≈ 近似不启用。
PLATE_SPAN_LO_PCT = 100     # 判据：span × 100 >= X × s 
PLATE_SPAN_HI_PCT = 500     # 判据：span × 100 <= X × s 
                            # （span = 两灯条中心在垂直于长轴方向的距离；s = 灯尺）
# "是不是同一个整体"：径向判据 —— 板心到灯心的**欧氏距离** < 门限
ARMOR_NEAR_K = 8.0          # 门限 = max(ARMOR_NEAR_K × r, ARMOR_NEAR_MIN_PX)
                            # r = led_radius = max(led.w,led.h)/2
ARMOR_NEAR_MIN_PX = 32      # 门限的像素地板(px)：半径很小时"半径倍数"本身是量化噪声
                            # （r=2 时 8×2=16px 会把真实偏移 21px 的板心判死）

# ---- 灯条形状判据（近档；L/t/tilt 都是反解值，不是包围盒）-------------------
BAR_ASPECT_MIN = 2.5      # 判据：L >= 2 × t（真实长宽比 >= 2，与倾斜无关）
BAR_FILL_PCT = 50       # 判据：b.px×100 >= 50 × b.w×b.h 即 fill >= 0.50
                        # **只在 tilt < BAR_FLAT_DEG（基本正放）时生效**：
                        # 斜的时候包围盒被撑大、fill 天然变低（30° 时 0.22），拿它筛会筛掉真灯条
# ---- 斜灯条（"平行但不竖直"）：包围盒口径会同时废掉长宽比与填充率 ----------
# 一条 5x40 的灯条转 30°，包围盒 24x37：长宽比 8.0→1.5、填充率 1.00→0.22，
# 于是"长宽比"和"填充率"**同时**失效。所以斜的时候改用反解出来的 L/t（见 bar_shape），
# 端点也从"包围盒边中点"改成"中心 ± (L/2)·主轴方向"（见 paired_ends）。
BAR_FLAT_DEG = 12       # tilt < 它 → 认为基本竖直/水平：启用 fill 判据 + 用包围盒口径取端点
BAR_PARALLEL_DEG = 15   # 判据：|tilt_a − tilt_b| <= 15°（"两条灯条平行"这条事实的用法）
BAR_THICK_MIN_RATIO_NUM = 3  # 判据：min(t_a,t_b) × 10 >= 3 × max(t_a,t_b)（两条一样粗，比值>=0.3）
BAR_BORDER_PENALTY = 30     # 贴搜索窗边（长度只是下界）的灯条，在 score 里加 30/条。
                            # score 的单位是"百分数"（长度差% + 错位%），所以 30 ≈ 多算 30% 长度差

BAR_LEN_RATIO_PCT = 40  # 判据：min(L_a,L_b) × 100 >= 40 × max(L_a,L_b)（两条长度比 >= 0.40）
BAR_OVERLAP_PCT = 40    # 判据：(hi − lo) × 100 >= 40 × min(L_a,L_b)
                        # 其中 [lo,hi] 是两条灯条**包围盒在长轴方向上的区间**求交
                        # （竖条：lo = max(a.y,b.y)，hi = min(a.y+a.h, b.y+b.h)）
                        # 含义：并排，不是首尾相接
BAR_GAP_MAX_PCT = 1000  # 判据：sep × 100 <= 1000 × max(L_a,L_b) 即 sep <= 10×较长的那条
                        # sep = 垂直于长轴的中心距（竖条 = |a.cx − b.cx|）
DIAG_MIN_SIN_PCT = 30   # 判据：|sin∠(两条对角线)| >= 0.30
                        # 两条对角线 = A上角点—B下角点 与 A下角点—B上角点；
                        # sin 用叉积算：|d| / (|a|×|b|)，d = ax·by − ay·bx。
                        # 夹角太小（近平行）时交点对 1px 抖动极敏感 → 宁可不出结果

# ---- 装甲板窗口 = 绿灯上方那块（首次/重捕/远距离用；尺寸随绿灯包围盒缩放）----
ARMOR_ROI_W_K = 3.0     # 窗口宽 = 3.0 × led.w（再取 max(..., ROI_MIN_SIDE)）
ARMOR_ROI_H_K = 2.0     # 窗口高 = 2.0 × led.h（同上）
ARMOR_ROI_GAP_K = 0.0   # 窗口下沿离绿灯上沿的间隔 = GAP_K × led.h
                        # 窗口 y 区间 = [led.y − gap − 高, led.y − gap]
                        # 0 = 下沿正好贴绿灯上沿（默认）；给到 1.0 会把整块装甲板顶出窗口（踩过）

# ---- 装甲板 detect-track：静止画面下结果乱跳的对策 ---------------------------
ARMOR_HOLD = 2          # 关联失败（这一帧没配成对）时，先返回上一次的中心最多几帧，
                        # 日志标 HOLD<n>；n > ARMOR_HOLD 就释放并整块重捕
ARMOR_ASSOC_K = 0.6     # 关联容差：tx = max(ARMOR_ASSOC_K × ref.w, ARMOR_ASSOC_MIN)，
                        # ty 同理用 ref.h（ref = 上一帧那一条灯条的包围盒）
                        # 判据：|Δcx|<=tx 且 |Δcy|<=ty 且 |Δw|<=tx 且 |Δh|<=ty 才算"还是它"
ARMOR_ASSOC_MIN = 4     # 上面的容差下限(px)
# ---- 单条灯条 ROI（绿灯够大时才开）：解决"合窗在某些角度包不住装甲板" ----------
# 合窗要同时盖住两条灯条 + 中间空白 + 转动后的外接矩形变化 → 尺寸被"板跨度"绑住，一斜就切。
# 单条窗只盖住它自己，与板跨度无关；中间的空白也不再付像素。
ARMOR_PER_BAR_R = 20.0   # 门限：r >= 它 → 每条灯条各开一个窗；否则开一个合窗
ARMOR_BAR_MARGIN_K = 0.25  # 单条窗 margin = 0.25 × L（**该灯条自己的长度**，不是整对的尺寸）
ARMOR_BAR_MARGIN_MIN = 8   # margin 下限(px)：盖住帧间位移与小幅转动
ARMOR_BAR_MARGIN_MAX = 24  # margin 上限(px)：不封顶的话"板比窗口大"时单条窗反而更贵
                           # （实测 L220/gap175：不封顶 67341px > 旧窗 38400px；封到 24 → 37520px）
                           # 最终 margin = clamp(int(0.25×L), 8, 24) + 增长补偿
                           # 增长补偿 = int(L × (本帧r / 上次采到那一对时的r − 1) × 0.5)
                           # （逐渐接近时目标每帧长大，不补就会被切：实测 ×1.35/帧 时长度只剩 0.58）

# ---- 锁（绿灯的）------------------------------------------------------------
CONFIRM_N = 2           # 近档：连续命中几帧才进 LOCK（hits >= 它）
                        # 远档用 FAR_CONFIRM 代替它
MISS_LIMIT = 3          # 连续 miss 几帧就释放锁（misses >= 它 → locked=False，回全图）
MARGIN_K = 0.8          # 锁定后绿灯搜索窗 = 上一帧报出的包围盒 外扩 m，
                        # m = max(int(0.8 × max(box.w, box.h)), ROI_MIN_MARGIN)
LED_DEADBAND = 2        # 死区(px)：|本帧x − 上次报出的x| <= 2 且 |Δy| <= 2 → 继续用旧框。
                        # 否��� ±1px 抖动会让下游窗口边界一帧一帧地蹭在灯条上（把灯条截断）
ROI_MIN_MARGIN = 12     # 上面 m 的下限(px)：目标很小时也得留出运动余量
ROI_MIN_SIDE = 24       # 窗口边长地板(px)：**用于** 绿灯搜索窗、绿灯上方整块窗、合窗；
                        # **不用于**单条窗（单条窗的地板是 ARMOR_BAR_MARGIN_MIN=8，
                        # 拿 24 卡它会让 r≈20 时单条窗静默失效 —— 踩过）

SEARCH_ROI = (0, 0, FRAME_W, FRAME_H)   # 未锁定时绿灯的全图搜索范围


# ============================================================================
# §2 结构 / 配置（hpp：struct LightBar / struct ArmorResult / struct Config）
# ============================================================================

class Box(object):
    """一个亮块的包围盒。绿灯和灯条共用（hpp: struct LightBar，多一个 px）。"""

    def __init__(self, x, y, w, h, px):
        self.x = int(x)
        self.y = int(y)
        self.w = int(w)
        self.h = int(h)
        self.px = int(px)          # 像素数（find_blobs 的 pixels()）
        self.cx = self.x + self.w // 2
        self.cy = self.y + self.h // 2
        self._shp = None           # (L, t, 倾角°) 缓存：bar_shape() 只算一次
        self.border = 0            # 1 = 包围盒贴着**搜索窗**边 → 长度/面积只是下界

    def shape(self):
        """灯条的真实 (长 L, 厚 t, 倾角°)。反解失败 → 退回包围盒口径（倾角 0）。

        反解一次就缓存：is_bar / pair_score / 端点 / 打印都要用，
        O(n²) 的配对里重复反解是白花钱（而且数学上完全一样）。
        """
        if self._shp is None:
            shp = bar_shape(self.w, self.h, self.px)
            if shp is None:
                # 反解不出来：说明它不是个干净的矩形。退回包围盒口径 = 旧行为，
                # 宁可少一点精度也不能因此丢掉真灯条（倾角记 0 = 按竖直处理）。
                shp = (float(self.w if self.w > self.h else self.h),
                       float(self.h if self.w > self.h else self.w), 0.0)
            self._shp = shp
        return self._shp


class Config(object):
    """配置对象（hpp: struct Config）。

    **字段名 = §1 里常量名的小写**（BIN_TH → cfg.bin_th），由 __init__ 自动生成，
    所以这里再也不用手写第二份字段/默认值清单。这一条是踩过坑才改的：
    手写清单时 find_led 用了 cfg.green_aspect_lo_num，而 Config 里忘了声明这一项，
    板子上跑到第 427 行才炸。现在只要 §1 有那个常量，字段就一定在；
    缺了会在构造时立刻炸（而不是几十帧以后在某个分支里炸）。

    想临时改一个值（主机侧测试用）：Config(bin_th=40) —— 不改 §1 的常量。
    """

    def __init__(self, **over):
        for k, v in globals().items():
            # 只看 §1 的常量：全大写、不带下划线前缀（_HAVE_MEDIA 那种私有量排除）
            if k.isupper() and not k.startswith('_'):
                setattr(self, k.lower(), v)
        for k, v in over.items():
            setattr(self, k, v)


def make_config():
    """板端唯一的配置入口：快照 §1 的常量。"""
    return Config()


# ============================================================================
# §3 几何：纯函数，不碰图像、不用浮点（hpp：可单元测试的那一层）
# ============================================================================

def _rdiv(num, den):
    """四舍五入的整除（den > 0）。整数进整数出，全程没有浮点误差。"""
    return (2 * num + den) // (2 * den)


def _absdiff(a, b):
    """|a - b|（整数，避免到处写三目表达式）。"""
    return a - b if a > b else b - a


def est_px(px, step):
    """隔点采样下的**真实像素数**估算（C++ 版里就是 est_px = n*sx*sy）。

    find_blobs 用 x_stride/y_stride 隔点采样时，blob.pixels() 只数采样点：
    一个 3x3 的真目标在 step=2 下只数到 1~4 个点。拿它去比门限 = 把远距离目标
    按 step² 倍缩水判掉（step=2 就是 4 倍）。所以像素数一律按这个函数换算，
    包围盒（w/h）不用换算 —— 采样点的 min/max 仍是真实边界。
    """
    return px * step * step


def bar_shape(w, h, area):
    """从包围盒 + 像素数**反解**斜灯条的真实长短边 → (L, 厚度t, 倾角°)，失败 None。

    为什么需要它：包围盒对旋转极其敏感。一条 5x40 的灯条转 30°，包围盒变成 24x37 ——
    长宽比 8.0→1.5、填充率 1.00→0.22，于是"长宽比 >= 2"和"填充率 >= 50%"**同时**失效，
    这就是"灯条平行但不竖直时识别差"的全部机械原因（端点也一起错了，见 paired_ends）。

    数据只有 (w, h, area) 三个数，未知数也正好三个 (L, t, θ)：
        w = L·|sinθ| + t·|cosθ|      h = L·|cosθ| + t·|sinθ|      area = L·t
    记 S=w+h、P=w·h、s=|sinθ|+|cosθ| ∈ [1,√2]、z=s²，消元得
        2·area·z² + (2P − 4·area − S²)·z + S² = 0
    取 [1,2] 内的根 → u = L+t = S/√z → 再由 L·t = area 解出 L、t。
    理想矩形上这是**精确**的（实测 40x5 转 30° 反解误差 <0.5px、<0.5°）。

    两条口径边界（调用方都要知道）：
      · 倾角只有**幅值**：θ 与 90°−θ 的包围盒一模一样 → 轴向由 w/h 谁大决定（见 bar_axis）；
      · 反解失败（判别式<0 或根不在 [1,2]）→ 返回 None，调用方退回包围盒口径
        （宁可少一点精度，也不能因为反解不出来就丢掉真灯条）。
    """
    if area <= 0 or w <= 0 or h <= 0:
        return None
    S = float(w + h)
    P = float(w * h)
    A = float(area)
    b = 2.0 * P - 4.0 * A - S * S
    disc = b * b - 8.0 * A * S * S
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    for z in ((-b + sq) / (4.0 * A), (-b - sq) / (4.0 * A)):
        if z < 1.0 - 1e-6 or z > 2.0 + 1e-6:
            continue
        if z < 1.0:
            z = 1.0
        elif z > 2.0:
            z = 2.0
        u = S / math.sqrt(z)                       # L + t
        if u * u < 4.0 * A:
            continue
        q = math.sqrt(u * u - 4.0 * A)
        L = 0.5 * (u + q)
        t = 0.5 * (u - q)
        if t <= 0.0:
            continue
        return (L, t, math.degrees(0.5 * math.asin(z - 1.0)))   # 倾角 0~45°
    return None


def bar_axis(b, sign):
    """灯条轴向单位向量（指向画面"上方"）。sign = ±1 选镜像分支。

    反解只给倾角**幅值**（θ 与 90°−θ 的包围盒完全一样），所以轴向这样定：
      · h > w（偏竖）→ 偏离竖直 θ；
      · w > h（偏横）→ 偏离竖直 90°−θ；
      · w == h（45° 附近）→ 45°。
    """
    L, t, tilt = b.shape()
    if b.w > b.h:
        tilt = 90.0 - tilt
    elif b.w == b.h:
        tilt = 45.0
    r = math.radians(tilt) * sign
    return (math.sin(r), -math.cos(r))


def bar_len(b):
    """灯条长边长度（反解后的真实 L；反解失败等价于包围盒长边）。"""
    return b.shape()[0]


def bar_thick(b):
    """灯条厚度（反解后的真实 t；"两条一样粗"这条判据用它）。"""
    return b.shape()[1]


def pair_span(a, b):
    """两条灯条中心在"垂直于长轴"方向上的距离。

    刚性几何下它 ∝ 绿灯尺度：这是"板上两灯条间距"这个物理量的像素化读数。
    """
    if a.h >= a.w:          # 竖条：横向间距
        return _absdiff(a.cx, b.cx)
    return _absdiff(a.cy, b.cy)


def bar_ends(bar):
    """灯条长轴的两个端点 = 你的"上角点 / 下角点"。

    竖条（h>=w）：上边中点、下边中点；横条：左边中点、右边中点。
    为什么取"边的中点"而不是"包围盒角点"：一对灯条的上/下角点围成的是装甲板
    那个四边形，只有取长轴端点，A上—B下 与 A下—B上 才正好是它的两条对角线，
    交点才是板心。取外角点会把交点整体拽偏。
    """
    if bar.h >= bar.w:
        return (bar.cx, bar.y), (bar.cx, bar.y + bar.h - 1)
    return (bar.x, bar.cy), (bar.x + bar.w - 1, bar.cy)


def line_intersection(p0, p1, p2, p3, min_sin_pct=15):
    """线段 p0p1 与 p2p3 的交点；不平行且交点同时落在两条线段内才返回 (x, y)，否则 None。

    全整数运算：d/t/u 都是有符号整数，只有最后求坐标才做一次四舍五入的除法。
    好处是"交点是否在段内"是精确比较 —— 用浮点会在边界情形上判反，
    而边界情形（角点几乎重合）恰好是噪声最大的地方。

    两条对角线夹角太小时（|sin| < min_sin_pct/100）直接判不合格：
    近平行线的交点对 1px 的角点抖动极其敏感，宁可不报也不要报一个乱跳的中心。
    """
    x0, y0 = p0
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    ax, ay = x1 - x0, y1 - y0          # 线段 1 的方向
    bx, by = x3 - x2, y3 - y2          # 线段 2 的方向、2 的起点

    d = ax * by - ay * bx              # 叉积（= |a||b|sinθ，符号带方向）
    if d == 0:
        return None                    # 平行或共线：没有唯一交点

    # |d|^2 >= (min_sin)^2 * |a|^2 * |b|^2，两边同乘 10000 避免开方/浮点
    ms = min_sin_pct
    if 10000 * d * d < ms * ms * (ax * ax + ay * ay) * (bx * bx + by * by):
        return None

    # t = tn/d、u = un/d 是两条线上的参数；统一把 d 取正，后面比较就不用管符号
    tn = (x2 - x0) * by - (y2 - y0) * bx
    un = (x2 - x0) * ay - (y2 - y0) * ax
    if d < 0:
        d = -d
        tn = -tn
        un = -un
    if tn < 0 or tn > d or un < 0 or un > d:
        return None                    # 交点在延长线上 → 不是这个四边形的中心

    return (x0 + _rdiv(ax * tn, d), y0 + _rdiv(ay * tn, d))


def pair_score(a, b, cfg):
    """两条灯条像不像同一块装甲板：像 → 分数（越小越像），不像 → None。

    门限（全部是"两条同属一块板"的必要条件；符号定义见 §1.0）：
      ①   同向：a 与 b 都偏竖 或 都偏横（按包围盒 h>=w 判；一竖一横不可能是同一块板）
      ①b  平行：|tilt_a − tilt_b| <= BAR_PARALLEL_DEG
      ①c  等粗：min(t_a,t_b) × 10 >= BAR_THICK_MIN_RATIO_NUM × max(t_a,t_b)
      ②   等长：min(L_a,L_b) × 100 >= BAR_LEN_RATIO_PCT × max(L_a,L_b)
      ③   重叠：(hi − lo) × 100 >= BAR_OVERLAP_PCT × min(L_a,L_b)
           [lo,hi] = 两条的**包围盒在长轴方向上**的区间求交（竖条用 y 区间）
      ④   间距：sep × 2 > (w_a + w_b)（横向不重叠 = 在二值图上本来是同一块的会被并掉）
           且 sep × 100 <= BAR_GAP_MAX_PCT × max(L_a,L_b)
           sep = 垂直于长轴的中心距（竖条 = |a.cx − b.cx|）
    分数 = (max(L)−min(L))×100/max(L) + 沿长轴错位×100/max(L) + BAR_BORDER_PENALTY×贴边条数
    其中 L/t/tilt 都是 bar_shape() **反解**出来的真实量（包围盒口径斜的时候会同时失效）。

    这里**不再**管"单条长度/像素数"：能进到这一步的灯条都已经过 is_bar
    （门限随尺度变），再按固定像素数筛一次就是重复且会漏掉远距离目标。
    注意配对门限（②③④）全是**相对量**（比值/倍数），天生不随距离失效。
    """
    a_vert = a.h >= a.w
    if a_vert != (b.h >= b.w):
        return None                                     # ① 必须同向（偏竖/偏横不能配）
    # ①b **倾角要一致**：这就是"两条灯条平行"这条已知事实的用法。
    #     只比"都竖"是不够的——两条灯条都歪着、但歪的角度差很多，不是一块板。
    ta = a.shape()[2]
    tb = b.shape()[2]
    if (ta - tb if ta > tb else tb - ta) > cfg.bar_parallel_deg:
        return None
    # ①c **两条一样粗**：同一块板上两条灯条厚度相同 → 厚度比卡一道，
    #     能挡掉"灯条上糊了一块亮斑"这种被撑变形的块（厚度被撑大）。
    ka = a.shape()[1]
    kb = b.shape()[1]
    tmax = ka if ka > kb else kb
    tmin = kb if ka > kb else ka
    if tmin * 10 < cfg.bar_thick_min_ratio_num * tmax:
        return None

    if a_vert:
        la, lb = bar_len(a), bar_len(b)        # **反解后的真实长边**（bbox 长边在斜条上偏短）
        sep = a.cx - b.cx if a.cx > b.cx else b.cx - a.cx
        half = a.w + b.w
        lo = a.y if a.y > b.y else b.y                  # 沿长轴投影重叠
        hi_a, hi_b = a.y + a.h, b.y + b.h
        off = a.cy - b.cy if a.cy > b.cy else b.cy - a.cy
    else:
        la, lb = bar_len(a), bar_len(b)
        sep = a.cy - b.cy if a.cy > b.cy else b.cy - a.cy
        half = a.h + b.h
        lo = a.x if a.x > b.x else b.x
        hi_a, hi_b = a.x + a.w, b.x + b.w
        off = a.x - b.x if a.x > b.x else b.x - a.x
    hi = hi_a if hi_a < hi_b else hi_b

    lmax = la if la > lb else lb
    lmin = lb if la > lb else la

    if lmin * 100 < cfg.bar_len_ratio_pct * lmax:       # ② 长度比
        return None
    if (hi - lo) * 100 < cfg.bar_overlap_pct * lmin:    # ③ 投影重叠
        return None
    if sep * 2 <= half:                                 # ④ 横向重叠 → 同一块
        return None
    if sep * 100 > cfg.bar_gap_max_pct * lmax:          # ④ 离太远
        return None
    return ((lmax - lmin) * 100 // lmax + off * 100 // lmax
            + cfg.bar_border_penalty * (a.border + b.border))


def paired_ends(a, b, cfg):
    """一对灯条的四个角点 (a0, a1, b0, b1)：各自沿**自己的轴向**取长轴端点。

    为什么不能用包围盒边的中点：一条 5x40 的灯条转 30°，包围盒上边中点根本不在
    灯条轴上（偏出去半个包围盒宽），四条"角点"于是围不成那块四边形，
    板心会整体偏、而且逐帧抖。这里改成：中心 ± (L/2)·轴向单位向量。

    轴向的镜像分支（±）由**两条平行**这条事实定：取让两条轴最平行的那一组分支。
    分支选反了只会把四个角点镜像一下，交点位置不变（两条对角线还是那两条线段），
    所以这一步是稳的。

    倾角基本为 0（< bar_flat_deg）时直接用包围盒口径 —— 与改造前逐位一致，
    保证"灯条竖直"这个已经跑通的工况零回归。
    """
    La, ta, tila = a.shape()
    Lb, tb, tilb = b.shape()
    if tila < cfg.bar_flat_deg and tilb < cfg.bar_flat_deg:
        a0, a1 = bar_ends(a)
        b0, b1 = bar_ends(b)
        return a0, a1, b0, b1

    best = None
    for sa in (1, -1):
        na = bar_axis(a, sa)
        for sb in (1, -1):
            nb = bar_axis(b, sb)
            dot = na[0] * nb[0] + na[1] * nb[1]
            if dot < 0.0:
                dot = -dot
            if best is None or dot > best[0]:
                best = (dot, na, nb)
    _, na, nb = best
    ha = La * 0.5
    hb = Lb * 0.5
    a0 = (int(round(a.cx + na[0] * ha)), int(round(a.cy + na[1] * ha)))
    a1 = (int(round(a.cx - na[0] * ha)), int(round(a.cy - na[1] * ha)))
    b0 = (int(round(b.cx + nb[0] * hb)), int(round(b.cy + nb[1] * hb)))
    b1 = (int(round(b.cx - nb[0] * hb)), int(round(b.cy - nb[1] * hb)))
    return a0, a1, b0, b1


def armor_center(a, b, cfg):
    """装甲板中心 = A 上角点—B 下角点 与 A 下角点—B 上角点 两条连线的交点。

    注意是**交叉**连线：a 自己的长轴和 b 自己的长轴永远平行，拿它们求交没有交点
    （这条写错过一次）。交点落在两条线段之内由 line_intersection 保证。
    """
    a0, a1, b0, b1 = paired_ends(a, b, cfg)
    return line_intersection(a0, b1, a1, b0, cfg.diag_min_sin_pct)


def _same_box(bars, b):
    """同一个块是否已经在列表里（两个单条窗万一重叠，collect 会带出重复块）。"""
    for o in bars:
        if o.x == b.x and o.y == b.y and o.w == b.w and o.h == b.h:
            return True
    return False


def _win_name(wins):
    '''日志用：本帧走的是"单条窗"还是"合窗"（门限 = ARMOR_PER_BAR_R）。'''
    if not wins:
        return '窗=无'
    if len(wins) > 1:
        return '窗=单条x%d' % len(wins)
    return '窗=合窗'


def _union_roi(wins):
    """多个窗的并集（只用于日志与"整块"参考，不参与搜索）。"""
    x0 = wins[0][0]
    y0 = wins[0][1]
    x1 = wins[0][0] + wins[0][2] - 1
    y1 = wins[0][1] + wins[0][3] - 1
    for w in wins[1:]:
        if w[0] < x0:
            x0 = w[0]
        if w[1] < y0:
            y0 = w[1]
        if w[0] + w[2] - 1 > x1:
            x1 = w[0] + w[2] - 1
        if w[1] + w[3] - 1 > y1:
            y1 = w[1] + w[3] - 1
    return (x0, y0, x1 - x0 + 1, y1 - y0 + 1)


def dist(x0, y0, x1, y1):
    """两点距离。**不要用 math.hypot** —— MicroPython 的 math 没有它
    （CanMV v1.8 上直接 AttributeError，板上跑到日志那一行才炸）。
    math.sqrt / sin / cos / asin / radians / degrees 都有，可以放心用。
    只比大小的地方用平方比较（省一次开方），要打印/显示的地方才开方。"""
    dx = x0 - x1
    dy = y0 - y1
    return math.sqrt(dx * dx + dy * dy)


def led_radius(led):
    """绿灯半径（等效）：py 侧只有包围盒 → 取长边的一半。C++ 侧直接用跟踪层的 r。
    led=None（主机侧单测可以不带锚）→ 0。"""
    if led is None:
        return 0.0
    return (led.w if led.w > led.h else led.h) * 0.5


def same_body(cfg, led, cx, cy):
    """板心与绿灯是不是**同一个整体**：径向距离 < ARMOR_NEAR_K × 绿灯半径。

    为什么用径向而不是矩形框（原来是"上高/横偏"两个方向约束）：**距离与倾角无关**。
    板子斜着转的时候，矩形框会把真目标切掉；径向判据不会。而且只有一个旋钮，
    用日志里的 "离心/半径" 实测值收紧即可。
    绿灯半径还没建立（<=0）时放行 —— 那时没有可比的基准。
    """
    r = led_radius(led)
    if r <= 0:
        return True
    dx = cx - led.cx
    dy = cy - led.cy
    lim = cfg.armor_near_k * r
    if lim < cfg.armor_near_min_px:
        lim = cfg.armor_near_min_px             # 小半径下按倍数算会被量化噪声判死
    return dx * dx + dy * dy <= lim * lim       # 平方比较，省一次开方


def plate_prior(cfg, led, s, a, b, c):
    """刚性几何先验：① 两灯条间距 ∝ 灯尺（板的**尺寸**是固定的）
                     ② 板心离灯心 < ARMOR_NEAR_K × 半径（是不是**同一个整体**）

    前提是"绿灯与装甲板固定在同一块板上"（已确认）。两条都是**与距离无关**的判据
    （比值/半径倍数），所以远档形状判据全不可信时它们是唯一还在工作的判据。
    间距那两个数是比值（百分数），默认很宽 = 基本不启用，用日志里的比值收紧。
    s <= 0（灯尺没建立）时只做②。
    """
    if not same_body(cfg, led, c[0], c[1]):
        return False
    if s <= 0:
        return True
    span = pair_span(a, b)
    if span * 100 < cfg.plate_span_lo_pct * s or span * 100 > cfg.plate_span_hi_pct * s:
        return False
    return True


def pick_armor(bars, cfg, led=None, s=0):
    """灯条列表 → (bar_a, bar_b, (cx, cy)) 或 None。取分数最小的一对。

    这是**无状态**的一帧判决：每帧从零重选，所以两对分数接近时会逐帧换人。
    要"稳"必须配合 ArmorTracker（先关联、再保持、最后才重选）。

    led/s 给了就用刚性几何先验筛一遍（见 plate_prior）；只在这一步筛，
    ArmorTracker 的关联/保持路径不再筛 —— 那一对在 detect 时已经过关了。
    """
    best = None
    n = len(bars)
    for i in range(n):
        for j in range(i + 1, n):
            a, b = bars[i], bars[j]
            sc = pair_score(a, b, cfg)
            if sc is None:
                continue
            c = armor_center(a, b, cfg)
            if c is None:
                continue
            if led is not None and not plate_prior(cfg, led, s, a, b, c):
                continue
            if best is None or sc < best[0] or (
                    sc == best[0] and a.px + b.px > best[1].px + best[2].px):
                best = (sc, a, b, c)
    if best is None:
        return None
    return (best[1], best[2], best[3])


def clamp_roi(roi, img_w, img_h, min_side):
    """把窗口裁进画面。裁完任一边 < min_side 就返回 None（= 本帧没有可用窗口）。"""
    x, y, w, h = int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3])
    if x < 0:
        w += x
        x = 0
    if y < 0:
        h += y
        y = 0
    if x + w > img_w:
        w = img_w - x
    if y + h > img_h:
        h = img_h - y
    if w < min_side or h < min_side:
        return None
    return (x, y, w, h)


def intersect_roi(r1, r2):
    """两个窗口的交；不相交返回 None（跟踪小窗不能跑到装甲板窗口之外）。"""
    x0 = r1[0] if r1[0] > r2[0] else r2[0]
    y0 = r1[1] if r1[1] > r2[1] else r2[1]
    x1a, x1b = r1[0] + r1[2], r2[0] + r2[2]
    y1a, y1b = r1[1] + r1[3], r2[1] + r2[3]
    x1 = x1a if x1a < x1b else x1b
    y1 = y1a if y1a < y1b else y1b
    if x1 <= x0 or y1 <= y0:
        return None
    return (x0, y0, x1 - x0, y1 - y0)


def armor_roi_from_led(led, img_w, img_h, cfg):
    """装甲板窗口 = 绿灯正上方的一块，尺寸跟着绿灯大小缩放。

    纵向 [led.y - gap - h, led.y - gap]：整个窗口都在绿灯上沿之上（不重叠）。
    gap = 0（默认）时窗口下沿正好贴着绿灯上沿 —— 装甲板就压在绿灯正上方，
    窗口从绿灯上沿往上长，才不会有灯条落到窗口外。
    横向以灯心为中心。出画就裁；裁得没意义就返回 None（本帧不检测装甲板）。

    注意：窗口只用"本帧实测"的绿灯位置。绿灯这一帧没测到就宁可不出装甲板结果，
    也不拿上一帧的旧位置开窗 —— 旧位置开的窗里出现的"灯条"是假的。
    """
    rw = int(led.w * cfg.armor_roi_w_k)
    rh = int(led.h * cfg.armor_roi_h_k)
    gap = int(led.h * cfg.armor_roi_gap_k)
    if rw < cfg.roi_min_side:
        rw = cfg.roi_min_side
    if rh < cfg.roi_min_side:
        rh = cfg.roi_min_side
    return clamp_roi((led.cx - rw // 2, led.y - gap - rh, rw, rh),
                     img_w, img_h, cfg.roi_min_side)


# ============================================================================
# §4 图像层（hpp：唯一碰图像的地方）
# ============================================================================

def _find_blobs(img, thresholds, roi, pix_min, step):
    """find_blobs 的统一入口（kwargs 直接给，不做固件版本退让）。

    step（x_stride/y_stride）是**隔点采样**，能省时间但会让包围盒变粗、细结构漏掉：
      · 绿灯是大亮斑 → step=2 足够；
      · 灯条是细长结构 → 必须 step=1，否则 1~2px 宽的灯条会被整条跳过。
    """
    return img.find_blobs(thresholds, roi=roi, x_stride=step, y_stride=step,
                          pixels_threshold=pix_min, area_threshold=pix_min,
                          merge=False)


def find_led(img, roi, cfg, step):
    """窗口内找最像绿灯的 blob（C++ 口径）；找不到返回 None。

    step = 隔点采样步长：未锁定走全图时用 cfg.green_step（大亮斑，省时间），
    锁定后走小窗时用 1（包围盒不被量化，装甲板窗口的下沿才稳）。

    门限也是尺度自适应的：
      · 硬地板 GREEN_PIX_FLOOR：再小不可能是光斑（远距离真目标可能就 2~4px）；
      · 形状判据（填充率/宽高比）**只对长边 >= TINY_PX 的目标生效** ——
        3px 的光斑"宽高比"就是量化噪声，用它筛会直接把远距离目标筛掉。
        远距离的甄别交给调用方的时间一致性（Lock 的多帧确认 + 位移平滑）。
    """
    best = None
    pt = cfg.green_pix_floor // (step * step)
    if pt < 1:
        pt = 1                      # find_blobs 内部也按采样点比，得先放宽再自己换算
    for b in _find_blobs(img, [cfg.th_green], roi, pt, step):
        bb = Box(b.x(), b.y(), b.w(), b.h(), est_px(b.pixels(), step))
        if bb.px < cfg.green_pix_floor:
            continue
        if bb.w >= bb.h:
            long_side = bb.w
        else:
            long_side = bb.h
        if long_side >= cfg.tiny_px:
            if bb.px * 100 < cfg.green_fill_pct * bb.w * bb.h:   # 填充率
                continue
            if bb.w * 10 < cfg.green_aspect_lo_num * bb.h:       # 宽高比下限
                continue
            if bb.w * 10 > cfg.green_aspect_hi_num * bb.h:       # 宽高比上限
                continue
        if best is None or bb.px > best.px:
            best = bb
    return best


def find_blobs_raw(img, roi, cfg):
    '''窗口内**全部**亮块，不做任何形状判据（track 阶段的图像层入口）。

    为什么 track 可以省掉门限：判定"还是同一个目标"靠的是**窗口 + 位置/尺寸容差**
    （目标在它自己的窗里连续变化），不需要长度/长宽比/填充率/圆度 ——
    那些是 detect 阶段"从零判断这是不是一条灯条"才需要的。
    代价：候选可能比 find_bars 多（没筛），但每块只剩一次 find_blobs（native）
    + 几次整数比较；省掉的是每个候选的 bar_shape（二次方程 + 2 次 sqrt + asin）。
    '''
    out = []
    for b in _find_blobs(img, armor_thresholds(cfg), roi, cfg.bar_px_floor, 1):
        bb = Box(b.x(), b.y(), b.w(), b.h(), b.pixels())
        if bb.x <= roi[0] or bb.y <= roi[1] or \
           bb.x + bb.w >= roi[0] + roi[2] or bb.y + bb.h >= roi[1] + roi[3]:
            bb.border = 1        # 贴窗边标记仍要：日志里要看得到被切，也便于诊断
        out.append(bb)
    return out


def armor_thresholds(cfg):
    """装甲板二值化阈值（黑白口径）：只卡亮度 L >= bin_th，A/B 全放开，不看颜色。

    find_bars 和预览二值化都用这一份 —— 保证"预览里看到的二值图"就是
    "检测真正跑的那张"，不会出现看着对、实际用另一套阈值的情况。
    """
    return [(cfg.bin_th, 100, -128, 127, -128, 127)]


def gate_bar(cfg, s_lo, s_hi):
    """由"绿灯尺度"算出灯条门限 → (最短长边, 最长长边, 最少像素数)。

    这是"变阈值"的落点：固定像素门限在"逐渐接近"的工况下必然自相矛盾 ——
    远距离真灯条只有 2~6px，近距离一块干扰亮块有几百 px。所以门限写成
    **绿灯尺度的倍数**（绿灯和装甲板刚性连接，尺度天然同步），只在物理上
    无意义的地方保留硬地板：

      最短长边 = max(BAR_LEN_FLOOR, BAR_LEN_MIN_K * s)
      最长长边 = max(BAR_LEN_FLOOR, BAR_LEN_MAX_K * s)
      最少像素 = max(BAR_PX_FLOOR, 最短长边 // 2)   ← 由长度推出来，不另设系数：
                 1px 宽的实心条长 L 就有 L 个像素，所以 L/2 是"成条"的下限

    s_lo/s_hi 是尺度包络（min/max(s, s_pred)）：目标在远处长得快，拿单点尺度
    定门限容易把这一帧的目标判掉，所以下界取小的、上界取大的。
    s 还未知（没抓到绿灯）时传 0 → 门限全落地板 = 最宽松，靠时间一致性兜住。
    """
    ln_min = int(cfg.bar_len_min_k * s_lo)
    if ln_min < cfg.bar_len_floor:
        ln_min = cfg.bar_len_floor
    ln_max = int(cfg.bar_len_max_k * s_hi)
    if ln_max < cfg.bar_len_floor:
        ln_max = cfg.bar_len_floor
    px_min = ln_min // 2
    if px_min < cfg.bar_px_floor:
        px_min = cfg.bar_px_floor
    return (ln_min, ln_max, px_min)


def is_bar(b, gate, cfg):
    """细长亮条判据（hpp：具名谓词，find_bars 只负责取 blob）。

    gate = gate_bar() 出的 (最短长边, 最长长边, 最少像素)。
    形状判据（长度下限/长宽比/填充率）在**远档**（长边 < TINY_PX）整条跳过：
    几个像素的"长宽比/填充率"是量化噪声、不是特征，拿它筛只会把真目标筛掉。
    远档的甄别交给时间一致性（Lock 的多帧确认 + 位移平滑）。
    """
    if b.w < 1 or b.h < 1 or b.px < gate[2]:
        return False
    long_side = bar_len(b)               # 反解后的真实长边（斜灯条也对）
    if long_side < gate[0] or long_side > gate[1]:
        return False
    if b.w >= b.h:
        ax = b.w
    else:
        ax = b.h
    if ax < cfg.tiny_px and b.px > 1:
        # 远档（包围盒长边都不到 tiny_px）：几个像素的"长宽比/填充率"是量化噪声，
        # 只认"有这么一条亮条"，甄别交给时间一致性（Lock 的多帧确认 + 位移平滑）。
        return True
    L, t, tilt = b.shape()
    if L < cfg.bar_aspect_min * t:       # 长宽比用**真实**长短边 → 与倾斜无关
        return False
    if tilt < cfg.bar_flat_deg and b.px * 100 < cfg.bar_fill_pct * b.w * b.h:
        # 基本竖直/水平时包围盒口径仍可信 → **保留填充率门限**（散点/镂空挡在这里）；
        # 斜的时候 fill 会被包围盒放大而天然变低（30° 时 0.22），拿它筛等于筛掉真灯条。
        return False
    return True


def find_bars(img, roi, cfg, gate):
    """窗口内找"细长发光灯条"。黑白口径：只卡亮度 L>=bin_th，A/B 全放开，不看颜色。

    局限（实测过，不是理论担心）：连通域是按"亮像素是否相连"分的，所以
    **任何贴着灯条的亮东西都会把灯条的包围盒撑大**（比如灯条上方一块亮斑和灯条
    在 8 邻域上连成一片），这时包围盒长宽比立刻掉下去、灯条就"消失"了。
    对策只有两条，都在调参侧：① BIN_TH 调到"板体/干扰压暗、灯条还亮"的位置；
    ② 曝光别过曝（过曝会让亮块互相粘连）。判据本身不该为这个现象让步 ——
    放宽长宽比就等于让方形亮块也能当灯条，配对会开始乱选。

    成本提示：BIN_TH 越低 / 曝光越亮 → 通过阈值的白像素越多 → find_blobs
    （要跑连通域）和候选灯条数都涨，帧时间跟着涨。识别够用就别再往下压 BIN_TH。
    """
    bars = []
    for b in _find_blobs(img, armor_thresholds(cfg), roi, gate[2], 1):
        bb = Box(b.x(), b.y(), b.w(), b.h(), b.pixels())
        # 贴**搜索窗**边 = 这条灯条可能被窗口削掉了 → 长度/面积只是下界。
        # 这是个真信号（实测：旧"绿灯上方整块"把 L140 的灯条切到只剩 122，配对被连带带偏），
        # 所以配对时罚分而不是硬否 —— 被削的真灯条仍然要用。
        if bb.x <= roi[0] or bb.y <= roi[1] or \
           bb.x + bb.w >= roi[0] + roi[2] or bb.y + bb.h >= roi[1] + roi[3]:
            bb.border = 1
        if is_bar(bb, gate, cfg):
            bars.append(bb)
    return bars


# ============================================================================
# §5 锁（hpp：帧间唯一状态）
# ============================================================================

class ScaleModel(object):
    """目标尺度的**线性一步外推**（借用仓库里"目标均匀变大/尺度线性"的口径）。

    s = 绿灯的等效尺寸 max(w, h)。ds 是逐帧增量，用 EMA 平滑：
        ds_ema <- 0.5*ds_ema + 0.5*(s - s_prev)
        s_pred = s + clamp(SCALE_V_K * ds_ema, ±SCALE_V_MAX)
    用途：门限包络 band() —— 目标在远处长得快，拿单点尺度定的门限会把这一帧的
    目标判掉，所以下界取 min(s, s_pred)、上界取 max(s, s_pred)。

    为什么只外推一帧：尺度随时间不是线性的（s ∝ 1/(Z0 - v·t)），外推越远越离谱。
    一帧（33ms）内线性够用；要外推多帧就必须换模型（这是仓库里卡尔曼在干的事，
    这里没必要 —— 我们没有"距离/速度"观测量，只有像素尺度）。
    """
    def __init__(self):
        self.s = None
        self.ds = 0.0

    def update(self, box):
        """喂本帧的绿灯框（Box 或 None）。没有实测就保持上次的 s（不外推）。"""
        if box is None:
            return self.s
        s = box.w if box.w > box.h else box.h
        if self.s is not None:
            self.ds = 0.5 * self.ds + 0.5 * (s - self.s)
        self.s = s
        return self.s

    def band(self, cfg):
        """门限用的尺度包络 (s_lo, s_hi)。s 未知 → (0, 0) = 门限全落地板。"""
        if self.s is None:
            return (0, 0)
        v = cfg.scale_v_k * self.ds
        if v > cfg.scale_v_max:
            v = cfg.scale_v_max
        elif v < -cfg.scale_v_max:
            v = -cfg.scale_v_max
        sp = self.s + v
        if sp < 0:
            sp = 0
        if self.s < sp:
            return (int(self.s), int(sp))
        return (int(sp), int(self.s))

    def gate(self, cfg):
        """本帧的灯条门限（远→近自动变）。"""
        lo, hi = self.band(cfg)
        return gate_bar(cfg, lo, hi)


class Lock(object):
    """目标锁。每帧喂一个测量，回答两个问题：本帧结果能不能用、当前锁没锁上。

    每条规则都对应一个已经踩过的坑：
      ① 连续 confirm_n 帧命中才进 LOCK —— 单帧命中可能只是噪声/反光（误锁）。
      ② 锁定后搜索窗只用**本帧实测位置 + margin**，不做速度外推。
         外推项（"和上一帧的位移再走一步"）一旦被一个坏值喂进去，窗口会自己
         越跑越远、再也拉不回来（锁丢）。目标是慢慢变大变近的，30fps 下
         不需要外推；将来真要加，也必须把外推长度限幅。
      ③ 连续 miss_limit 帧没命中 → 释放锁、回全图搜索 —— 否则一个已经消失的
         目标会把窗口永久钉在画面某个角落，之后什么都搜不到（锁死）。
      ④ 没命中就把 box 清成 None，**绝不返回上一帧的坐标** —— 上层拿到旧值会
         以为还有目标，这是这类脚本最常见的错（假锁定）。
      ⑤ 死区（deadband）：新测量和上次报出去的框相差不超过 deadband 像素时，
         继续用旧框。±1px 的量化抖动会让**下游窗口边界来回蹭**（装甲板窗口的
         下沿就贴在绿灯上沿，蹭一下就把灯条截断）。注意是跟"上次报出去的框"比，
         所以目标真在慢慢移动时照样跟得上、不会冻住。
      ⑥ **远档的判据只能是时间**：尺度小（px < green_pix_min 或长边 < tiny_px）
         时，单帧形状和噪声不可分，此时
           · 确认帧数提到 far_confirm（多要一帧证据）
           · 加位移平滑门限：这一帧的位置和上一帧差太远（> max(far_jump_min,
             far_jump_k*尺度)）就**从头数**，不许把几个散落的噪声点累加成一"确认"
         近档（目标够大）形状可信，确认帧数用 confirm_n 就够。
    """

    def __init__(self, confirm_n, miss_limit, margin_k, min_margin, min_side,
                 deadband=0):
        self.confirm_n = confirm_n
        self.miss_limit = miss_limit
        self.margin_k = margin_k
        self.min_margin = min_margin
        self.min_side = min_side
        self.deadband = deadband
        self.locked = False
        self.hits = 0
        self.misses = 0
        self.box = None       # 上次报出去的包围盒；没命中就是 None

    def update(self, box, cfg):
        """喂本帧测量（Box 或 None），返回 (本帧有实测结果, 是否已锁定)。

        远档（目标只有几个像素）额外做两件事：确认帧数提到 far_confirm；
        位置跳太远就从头数（位移平滑）—— 远距离单帧形状和噪声不可分，
        只有"连续几帧在同一条平滑轨迹上"才是目标。
        """
        if box is None:
            self.box = None                 # ④ 不留旧值
            self.hits = 0
            self.misses += 1
            if self.misses >= self.miss_limit:
                self.locked = False         # ③ 释放锁，下一帧回全图
                self.misses = 0
            return False, self.locked

        s = box.w if box.w > box.h else box.h
        far = box.px < cfg.green_pix_min or s < cfg.tiny_px
        if far and self.box is not None:
            j = int(self.box_scale() * cfg.far_jump_k)      # 位移平滑门限
            if j < cfg.far_jump_min:
                j = cfg.far_jump_min
            dx = box.cx - self.box.cx
            dy = box.cy - self.box.cy
            if dx < 0:
                dx = -dx
            if dy < 0:
                dy = -dy
            if dx > j or dy > j:
                self.hits = 0               # ⑥ 跳太远：从头数，不许累加出"确认"
        self.hits += 1
        self.misses = 0
        if self.locked and self.box is not None and self.deadband > 0:
            if _absdiff(box.x, self.box.x) <= self.deadband and \
               _absdiff(box.y, self.box.y) <= self.deadband:
                box = self.box              # ⑤ 死区内：继续用上次报出去的框
        self.box = box
        if not self.locked and self.hits >= (cfg.far_confirm if far else self.confirm_n):
            self.locked = True              # ① 连续命中够了才锁
        return True, self.locked

    def box_scale(self):
        """上一帧报出去的绿灯尺度（没框时 0）。"""
        if self.box is None:
            return 0
        return self.box.w if self.box.w > self.box.h else self.box.h

    def search_roi(self, img_w, img_h, cfg):
        """本帧该在哪里找绿灯：没锁=全图，锁了=上一帧位置 + margin。

        注意 self.box 是 Box 对象（远档要读它的 px），不是元组 —— 改成对象时
        这里漏改过一处，`x, y, w, h = self.box` 直接 TypeError（已加自测钉住）。
        """
        if not self.locked or self.box is None:
            return clamp_roi(cfg.search_roi, img_w, img_h, cfg.roi_min_side)
        b = self.box
        m = int((b.w if b.w > b.h else b.h) * self.margin_k)
        if m < self.min_margin:
            m = self.min_margin
        return clamp_roi((b.x - m, b.y - m, b.w + 2 * m, b.h + 2 * m),
                         img_w, img_h, self.min_side)


class ArmorTracker(object):
    """装甲板 detect-track 的 track 那一半。状态只有"上一对灯条 + 中心 + 丢了帧数"。

    为什么**画面静止**时结果还会乱跳（三种成因，逐条对治）：
      ① 逐帧重选：pick_armor 每帧从零挑"分数最高"的一对，两对分数接近时逐帧换人
         → 规则 A：先做**关联**。上一对还在（位置/尺寸在容差内）就继续用它，
           绝不因为这一帧另一对分数更高而换人。
      ② 阈值边缘抖动：BIN_TH 附近的像素逐帧翻，灯条包围盒 ±1px、长度在门限上
         翻来翻去，某帧整条灯条就"消失"了
         → 规则 B：**保持**（hold）最多 armor_hold 帧，返回旧中心但打 held 标记。
           这不是"把旧值冒充成新测量"：日志里标了保持几帧，上层分得清
           （没标记的才是本帧实测）。连丢够 armor_hold 帧就释放。
      ③ 窗口边界蹭着灯条：装甲板窗口下沿贴在绿灯上沿，绿灯包围盒一跳，窗口就把
         灯条截断一段、灯条长度跟着变
         → 规则 C：绿灯包围盒带死区（LED_DEADBAND）+ 锁定后绿灯搜索步长 1。
         这两条在 Lock/主循环里，不在本类。

    效率：跟踪成功后绿灯够大就**每条灯条各开一个小窗**（ARMOR_PER_BAR_R 定门限），
    否则开一个合窗；两者 margin 都按单条灯条长度给（ARMOR_BAR_MARGIN_*），
    比"绿灯上方整块"小得多；丢了 armor_hold 帧就退回整块重新 detect。
    """

    def __init__(self, cfg):
        self.a = None      # 上一对灯条（Box）
        self.b = None
        self.c = None      # 上一帧的中心
        self.age = 0       # 已经保持了几帧（0 = 本帧有实测）
        self.r_last = 0.0  # 上次采到那一对时的绿灯半径（算尺度增长用）

    def reset(self):
        self.a = None
        self.b = None
        self.c = None
        self.age = 0
        self.r_last = 0.0

    def _reach_box(self, led, cfg):
        """锚的"够得着"范围：以灯心为中心、ARMOR_NEAR_K × 半径 为半边的外接方框。

        单条窗**不再**被"绿灯上方那块"裁剪 —— 那正是"某些角度包不住装甲板"的原因
        （那块窗口的尺寸被板跨度绑住）。安全性改由这条径向范围提供：
        窗口可以跟着灯条走，但永远不许跑到离灯心 ARMOR_NEAR_K 个半径之外。
        """
        half = cfg.armor_near_k * led_radius(led)
        return (led.cx - half, led.cy - half, half * 2.0, half * 2.0)

    def _grow_pad(self, led, bar):
        """按"这一帧目标长大了多少"再补一圈 margin。

        工况是**逐渐接近**：30fps 下目标每帧能长 10~20%（近末端更多）。窗口是按
        "上次采到的那条灯条"开的，不补偿增长的话这一帧它就顶出窗外 → 被切 → 丢帧 →
        连续丢够 hold 帧就退回整块重捕（实测：s 从 40 长到 60 那一帧就掉了）。

        增长比直接用绿灯半径的变化量算（灯尺与灯条是同一个尺度）。
        """
        if self.r_last <= 0:
            return 0
        g = led_radius(led) / self.r_last - 1.0
        if g <= 0.0:
            return 0
        return int(bar_len(bar) * g * 0.5)

    def _bar_window(self, bar, led, img_w, img_h, cfg):
        """**一条灯条自己**的小窗：margin 按它自己的长度给（与整块板的跨度无关）。

        合窗必须同时盖住两条灯条 + 中间空白 + 转动后的外接矩形变化，所以尺寸被
        "板跨度"绑住；单条窗只需盖住它自己 —— 板转多少度都不影响这个窗口够不够。
        """
        m = int(cfg.armor_bar_margin_k * bar_len(bar)) + self._grow_pad(led, bar)
        if m < cfg.armor_bar_margin_min:
            m = cfg.armor_bar_margin_min
        elif m > cfg.armor_bar_margin_max + self._grow_pad(led, bar):
            m = cfg.armor_bar_margin_max + self._grow_pad(led, bar)
        w = intersect_roi((bar.x - m, bar.y - m, bar.w + 2 * m, bar.h + 2 * m),
                          self._reach_box(led, cfg))
        if w is None:
            return None
        # 地板用"margin 下限"（8px）而不是 roi_min_side（24px）：后者是给**绿灯/整块窗**
        # 定的（太小扫了没意义），拿它卡单条窗会**静默禁用**这个功能 ——
        # 实测 r=20（刚过 ARMOR_PER_BAR_R）时窗只有 54x22，被 24 的地板判成"没意义"，
        # 于是每帧都悄悄退回合窗。C++ 侧本来就是 8，这里对齐。
        return clamp_roi(w, img_w, img_h, int(cfg.armor_bar_margin_min))

    def _union_window(self, full_roi, led, img_w, img_h, cfg):
        """合窗（远距离 / 首次 / 单条窗没建起来）：上一对灯条周围一个窗，与整块取交。

        margin 按**灯条长度**给，不再按"整个对的尺寸×0.8" —— 后者在近距离会开出
        十几万像素的窗口（按对尺寸 0.8 倍算过：128880px vs 35280px）。
        """
        if self.a is None or self.b is None:
            return full_roi
        x0 = self.a.x if self.a.x < self.b.x else self.b.x
        y0 = self.a.y if self.a.y < self.b.y else self.b.y
        x1a, x1b = self.a.x + self.a.w, self.b.x + self.b.w
        y1a, y1b = self.a.y + self.a.h, self.b.y + self.b.h
        x1 = x1a if x1a > x1b else x1b
        y1 = y1a if y1a > y1b else y1b
        side = bar_len(self.a)
        side_b = bar_len(self.b)
        if side_b > side:
            side = side_b
        pad = self._grow_pad(led, self.a if bar_len(self.a) > bar_len(self.b) else self.b)
        m = int(cfg.armor_bar_margin_k * side) + pad
        if m < cfg.armor_bar_margin_min:
            m = cfg.armor_bar_margin_min
        elif m > cfg.armor_bar_margin_max + pad:
            m = cfg.armor_bar_margin_max + pad
        small = clamp_roi((x0 - m, y0 - m, (x1 - x0) + 2 * m, (y1 - y0) + 2 * m),
                          img_w, img_h, cfg.roi_min_side)
        if small is None:
            return full_roi
        return intersect_roi(small, full_roi) or full_roi

    def plan(self, led, img_w, img_h, cfg):
        '''本帧该怎么扫 —— 跟踪器只给**计划**，图像层由调用方执行。
        （与仓库 C++ 里 TargetTracker::begin_frame → Plan → end_frame 同一套路：
          跟踪器不认识像素，调用方按计划去扫，扫完再 end_frame 吃结果。）

        返回 None，或 (mode, wins, gated)：
          mode='detect'：wins = [整块窗]，gated=True
                         → 调用方用 find_bars(img, win, cfg, gate)（全门限）
          mode='track' ：wins = 每条灯条一个窗（或一个合窗），gated=False
                         → 调用方用 find_blobs_raw(img, win, cfg)（只捞块）
        返回 None = 这一帧没有可用窗口（绿灯锚出画）→ 调用方整帧跳过装甲板。

        注意：单条窗只需"锚够得着"的范围，**不需要**"绿灯上方整块"那个窗 ——
        所以算不出整块窗时照样能跟踪（旧写法把整块窗当前置条件，会为它掐掉整帧）。
        '''
        full = armor_roi_from_led(led, img_w, img_h, cfg)
        if self.a is not None and self.b is not None:
            if led_radius(led) >= cfg.armor_per_bar_r:
                wins = []
                for bar in (self.a, self.b):
                    w = self._bar_window(bar, led, img_w, img_h, cfg)
                    if w is None:
                        break
                    wins.append(w)
                if len(wins) == 2:
                    return ('track', wins, False)   # 单条窗：窗本身就是区域门限
            if full is not None:
                return ('track', [self._union_window(full, led, img_w, img_h, cfg)], False)
        if full is None:
            return None
        return ('detect', [full], True)

    def search_windows(self, full_roi, led, img_w, img_h, cfg):
        """本帧该扫**哪些**窗（返回 1~2 个）。

        绿灯够大（半径 >= ARMOR_PER_BAR_R）且已在跟踪 → 两条灯条各一个小窗：
        窗口只需盖住自己，与板跨度无关，斜到多少度都不会"包不住"。
        远距离（灯条只有几像素，单条窗太小不可靠）→ 还是合窗。
        任一条的单条窗建不起来 → 整帧退回合窗（宁可多扫，不可漏）。
        """
        if self.a is None or self.b is None:
            return [full_roi]
        if led_radius(led) < cfg.armor_per_bar_r:
            return [self._union_window(full_roi, led, img_w, img_h, cfg)]
        wins = []
        for bar in (self.a, self.b):
            w = self._bar_window(bar, led, img_w, img_h, cfg)
            if w is not None:
                wins.append(w)
        if len(wins) < 2:
            return [self._union_window(full_roi, led, img_w, img_h, cfg)]
        return wins

    def _assoc(self, bars, ref, cfg):
        """在候选里找 ref 的同一条灯条：位置和尺寸都在容差内，取中心最近的那个。"""
        tx = int(ref.w * cfg.armor_assoc_k)
        ty = int(ref.h * cfg.armor_assoc_k)
        if tx < cfg.armor_assoc_min:
            tx = cfg.armor_assoc_min
        if ty < cfg.armor_assoc_min:
            ty = cfg.armor_assoc_min
        found = None
        best = None
        for b in bars:
            dx = b.cx - ref.cx if b.cx > ref.cx else ref.cx - b.cx
            dy = b.cy - ref.cy if b.cy > ref.cy else ref.cy - b.cy
            dw = b.w - ref.w if b.w > ref.w else ref.w - b.w
            dh = b.h - ref.h if b.h > ref.h else ref.h - b.h
            if dx > tx or dy > ty or dw > tx or dh > ty:
                continue
            d = dx * dx + dy * dy
            if best is None or d < best:
                best = d
                found = b
        return found

    def update(self, win_bars, mode, cfg, led=None, s=0):
        '''喂本帧捞到的块（**按窗分组**，与 plan() 的 wins 一一对应）
        → ((bar_a, bar_b, (cx,cy)) 或 None, held_age)。

        mode='track'：**只认亲**（位置 + 尺寸容差），一条形状判据都不做。
          判定"还是同一个目标"靠的就是**在它自己的窗里连续变化**：
          位置容差挡住"跳到别处"，尺寸容差挡住"换了个大小的东西"。
          所以这里不算 L/t/tilt、不查长度/长宽比/填充率/圆度；反解只对
          **认到的那两条**做（端点要用 L 和主轴，见 paired_ends）。
          `same_body`（板心到灯心的径向距离）保留，但只当**释放触发**：
          一旦认到的东西跑到锚够不着的地方，就记 miss 而不是静默返回一个坏中心，
          连续超 ARMOR_HOLD 帧就退回 detect 重捕。
        mode='detect'：全门限。is_bar 已由调用方（find_bars）做过，
          这里做 O(n²) 配对 + 刚性先验，然后锁上。

        led/s 只给 detect 的配对路径用（pick_armor 的刚性几何先验）。

        held_age = 0：本帧实测（可以用作"新测量"）。
        held_age > 0：这是保持的旧中心，已经连续丢了这么多帧（上层要区别对待）。
        '''
        if mode == 'track':
            if self.a is None or self.b is None or self.c is None:
                return None, 0
            if len(win_bars) == 2:
                na = self._assoc(win_bars[0], self.a, cfg)   # 各窗各认各的
                nb = self._assoc(win_bars[1], self.b, cfg)
            else:
                pool = win_bars[0] if win_bars else []
                na = self._assoc(pool, self.a, cfg)
                nb = self._assoc(pool, self.b, cfg)
            if na is not None and nb is not None and na is not nb:
                c = armor_center(na, nb, cfg)                # 端点用反解（只算这两条）
                if c is not None and same_body(cfg, led, c[0], c[1]):
                    self.a, self.b, self.c = na, nb, c
                    self.age = 0
                    self.r_last = led_radius(led)
                    return (na, nb, c), 0
            self.age += 1                                    # 认不到 → 记 miss
            if self.age <= cfg.armor_hold:
                return (self.a, self.b, self.c), self.age
            self.reset()                                     # 超时 → 退回 detect
            return None, 0

        bars = win_bars[0] if win_bars else []               # detect：全门限那条路
        r = pick_armor(bars, cfg, led, s)
        if r is None:
            return None, 0
        self.a, self.b, self.c = r
        self.age = 0
        self.r_last = led_radius(led)
        return r, 0

# ============================================================================
# §6 板端：曝光注入 + 主循环（hpp 之外）
# ============================================================================

def setup_exposure(sensor, cfg):
    """★ 手动曝光注入。只在 sensor.run() **之后**调用（之前固件不收）。"""
    if cfg.exposure_us is None:
        print('exposure: EXPOSURE_US=None，保持固件默认（AE 已关）')
    else:
        sensor.exposure(int(cfg.exposure_us))
        print('exposure: 设 %s us，回读 %s' % (cfg.exposure_us, sensor.exposure()))
    if cfg.gain_db is not None:
        sensor.again(cfg.gain_db)
        print('gain: again(%s) 已设' % cfg.gain_db)


def _fmt_box(b):
    return '(%d,%d %dx%d px=%d)' % (b.x, b.y, b.w, b.h, b.px)


def _fmt_bar(b):
    """灯条：包围盒 + **反解出来的真实长短边与倾角** + bbox 口径的 aspect/fill。

    为什么要打反解值：斜灯条上 bbox 的 aspect/fill 会同时"变难看"（30° 时 1.5 / 22%），
    拿它调 BAR_ASPECT_MIN / BAR_FILL_PCT 会被误导 —— 真正参与判据的是 L、t。
    """
    L, t, tilt = b.shape()
    return '%s%s L=%.1f t=%.1f tilt=%.0f° bbox(asp=%.1f fill=%d%%)' % (
        _fmt_box(b), 'B' if b.border else '', L, t, tilt,
        (b.w if b.w > b.h else b.h) * 1.0 / (b.h if b.w > b.h else b.w),
        b.px * 100 // (b.w * b.h))


def main():
    cfg = make_config()
    led_lock = Lock(cfg.confirm_n, cfg.miss_limit, cfg.margin_k,
                    cfg.roi_min_margin, cfg.roi_min_side, cfg.led_deadband)
    tracker = ArmorTracker(cfg)
    scale = ScaleModel()
    frame = 0
    t_win = time.ticks_ms()
    acc = [0, 0, 0, 0, 0]   # 阶段耗时累计 + 帧数（time_stages 用）

    sensor = Sensor(width=cfg.frame_w, height=cfg.frame_h, fps=cfg.sensor_fps)
    sensor.reset()
    sensor.set_framesize(width=cfg.frame_w, height=cfg.frame_h)
    sensor.set_pixformat(Sensor.RGB565)
    if cfg.preview:
        Display.init(Display.VIRT, width=cfg.frame_w, height=cfg.frame_h, to_ide=True)
    MediaManager.init()

    # 硬约束：关 AE 必须在 run() 之前（run 之后固件不支持再开关自动曝光）；
    # 而 exposure()/again() 又必须在 run() 之后（见 setup_exposure）。
    sensor.auto_exposure(False)
    print('AE off')
    sensor.run()
    setup_exposure(sensor, cfg)
    print('start: %dx%d@%d EXPOSURE_US=%s BIN_TH=%d TH_GREEN=%s CONFIRM=%d MISS=%d' %
          (cfg.frame_w, cfg.frame_h, cfg.sensor_fps, cfg.exposure_us,
           cfg.bin_th, cfg.th_green, cfg.confirm_n, cfg.miss_limit))

    try:
        while True:
            t_a = time.ticks_us()
            img = sensor.snapshot()
            t_b = time.ticks_us()
            frame += 1
            w_img = img.width()
            h_img = img.height()

            # ---- 绿灯（锚）：远档放宽形状判据、要求更多帧确认 + 位移平滑 ----
            # 锁上以后用 step=1：窗口小、不差这点钱，但包围盒不再被隔点采样量化，
            # 装甲板窗口的下沿就不会一帧一跳（那是静止画面下"灯条忽长忽短"的主因）。
            search = led_lock.search_roi(w_img, h_img, cfg)
            led = find_led(img, search, cfg,
                           1 if led_lock.locked else cfg.green_step) \
                if search is not None else None
            led_fresh, locked = led_lock.update(led, cfg)
            # 尺度是状态：用报出去的绿灯框（死区稳过的）喂线性一步外推，
            # 灯条门限随它变（远档 → 门限落地板；近档 → 门限按倍数放大）
            scale.update(led_lock.box)
            gate = scale.gate(cfg)
            t_c = time.ticks_us()

            # ---- 装甲板：detect-track ----
            # detect：窗口来自**本帧实测**的绿灯（绿灯没锁就一直不检测）；
            # track：上一对灯条周围开小窗（省时间），丢了 ARMOR_HOLD 帧才退回整块。
            wins = None
            aroi = None
            bars = None
            armor = None
            held = 0
            mode = '-'
            if led_fresh and locked:
                # 跟踪器只给计划（mode/wins/gated），图像层由这里执行 ——
                # detect 走全门限（find_bars），track 只捞块（find_blobs_raw，不做形状判据）
                p = tracker.plan(led, w_img, h_img, cfg)
                if p is not None:
                    mode, wins, gated = p
                    win_bars = []
                    for w in wins:
                        if gated:
                            win_bars.append(find_bars(img, w, cfg, scale.gate(cfg)))
                        else:
                            win_bars.append(find_blobs_raw(img, w, cfg))
                    aroi = _union_roi(wins)                          # 日志/画框用并集
                    bars = win_bars[0]
                    armor, held = tracker.update(win_bars, mode, cfg, led, scale.s or 0)
            t_d = time.ticks_us()

            # ---- 预览 ----
            if cfg.preview:
                # 模式 1：整帧压成二值图。**必须在画标注之前**，否则 binary() 会把
                # 标注一起压掉；检测已经做完了，改这张图不影响本帧结果。
                if cfg.preview_bin == 1:
                    img.binary(armor_thresholds(cfg))
                # 模式 2：原始帧不动，把窗口的二值图放大贴到左上角（小图几十 KB）
                elif cfg.preview_bin == 2 and aroi is not None:
                    sub = img.copy(aroi).binary(armor_thresholds(cfg))
                    z = cfg.preview_bin_zoom
                    if sub.height() * z > h_img // 3:
                        z = 1          # 放大后糊满整幅就没意义了
                    img.draw_image(sub, 0, 0, x_scale=z, y_scale=z)
                if led is not None:
                    img.draw_rectangle(led.x, led.y, led.w, led.h,
                                       color=(255, 255, 0), thickness=1)
                    img.draw_cross(led.cx, led.cy, color=(255, 255, 0), size=5)
                if wins:
                    for w in wins:      # 单条窗模式会画出两个框
                        img.draw_rectangle(w[0], w[1], w[2], w[3],
                                           color=(0, 0, 255), thickness=1)
                if bars is not None:
                    for b in bars:
                        img.draw_rectangle(b.x, b.y, b.w, b.h,
                                           color=(255, 0, 255), thickness=2)
                if armor is not None:
                    ba, bbar, c = armor
                    (a0, a1, b0, b1) = paired_ends(ba, bbar, cfg)
                    img.draw_line(a0[0], a0[1], b1[0], b1[1],
                                  color=(0, 255, 255), thickness=1)   # A上 — B下
                    img.draw_line(a1[0], a1[1], b0[0], b0[1],
                                  color=(0, 255, 255), thickness=1)   # A下 — B上
                    img.draw_cross(c[0], c[1], color=(255, 0, 0), size=8)
                Display.show_image(img)
            t_e = time.ticks_us()

            # 阶段累计（只为了回答"33.3ms 花在哪一段"，本身开销可忽略）
            if cfg.time_stages:
                acc[0] += time.ticks_diff(t_b, t_a)     # snapshot（等帧 + 拷贝）
                acc[1] += time.ticks_diff(t_c, t_b)     # 找绿灯
                acc[2] += time.ticks_diff(t_d, t_c)     # 找装甲板
                acc[3] += time.ticks_diff(t_e, t_d)     # 二值化 + 画 + 显示
                acc[4] += 1

            # ---- 统计行 ----
            if frame % cfg.print_every == 0:
                now = time.ticks_ms()
                dt = time.ticks_diff(now, t_win)
                fps = cfg.print_every * 1000.0 / dt
                t_win = now

                msg = '[%5d] %s led=%s 尺=%s 半径=%s' % (
                    frame, 'LOCK' if locked else 'HUNT',
                    _fmt_box(led) if led is not None else '-',
                    scale.s if scale.s is not None else '?',
                    '%.0f' % led_radius(led) if led is not None else '?')
                if led is None:
                    msg += '(连丢%d)' % led_lock.misses

                if aroi is None:
                    msg += ' | armor: 无窗口(绿灯未锁定或窗口出画)'
                elif armor is None:
                    msg += ' | armor: MISS %s/%s 窗口(%d,%d %dx%d) 块%d条' % (
                        mode, _win_name(wins), aroi[0], aroi[1], aroi[2], aroi[3], len(bars))
                    for b in bars:
                        msg += ' ' + _fmt_bar(b)
                elif held:
                    # 保持：中心是旧的，标出保持了第几帧 —— 它不是本帧实测
                    msg += ' | armor: HOLD%d 中心=(%d,%d)' % (held, armor[2][0], armor[2][1])
                else:
                    ba, bb2, c = armor
                    msg += ' | armor: OK A[%s] B[%s] 中心=(%d,%d)' % (
                        _fmt_bar(ba), _fmt_bar(bb2), c[0], c[1])
                    # 刚性几何比值：这几行的值就是"板上固定几何 ÷ 灯尺"，与距离无关。
                    # 拿它去填 §1 的 PLATE_*_PCT，先验才真正生效（不用统计，看几行就够）。
                    if scale.s:
                        msg += ' %s/%s 比值(条长/尺=%.2f,%.2f 间距/尺=%.2f 离心/半径=%.2f)' % (
                            mode, _win_name(wins),
                            bar_len(ba) * 1.0 / scale.s, bar_len(bb2) * 1.0 / scale.s,
                            pair_span(ba, bb2) * 1.0 / scale.s,
                            dist(c[0], c[1], led.cx, led.cy) / (scale.s * 0.5))

                msg += ' | %.1f fps' % fps
                if cfg.time_stages and acc[4]:
                    n = acc[4] * 1000.0
                    msg += ' | ms: snap=%.1f led=%.1f armor=%.1f show=%.1f' % (
                        acc[0] / n, acc[1] / n, acc[2] / n, acc[3] / n)
                    acc[0] = acc[1] = acc[2] = acc[3] = acc[4] = 0
                print(msg)

    except KeyboardInterrupt:
        print('stopped')
    finally:
        # 释放顺序：先停 sensor，再 display，最后 media（顺序反了会卡住）
        sensor.stop()
        if cfg.preview:
            Display.deinit()
        MediaManager.deinit()


# ============================================================================
# §7 几何自测（主机侧可跑，不需要板子：python3 tools/armor_detect_k230.py）
# ============================================================================

def _selftest_geometry():
    """把"配对 + 对角线交点"这套算法在合成灯条上验一遍。

    这里验的就是最容易被写错、又最难在板上看出来的部分：
    取错角点、连线接反、平行时不判、交点在延长线上还当真、配对选错人。
    """
    cfg = make_config()
    fails = []

    def check(name, cond, extra=''):
        print('  %-46s %s %s' % (name, 'PASS' if cond else 'FAIL', extra))
        if not cond:
            fails.append(name)

    # ① 正对的两条等高竖灯条：中心必须是四条角点的几何中心
    a = Box(100, 50, 6, 40, 240)
    b = Box(140, 50, 6, 40, 240)
    r = pick_armor([a, b], cfg)
    ok = r is not None
    check('等高竖直灯条 → 配对成功', ok)
    if ok:
        c = r[2]
        # A(103,50)-(103,89)  B(143,50)-(143,89) → 中心 (123, 69.5)
        check('  板心 = 两灯条中点 (123,69~70)', abs(c[0] - 123) <= 1 and abs(c[1] - 69.5) <= 1,
              '得到 %s' % (c,))

    # ② 长度不等：交点仍必须落在两段之内（不能跑到延长线上）
    a2 = Box(100, 50, 6, 40, 240)
    b2 = Box(140, 60, 6, 40, 240)
    r2 = pick_armor([a2, b2], cfg)
    ok2 = r2 is not None
    check('长度不等(错位10) → 仍配对成功', ok2)
    if ok2:
        c2 = r2[2]
        inside = 100 <= c2[0] <= 146 and 50 <= c2[1] <= 100
        check('  交点在两灯条之间', inside, '得到 %s' % (c2,))

    # ③ 一竖一横：必须拒绝
    check('一竖一横 → 拒绝', pick_armor([Box(100, 50, 6, 40, 240),
                                        Box(10, 10, 40, 6, 240)], cfg) is None)

    # ④ 三条灯条：必须挑出真正成对的那两条（另外一条是干扰）
    ia = Box(300, 100, 5, 40, 200)
    ib = Box(340, 100, 5, 40, 200)
    ic = Box(500, 300, 5, 40, 200)
    r4 = pick_armor([ic, ia, ib], cfg)
    ok4 = r4 is not None
    check('三条灯条 → 挑中成对的那两条', ok4 and r4[0].x == 300 and r4[1].x == 340,
          '' if not ok4 else '选中 x=%d,%d' % (r4[0].x, r4[1].x))

    # ⑤ 灯条判据本身。用**固定参考配置**，不拿 §1 的现场调参当基准 ——
    #    否则板上把 BAR_ASPECT_MIN 调松之后，这条自测会变成假报警。
    tc = Config()
    tc.bar_aspect_min, tc.bar_fill_pct = 3, 50
    tc.bar_len_min_k, tc.bar_len_max_k = 0.2, 8.0
    tc.bar_len_floor, tc.bar_px_floor, tc.tiny_px = 2, 2, 8
    gn = gate_bar(tc, 40, 40)          # 近档尺度 40：门限 = max(2, 8)=8 / 320 / 4
    check('判据(近档 s=40): 6x60 实心(长:短=10) → 是灯条', is_bar(Box(0, 0, 6, 60, 360), gn, tc))
    check('判据(近档): 20x40 (长:短=2 < 3) → 不是灯条', not is_bar(Box(0, 0, 20, 40, 800), gn, tc))
    check('判据(近档): 长边 6 < 门限 8 → 不是灯条', not is_bar(Box(0, 0, 2, 6, 12), gn, tc))
    check('判据(近档): 6x60 但像素不足(散点) → 不是灯条', not is_bar(Box(0, 0, 6, 60, 72), gn, tc))

    # ⑤b 变阈值：同一个判据在远/近两档下结论不同 —— 这就是"不能有固定像素门限"
    gf = gate_bar(tc, 4, 4)            # 远档尺度 4：门限 = 2 / 32 / 2
    check('变阈值: 远档(s=4) 2x6 的小灯条 → 算灯条(近档会被判掉)',
          is_bar(Box(0, 0, 2, 6, 12), gf, tc) and not is_bar(Box(0, 0, 2, 6, 12), gn, tc),
          '远档门限 %s / 近档门限 %s' % (gf, gn))
    check('变阈值: 远档 6x60 的大亮条 → 不算灯条(相对尺子太大了)',
          not is_bar(Box(0, 0, 6, 60, 360), gf, tc))
    check('变阈值: 门限永不低于硬地板',
          gate_bar(tc, 0, 0) == (tc.bar_len_floor, tc.bar_len_floor, tc.bar_px_floor),
          '得到 %s' % (gate_bar(tc, 0, 0),))

    # ⑤c 尺度线性模型：目标在远处长得快，门限要按包络给（下界取小、上界取大）
    sm = ScaleModel()
    sm.update(Box(0, 0, 8, 8, 64))
    for k in range(4):
        sm.update(Box(0, 0, 8 + 3 * (k + 1), 8 + 3 * (k + 1), 64))
    lo, hi = sm.band(tc)
    check('尺度模型: 连续变大 → 包络下界=上一帧、上界=外推值',
          lo == 20 and hi > 20, '得到 s=%s ds=%.1f 包络=(%d,%d)' % (sm.s, sm.ds, lo, hi))
    sm.ds = 99.0
    lo2, hi2 = sm.band(tc)
    check('尺度模型: 外推被 SCALE_V_MAX 限幅(防坏值跑飞)',
          hi2 - sm.s <= tc.scale_v_max, '得到 外推 %d' % (hi2 - sm.s))
    check('尺度模型: 没抓到绿灯(s 未知) → 门限全落地板(最宽松)',
          ScaleModel().gate(tc) == gate_bar(tc, 0, 0))

    # ⑤d 远档的判据只能是时间：多要一帧确认 + 位移平滑
    lk = Lock(tc.confirm_n, tc.miss_limit, tc.margin_k, tc.roi_min_margin,
              tc.roi_min_side, tc.led_deadband)
    tiny = Box(300, 200, 3, 3, 9)      # 远档：3px 光斑
    check('远档: 3px 目标 2 帧还不够(要 far_confirm=3 帧)',
          lk.update(tiny, tc) == (True, False) and lk.update(tiny, tc) == (True, False))
    check('远档: 第 3 帧才锁定', lk.update(tiny, tc) == (True, True))
    lk2 = Lock(tc.confirm_n, tc.miss_limit, tc.margin_k, tc.roi_min_margin,
               tc.roi_min_side, tc.led_deadband)
    lk2.update(tiny, tc)
    far_away = Box(400, 300, 3, 3, 9)  # 远档但位置跳了 100px → 不许累加确认
    check('远档: 位置跳太远 → 确认计数从头数(位移平滑)',
          lk2.update(far_away, tc) == (True, False) and lk2.hits == 1)
    big = Box(300, 200, 40, 40, 1600)  # 近档：形状可信，2 帧就锁
    lk3 = Lock(tc.confirm_n, tc.miss_limit, tc.margin_k, tc.roi_min_margin,
               tc.roi_min_side, tc.led_deadband)
    check('近档: 40px 目标 2 帧就锁(confirm_n)',
          lk3.update(big, tc) == (True, False) and lk3.update(big, tc) == (True, True))

    # ⑤e search_roi 必须能吃 Box 对象（远档要读 px，所以 box 不再是元组）
    lk4 = Lock(tc.confirm_n, tc.miss_limit, tc.margin_k, tc.roi_min_margin,
               tc.roi_min_side, tc.led_deadband)
    check('search_roi: 没锁 → 全图', lk4.search_roi(640, 480, tc) == tc.search_roi)
    lk4.update(big, tc)
    lk4.update(big, tc)
    roi4 = lk4.search_roi(640, 480, tc)
    check('search_roi: 锁定 → 上一帧框 + margin',
          roi4 is not None and roi4[0] < big.x and roi4[1] < big.y
          and roi4[2] > big.w and roi4[3] > big.h, '得到 %s' % (roi4,))
    lk5 = Lock(tc.confirm_n, tc.miss_limit, 0.5, 0, 1, 0)
    lk5.update(Box(600, 460, 40, 40, 1600), tc)
    lk5.update(Box(600, 460, 40, 40, 1600), tc)
    check('search_roi: 贴着画面右下角 → 被裁进画面',
          (lambda r: r is not None and r[0] + r[2] <= 640 and r[1] + r[3] <= 480)(
              lk5.search_roi(640, 480, tc)), '得到 %s' % (lk5.search_roi(640, 480, tc),))

    # ⑥ 距离过远：不是同一块板
    check('间距 400 > BAR_GAP_MAX_PCT×长边 → 拒绝',
          pick_armor([Box(100, 0, 4, 40, 160), Box(500, 0, 4, 40, 160)], cfg) is None)

    # ⑦ 近平行 / 共线：交点必须判为不存在（否则会算出个乱跳的中心）
    check('共线两段 → 无交点',
          line_intersection((0, 0), (10, 0), (20, 0), (30, 0), cfg.diag_min_sin_pct) is None)
    check('近平行两段 → 无交点',
          line_intersection((0, 0), (100, 0), (0, 1), (100, 1),
                            cfg.diag_min_sin_pct) is None)

    # ⑧ 交点在延长线上：必须判为不合格
    check('交点在段外 → 无交点',
          line_intersection((0, 0), (10, 0), (20, 0), (20, 10),
                            cfg.diag_min_sin_pct) is None)

    # ⑨ 横灯条：长轴端点取左右边中点
    hb = Box(100, 50, 40, 6, 240)
    p0, p1 = bar_ends(hb)
    check('横灯条端点 = 左右边中点', p0 == (100, 53) and p1 == (139, 53),
          '得到 %s %s' % (p0, p1))

    # ⑩ 窗口：下沿必须正好贴着绿灯上沿，其余部分全在绿灯上方
    led = Box(300, 200, 40, 40, 1600)
    roi = armor_roi_from_led(led, 640, 480, cfg)
    check('装甲板窗口下沿贴着绿灯上沿(且不重叠)',
          roi is not None and roi[1] + roi[3] == led.y, '得到 %s' % (roi,))
    check('  窗口水平以灯心为中心',
          roi is not None and roi[0] + roi[2] // 2 == led.cx, '得到 %s' % (roi,))
    # ⑪ 间隔系数 >0 时窗口整体上移（= 下沿离绿灯越来越远，别用默认值干这事）
    c_up = make_config()
    c_up.armor_roi_gap_k = 1.0
    roi_up = armor_roi_from_led(led, 640, 480, c_up)
    check('gap_k=1.0 时下沿离绿灯 1 倍灯高',
          roi_up is not None and roi_up[1] + roi_up[3] == led.y - led.h,
          '得到 %s' % (roi_up,))

    # ⑫ 预览的二值化必须和检测用同一份阈值（"看到的就是检测用的"）
    check('预览二值化阈值 == 检测阈值',
          armor_thresholds(cfg) == [(cfg.bin_th, 100, -128, 127, -128, 127)],
          '得到 %s' % (armor_thresholds(cfg),))

    # ⑭ 隔点采样必须换算成真实像素数：否则远距离目标被 step² 倍缩水判掉
    check('est_px: step=2 数到 2 个采样点 = 真实 8 px', est_px(2, 2) == 8)
    check('est_px: step=1 不变', est_px(9, 1) == 9)
    check('est_px: 缩水案例 —— 3x3 真目标在 step=2 下数到 2 点，不换算会低于地板 3',
          est_px(2, 2) >= cfg.green_pix_floor and 2 < cfg.green_pix_floor)

    # ⑮ 刚性几何先验（前提已确认：灯与板固定在同一块板上）；与距离无关
    A1, B1 = Box(300, 100, 6, 40, 240), Box(340, 100, 6, 40, 240)
    led1 = Box(300, 200, 40, 40, 1600)
    chk = Config()
    chk.plate_span_lo_pct, chk.plate_span_hi_pct = 50, 500     # 间距/尺 ∈ [0.5, 5]
    chk.plate_up_lo_pct, chk.plate_up_hi_pct = 20, 400         # 高/尺 ∈ [0.2, 4]
    chk.plate_offx_pct = 200                                   # |横偏|/尺 <= 2
    span = pair_span(A1, B1)
    check('pair_span: 竖条取横向中心距', span == 40, '得到 %d' % span)
    check('pair_span: 横条取纵向中心距',
          pair_span(Box(0, 0, 40, 6, 240), Box(0, 30, 40, 6, 240)) == 30)
    def sc10(b, k):
        return Box(b.x * k, b.y * k, b.w * k, b.h * k, b.px * k * k)
    for k in (1, 10):           # 同一套**比值**、整幅几何按 k 一起缩放 → 结论必须一致
        r = pick_armor([sc10(A1, k), sc10(B1, k)], chk, sc10(led1, k), 40 * k)
        check('先验: 同一套比值、整幅缩放 %d 倍 → 通过(与距离无关)' % k, r is not None)
    led_far = Box(300, 200, 40, 40, 1600)                      # 灯尺不变，板心挪太远
    A2, B2 = Box(300, 0, 6, 40, 240), Box(340, 0, 6, 40, 240)  # 高/尺 = (200-20)/40 = 4.5
    check('先验: 板心离灯太远(高/尺=4.5 > 4) → 拒绝',
          pick_armor([A2, B2], chk, led_far, 40) is None)
    A3, B3 = Box(300, 100, 6, 40, 240), Box(316, 100, 6, 40, 240)
    check('先验: 两灯条贴太近(间距/尺=0.4 < 0.5) → 拒绝',
          pick_armor([A3, B3], chk, led1, 40) is None)
    A4, B4 = Box(440, 100, 6, 40, 240), Box(480, 100, 6, 40, 240)
    check('先验: 整块偏到灯心一侧太远(横偏/尺=4.5 > 2) → 拒绝',
          pick_armor([A4, B4], chk, led1, 40) is None)
    check('先验: 灯尺没建立(s=0) → 跳过"间距/尺"，但**径向判据照用**',
          plate_prior(chk, led1, 0, A2, B2, (320, 100))
          and not plate_prior(chk, led1, 0, A2, B2, (320, 40)))

    # ⑤f 斜灯条（"两条灯条平行但不竖直"）：包围盒口径会同时废掉长宽比与填充率
    sc_ = Config()
    sc_.bar_aspect_min, sc_.bar_fill_pct = 2, 50
    sc_.bar_len_min_k, sc_.bar_len_max_k = 0.2, 8.0
    sc_.bar_len_floor, sc_.bar_px_floor, sc_.tiny_px = 2, 2, 8
    sc_.bar_flat_deg, sc_.bar_parallel_deg, sc_.bar_thick_min_ratio_num = 12, 15, 3
    g60 = gate_bar(sc_, 60, 60)
    # 一条真实的 5x40 灯条转 30°：包围盒 = 24x37、面积 200（下面这几组都用它）
    tb = Box(300, 100, 24, 37, 200)
    L_, t_, til_ = tb.shape()
    check('斜灯条: 反解回真实长短边 40x5 / 30°',
          abs(L_ - 40) < 1.0 and abs(t_ - 5) < 0.5 and abs(til_ - 30) < 1.5,
          '得到 (%.1f,%.1f,%.1f°)' % (L_, t_, til_))
    check('斜灯条: 旧(bbox)口径确实会判死 —— 长宽比 1.5<2 且 填充 22%<50%',
          37 < 2 * 24 and 200 * 100 < 50 * 24 * 37)
    check('斜灯条: 新口径判成活灯条', is_bar(tb, g60, sc_))
    fb = Box(300, 100, 5, 40, 200)
    check('竖直灯条零回归: 仍成活灯条且 L=40', is_bar(fb, g60, sc_) and abs(fb.shape()[0] - 40) < 0.01)
    # 端点必须落在**灯条自己的轴**上（不是包围盒边的中点）
    ta_, tb_ = Box(300, 100, 24, 37, 200), Box(364, 100, 24, 37, 200)
    e_ = paired_ends(ta_, tb_, sc_)
    d_ = dist(e_[0][0], e_[0][1], ta_.cx, ta_.cy)
    check('斜灯条: 端点落在长轴上(离中心 = L/2 ≈ 20)', abs(d_ - 20.0) < 1.5,
          '得到 %.1f' % d_)
    c_ = armor_center(ta_, tb_, sc_)
    check('斜灯条: 板心 ≈ 两条灯条中心的中点', c_ is not None
          and abs(c_[0] - 344) <= 2 and abs(c_[1] - 118) <= 2, '得到 %s' % (c_,))
    up_ = Box(364, 100, 5, 40, 200)          # 一条竖直的
    check('平行判据: 倾角差 30° > 15° → 不成对', pair_score(ta_, up_, sc_) is None)
    fat_ = Box(364, 100, 42, 47, 1000)       # 同倾角但厚度反解成 ~25（vs 5）
    check('厚度判据: 两条灯条粗细差 5 倍 → 不成对',
          abs(fat_.shape()[2] - til_) < 15 and pair_score(ta_, fat_, sc_) is None,
          '得到 tilt=%.0f° t=%.1f' % (fat_.shape()[2], fat_.shape()[1]))

    # ⑬ detect-track：detect 才判门限，track 只认亲（按窗 + 位置/尺寸容差）
    A0 = Box(300, 100, 6, 40, 240)
    B0 = Box(340, 100, 6, 40, 240)
    FULL = (200, 100, 240, 160)
    tr = ArmorTracker(cfg)
    led_small = Box(300, 200, 8, 8, 64)      # 半径 4 < ARMOR_PER_BAR_R → 合窗
    led_big = Box(300, 200, 60, 60, 3600)    # 半径 30 >= ARMOR_PER_BAR_R → 单条窗
    led_edge = Box(300, 200, 40, 40, 1600)   # 半径 20 == 门限（刚够）

    # plan()：三种路径 + "算不出整块窗也能跟踪"
    p0 = tr.plan(led_big, 640, 480, cfg)
    check('plan(无跟踪状态): detect + 整块窗(=armor_roi_from_led 现算) + 要门限',
          p0 is not None and p0[0] == 'detect' and p0[2] is True
          and p0[1] == [armor_roi_from_led(led_big, 640, 480, cfg)],
          '得到 %s' % (p0,))
    r, held = tr.update([ [A0, B0] ], 'detect', cfg, led_big, 60)
    check('detect: 首次成对 → 出结果(held=0)', r is not None and held == 0)
    p1 = tr.plan(led_big, 640, 480, cfg)
    check('plan(跟踪中+灯大): track + 两条灯条各一个窗 + **不要门限**',
          p1 is not None and p1[0] == 'track' and len(p1[1]) == 2 and p1[2] is False,
          '得到 %s' % (p1,))
    p2 = tr.plan(led_small, 640, 480, cfg)
    check('plan(跟踪中+灯小): track + 一个合窗 + 不要门限',
          p2 is not None and p2[0] == 'track' and len(p2[1]) == 1 and p2[2] is False,
          '得到 %s' % (p2,))
    led_top = Box(300, 2, 60, 60, 3600)      # 绿灯贴画面顶：算不出"上方整块"窗
    p3 = tr.plan(led_top, 640, 480, cfg)
    check('plan: 算不出整块窗时，单条窗路径照样能跟踪（旧写法会掐掉整帧）',
          p3 is not None and p3[0] == 'track' and len(p3[1]) == 2, '得到 %s' % (p3,))

    # **这一条是本轮的核心**：track 路径不再做"重选"和形状判据。
    #   可断言的两件事：① pick_armor（O(n²) 配对 + 全部门限）在 track 期间 0 次；
    #                  ② bar_shape（反解）在 track 期间只算认到的那 2 条/帧。
    #   至于 is_bar：它由**调用方**在 find_bars 里调（plan 给了 gated=False 就改用
    #   find_blobs_raw，见上面那条断言），所以这里数不到它 —— 那是设计如此。
    n_pick = [0]
    n_shape = [0]
    orig_pick = globals()['pick_armor']
    orig_shape = globals()['bar_shape']
    def _cnt_pick(bars, cfg_, led=None, s=0):
        n_pick[0] += 1
        return orig_pick(bars, cfg_, led, s)
    def _cnt_shape(w, h, a):
        n_shape[0] += 1
        return orig_shape(w, h, a)
    globals()['pick_armor'] = _cnt_pick
    globals()['bar_shape'] = _cnt_shape
    try:
        tr2 = ArmorTracker(cfg)
        tr2.update([ [A0, B0] ], 'detect', cfg, led_big, 60)      # detect：会走 pick_armor
        det_pick = n_pick[0]
        n_pick[0] = 0
        n_shape[0] = 0
        for k in range(6):                                        # 静止 + ±1px 抖动
            d = 1 if k % 2 else -1
            tr2.update([ [Box(300 + d, 100, 6, 40, 240)],             # 单条窗：各窗各认
                         [Box(340, 100 + d, 6, 40, 240)] ],
                       'track', cfg, led_big, 60)
        track_pick = n_pick[0]
        track_shape = n_shape[0]
    finally:
        globals()['pick_armor'] = orig_pick
        globals()['bar_shape'] = orig_shape
    check('track: 不再"重选"（pick_armor 0 次；detect 那次 %d 次）' % det_pick,
          det_pick > 0 and track_pick == 0, 'track 期间 pick_armor=%d' % track_pick)
    check('track: 反解只对认到的两条算（6 帧 × 2 条 = 12 次）',
          track_shape == 12, 'bar_shape 调用 %d 次' % track_shape)

    ok_track = True
    cs = []
    for k in range(6):                      # 静止画面 + ±1px 抖动 6 帧
        d = 1 if k % 2 else -1
        rr, hh = tr.update([ [Box(300 + d, 100, 6, 40, 240)],
                             [Box(340, 100 + d, 6, 40, 240)] ],
                           'track', cfg, led_big, 60)
        if rr is None or hh != 0:
            ok_track = False
        else:
            cs.append(rr[2][0])
    check('track: ±1px 抖动 6 帧 → 不丢、不保持、中心只差 1px',
          ok_track and cs and max(cs) - min(cs) <= 1, '得到 %s' % (cs,))
    sr = tr.plan(led_small, 640, 480, cfg)[1]
    check('track(远): 灯条太小时仍是**一个**合窗，且比整块小',
          len(sr) == 1 and sr[0][2] * sr[0][3] * 3 < FULL[2] * FULL[3], '得到 %s' % (sr,))
    sr2 = tr.plan(led_big, 640, 480, cfg)[1]
    inside = True
    for i, bar in enumerate((tr.a, tr.b)):
        w = sr2[i]
        if not (w[0] <= bar.x and w[1] <= bar.y
                and w[0] + w[2] >= bar.x + bar.w and w[1] + w[3] >= bar.y + bar.h):
            inside = False
    check('track(近): 每个窗都完整包住自己那条灯条', inside, '得到 %s' % (sr2,))
    area2 = sr2[0][2] * sr2[0][3] + sr2[1][2] * sr2[1][3]
    check('track(近): 单条窗合计像素 < 整块', area2 < FULL[2] * FULL[3],
          '得到 %d < %d' % (area2, FULL[2] * FULL[3]))

    tr3 = ArmorTracker(cfg)                 # 另一对更"完美"时不许换人
    Aa, Bb = Box(300, 100, 6, 40, 240), Box(340, 100, 6, 36, 220)
    tr3.update([ [Aa, Bb] ], 'detect', cfg, led_big, 60)
    D1, D2 = Box(200, 250, 6, 40, 240), Box(240, 250, 6, 40, 240)
    naive = pick_armor([Aa, Bb, D1, D2], cfg)
    r2, h2 = tr3.update([ [Aa, Bb, D1, D2] ], 'track', cfg, led_big, 60)
    check('track: 窗里出现另一对 → 仍认原来那两条(不换人)',
          naive is not None and naive[0].x == D1.x and r2 is not None and r2[0].x == Aa.x,
          '无状态会选 x=%d，跟踪仍用 x=%d' % (naive[0].x, r2[0].x))

    if cfg.armor_hold >= 1:                 # 保持 → 到期释放
        tr4 = ArmorTracker(cfg)
        tr4.update([ [A0, B0] ], 'detect', cfg, led_big, 60)
        h_seq = []
        for k in range(cfg.armor_hold + 1):
            rr, hh = tr4.update([ [] ], 'track', cfg, led_big, 60)   # 窗里什么都没有
            h_seq.append((rr is not None, hh))
        check('hold: 丢 %d 帧内保持(带 age 标记)' % cfg.armor_hold,
              all(ok for ok, _ in h_seq[:cfg.armor_hold])
              and h_seq[cfg.armor_hold - 1][1] == cfg.armor_hold,
              '得到 %s' % (h_seq,))
        check('hold: 超过 %d 帧释放(不无限保持)' % cfg.armor_hold,
              h_seq[cfg.armor_hold][0] is False, '得到 %s' % (h_seq,))

    # same_body 在 track 里是"释放触发"：认到的东西跑到锚够不着 → 记 miss 而不是给坏中心
    tr5 = ArmorTracker(cfg)
    tr5.update([ [A0, B0] ], 'detect', cfg, led_big, 60)
    far_pair = [Box(900, 100, 6, 40, 240), Box(940, 100, 6, 40, 240)]  # 离灯心很远
    rr5, hh5 = tr5.update([ [far_pair[0]], [far_pair[1]] ], 'track', cfg, led_big, 60,
                          )
    check('track: 认到的东西跑出锚的够得着范围 → 记 miss(held>0)，不给坏中心',
          rr5 is None or hh5 > 0, '得到 held=%d' % hh5)

    # 径向判据：板心离灯心太远 → 不是同一个整体
    led_n = Box(300, 200, 60, 60, 3600)      # 半径 30 → NEAR_K(8) × 30 = 240px
    check('同一个整体: 板心在灯心旁 40px → 通过', same_body(cfg, led_n, 340, 200))
    check('同一个整体: 板心在灯心旁 400px → 拒绝', not same_body(cfg, led_n, 700, 200))
    check('同一个整体: 半径没建立 → 放行', same_body(cfg, Box(0, 0, 0, 0, 0), 700, 200))
    check('同一个整体: 斜着偏（对角线）也按距离算，不受方向影响',
          same_body(cfg, led_n, 300 + 200, 200 + 200) is False      # 283px > 240
          and same_body(cfg, led_n, 300 + 100, 200 + 100) is True)  # 141px < 240

    tr4 = ArmorTracker(cfg)                 # 关联容差：整体挪太远 → 不认领
    tr4.update([[A0, B0]], 'detect', cfg, led_big, 60)
    r4, h4 = tr4.update([[Box(300, 200, 6, 40, 240), Box(340, 200, 6, 40, 240)]],
                        'track', cfg, led_big, 60)
    check('assoc: 整体挪 100px → 不张冠李戴(先保持而不是认领)', h4 > 0,
          '得到 held=%d' % h4)

    print('---')
    if fails:
        print('几何自测: FAIL %d 项: %s' % (len(fails), fails))
        return False
    print('几何自测: 全部 PASS')
    return True


if __name__ == '__main__':
    if RUN_SELFTEST or not _HAVE_MEDIA:
        ok = _selftest_geometry()
        raise SystemExit(0 if ok else 1)
    main()
