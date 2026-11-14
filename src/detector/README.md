# 颜色目标检测与跟踪

本目录在 RGB565 帧上按 Lab 颜色识别绿灯与装甲板，算法与参数移植自
[`tools/sc132gs_test.py`](../../tools/sc132gs_test.py) 的 `ColorTargetDetector`
（SC132GS、AWB 开、480×640 实拍调参）。不依赖 K230/MPP，只借用 `camera::FrameView`。

| 文件 | 类 / 职责 |
|---|---|
| `color.hpp/.cpp` | `Color`、`Lab`、`LabRange`；`ColorTable`：RGB565→Lab（OpenMV 公式）和颜色掩码查找表 |
| `target.hpp/.cpp` | 值类型（`Blob`、`Target`、`DetectResult`、`ScanCounters`）；`Association` 身份代价；`Light`、`Armor` 规则 |
| `track.hpp/.cpp` | `Track`：连续确认、实测速度预测、ROI 规划、丢失与间隔重置 |
| `detect.hpp/.cpp` | `Detect::Workspace` 有界提取；`Detect` 单帧测量与参数表；组件工厂接口 |
| `detector.cpp` | 唯一执行 component `Detector`：取帧、调用 Detect/Track、发布坐标 |

## 接口

```cpp
#include "src/detector/detect.hpp"

using namespace dart::detection;
Detect::Config config;                       // 默认值即 sc132gs_test.py 的 DETECTOR_CONFIG
Detect::set_parameter(config, "color.green.l_min", 56);
Detect detect{config};
Track track{config.light, config.armor, config.track, config.roi};
auto prepared = detect.Prepare(camera.frame_info()); // 格式、最大尺寸、pixel_aspect
auto result = detect.update(lease.view(), track);    // 帧租约有效期间调用
track.Reset();                                       // 重启图像源后清除历史
```

- `Detect::Supports(format)` 只接受 `PixelFormat::kRgb565`。`Prepare()` 校验配置，一次性分配
  查找表（64 KiB 掩码 + 192 KiB Lab）与提取缓冲；之后逐帧不分配内存。
- `pixel_aspect` 来自相机 `FrameInfo`，用于还原灯的圆度和宽高比（脚本中的 `shape_x_scale`）。
- 无目标是成功结果：`kind == kNone`，`coordinate()` 为 `(-1, -1)`。工作区不足、非法帧、
  格式不符和乱序帧返回 `DetectionError`；测量失败会清除跟踪历史。
- `DetectorFactory::create()` 的 `Spec::parameter` 使用字段路径，另有 `enabled`、`acquire_ms`、
  `max_acquire_errors`。`enabled=1` 且相机不是 RGB565 时初始化失败（`kUnsupportedFormat`）；
  `enabled=0` 仍取帧供录像等消费者使用。生命周期、租约转移和坐标发布规则与此前一致：
  先发布坐标，再让同步消费者借用帧；返回错误的路径清除坐标。

参数键与 Python `DETECTOR_CONFIG` 相同；颜色阈值写作
`color.<green|red|blue>.<l_min|l_max|a_min|a_max|b_min|b_max>`（对应 CanMV 的
`(L0, L1, a0, a1, b0, b1)`），参与识别的颜色用掩码 `light.colors` / `armor.colors`
（1 绿、2 红、4 蓝；默认灯为绿，装甲板为三色）。运行时通过 `dart_app --detect KEY=VALUE` 传入。

## 测量

- **像素分类**：`ColorTable` 预先把 65536 个 RGB565 值换算为 OpenMV 约定的整数 Lab
  （RGB565 位复制展开为 RGB888，sRGB/D65，floor），并生成每种颜色的区间掩码。逐像素只查表。
- **提取**：每种请求的颜色单独扫描窗口，按 `step` 间隔取种子（对应 `find_blobs` 的
  `x_stride/y_stride`），种子命中后用扫描线填充得到像素精确的 8 邻域连通域。
  面积和包围框面积都需达到下限（灯 `light.min_area`，灯条 `armor.area_floor`）。
  超过 `workspace.max_candidates` 个区域或 `workspace.max_spans` 个待处理行段时返回
  `kWorkspaceExhausted`，不静默截断。
- **区域特征**：像素质心、包围框、平均 Lab（区域自身像素）、按 pixel_aspect 修正的
  圆度（轴比 × 与圆盘填充率 π/4 的接近程度）与宽高比，以及由每行端点凸包求得的
  最小面积矩形（长、厚、主轴、中心，对应 `min_corners()`）。
- **颜色质量**：平均色度 `sqrt(a²+b²)` 低于 `color.min_chroma` 的区域直接拒绝（白色高光
  不算颜色证据）；`color_quality = min(1, chroma / max(24, 2·min_chroma)) × (0.5 + 0.5 × 区间居中度)`，
  是调参分数，不是概率。

## 选择与跟踪

- **灯**：面积、最大面积比例、圆度（所有尺寸，含远档）、非远档的填充率与修正宽高比。
  获取阶段按 `面积 × 圆度^circularity_weight × 颜色质量` 取最大；跟踪阶段只接受与预测
  位置关联的同色候选（`Association`：同色、宽高比 `track.min_size_ratio`、颜色质量比
  `track.min_color_ratio`、Lab 距离 `track.max_lab_distance`、半径
  `max(track.association_min_px, track.association_k × 上次最大边)`）。
- **装甲板**：只在当前灯上方窗口（`armor.window_*_k` × 灯直径）用 step 1 扫描
  `armor.colors`。灯条为实心矩形（长度、长宽比、定向填充率）；两条灯条须同色、平行、
  长度/厚度相近、投影重叠、间距在灯尺度范围内，板心为端点四边形对角线交点，
  位置受灯锚定（`plate_*_pct`）。两条灯条 Lab 距离不超过 `armor.max_lab_distance`，
  条间 3×3 采样点中与灯条同色的比例不超过 `armor.max_gap_color_fraction`。
  已跟踪的装甲板只接受与预测框、两条灯条位置和身份代价都匹配的新测量。
- **确认**：灯连续 `light.confirm_frames`（远档 `far_confirm_frames`）帧关联才发布；
  远档另有跳变门限 `far_jump_*`。装甲板连续 `armor.confirm_frames` 帧确认；
  已跟踪装甲板缺失 `armor.track_hold` 帧后释放，之后逐帧搜索直到新板确认。
- **ROI**：锁定后按预测灯（及装甲板）规划窗口；ROI 内丢失或贴边时同帧全图重扫，
  仍只关联原目标。连续 `roi.reset_after_misses` 帧丢失、接收间隔超过 `roi.max_gap`
  或尺寸变化时清除全部历史。跟踪中每 `track.validation_interval` 帧或颜色质量降到
  上次完整复查的 `track.quality_drop_ratio` 以下时搜索装甲板。
- 预测只用于搜索和关联，不作为测量发布；`light.deadband_px` 只作用于发布值，
  速度估计使用原始测量。

结果附带 `reason`（`DetectReason`，与脚本的 `reason` 字段对应）、各级拒绝计数
`scan.counters`、确认计数、搜索窗口与重扫标记，便于和 Python 脚本逐项对照。

## 与 Python 脚本的已知差异

- 连通性为 8 邻域；CanMV `find_blobs` 的填充规则未逐位核对。斜细灯条更不易被拆碎。
- 平均 Lab 只统计区域自身像素；脚本的 `get_statistics(roi=bbox)` 会计入包围框内同色的其它像素。
- 二阶矩按单位像素面积计入 1/12，单像素和极小区域的圆度有定义；大区域不受影响。
- 每种颜色一个 Lab 区间（脚本允许 1..4 个，默认均为 1 个）。
- 不含脚本的调试候选、预览绘制和颜色取样。

以上默认值来自单一场景的实拍调参，阈值与几何参数仍需在 RT-Smart 的实际 RGB565
输出上复核。主机测试只验证算法行为、组件生命周期与交叉编译链接；
识别质量、耗时和 RGB565 ISP 通路需在板端验证。
