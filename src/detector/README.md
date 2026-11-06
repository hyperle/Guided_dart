# 检测与跟踪

`detector.cpp` 是本目录唯一的 `.cpp`，其中 `Detector` 继承
`executor::Component`，负责取帧、组织计算和发布目标像素坐标。
算法类在头文件中实现，配置、工作区和历史状态归所属类持有。

| 文件 | 类 / 职责 |
|---|---|
| `target.hpp` | `Light`、`Armor` 的目标筛选和几何计算，以及坐标、候选和结果值类型 |
| `detect.hpp` | `Detect` 的单帧扫描、候选提取、质量检查和有界 `Workspace`；对外 component 工厂契约 |
| `track.hpp` | `Track` 的目标关联、连续确认、速度预测、ROI 规划和丢失状态 |
| `detector.cpp` | 唯一执行 component `Detector`，参数读取、相机生命周期和 Blackboard 输出 |

`DetectorFactory::create()` 接受 component 的 `Spec`、相机接口、目标输出端口、
诊断状态及可选的同步帧消费者，返回 `std::unique_ptr<executor::Component>`。
将 `executor::Output<Point>` 绑定到 `BlackBoard::target` 后，Detector 只写该槽位，
无需读取 Blackboard。`BlackBoard::update()` 只刷新 IMU，不再搬运视觉坐标。

生命周期沿用 `initialze()`、`run(stop_token)` / `step()`、`shutdown()`：

- 初始化校验配置、一次性分配计算工作区、重置历史并启动相机。
- `capture_inputs()` 取帧；`update()` 调用 Detect 和 Track；`publish_outputs()`
  先发布坐标，再让录像、回传和诊断同步消费当前帧。
- 有效 Armor 优先于 Light；无当前有效目标、无帧、容量丢帧和识别错误均发布
  `(-1, -1)`。预测和短时保留的历史只用于搜索与关联，不发布为识别结果。
- 相机错误保留独立诊断，连续取帧错误达到 `max_acquire_errors` 时退出。
  返回错误而跳过后续生命周期阶段的路径，也会清除目标坐标。
- 同步消费者只在回调期间借用帧。录像接受时可以移动租约；其余租约由 Detector
  释放。退出前调用方先停止消费者并排空已转移的租约，再关闭 Detector 和相机。
  销毁 component 时，相机、目标通道和诊断状态必须仍然存活。

`Spec::parameter` 使用配置字段路径，如 `threshold`、`workspace.min_contrast`、
`light.confirm_frames`、`armor.confirm_frames`、`track.validation_interval` 和 `roi.max_gap`。
另有 `enabled`（0/1）、`acquire_ms` 和 `max_acquire_errors`；后两项为 1..10000 的整数。
未知参数、非有限值和非整数的计数参数会被拒绝。`enabled=0` 时仍取帧供录像等消费者使用，
目标槽位保持无效坐标。

计算类仅借用 `camera::FrameView`，不操作相机或保留像素指针，可独立调用：

```cpp
#include "src/detector/detect.hpp"

using namespace dart::detection;
Detect::Config config;
Detect detect{config};
Track track{config.light, config.armor, config.track, config.roi};
auto prepared = detect.Prepare(max_width, max_height);
// prepared 成功后，在帧租约有效期间调用：
auto result = detect.update(frame, track);
// 重新启动图像源后清除历史：
track.Reset();
```

无目标是成功结果，`kind == TargetKind::kNone`、`target.valid == false`，
`target.center` 和 `coordinate()` 均为 `(-1, -1)`。
工作区不足、非法输入和乱序帧返回 `DetectionError`。初始化后算法热路径不动态分配；
一个 Detect / Track 组合由同一条按帧顺序执行的流水线使用。

当前测量与跟踪规则：

- 直接在 ROI 的 Y8 上用可配置阈值提取 8 邻域连通域，按游程计算面积、质心、
  二阶矩、圆度和主轴；常规 step=1 扫描中两个检测类读同一组候选。首次全图获取时灯用 search_step 采样，装甲只在灯锚定的小窗 step=1 精扫。
- 连通域保留前景均值，读取包围框外 3 px 环带（`workspace.background_margin`）的
  背景均值和标准差。背景读取完整帧，不受 ROI 截断影响。若至少 75% 且不少于
  8 个环带像素比候选暗，排除其余同亮或更亮像素，减少相邻灯条污染；否则保留全部
  环带，避免把大片亮背景误估成暗背景。默认要求对比度 `C = 前景均值 - 背景均值`
  至少 12（`workspace.min_contrast`），`SNR = C / (背景标准差 + 4)` 至少 3
  （`workspace.min_snr`）。4 为固定灰度噪声底，避免均匀背景除零或过度放大小灰度差。
- 候选通过灰度差与 SNR 后，在其附近重用游程工作区检查 `T±8`
  （`workspace.stability_delta`）。低阈值不低于背景均值加标准差的下一整数，且不高于 T；
  高阈值饱和到 255，重复的阈值跳过。局部窗口四周扩展 `max(4, 最大边/2)` px，
  维持当前扫描的采样格点。按原阈值像素重叠匹配连通域，以面积变化、中心漂移和
  高阈值下的碎裂程度计算 0..1 稳定性；低阈值连通域延伸出局部窗口视为粘连。
  默认下限 0.5（`workspace.min_stability`），容许中心不变时面积缩至 1/4，
  不要求三个阈值的轮廓完全相同。该检查不是三次全图扫描，不额外分配整图缓存。
  `min_contrast=0`、`min_snr=0` 分别关闭对应门限；`stability_delta=0` 或
  `min_stability=0` 关闭稳定性检查。要恢复仅阈值提取，需要同时关闭三项。
- 灯检测保留面积、填充率、宽高比和圆度检查，排序改为面积乘圆度权重再乘灰度质量。
  灰度质量为 `clamp(SNR / max(6, 2*min_snr), 0, 1) * 稳定性`，贴边时再乘 0.25；
  它是调参分数，不是识别正确的概率。远档继续跳过不可靠的形状检查并使用跨帧确认，
  新候选的位置、宽高或对比度不满足关联门限时，大小目标都会重新计数。
- 装甲板沿用 v1 的本帧灯锚、灯尺度门限、灯条形状、平行/长度/厚度/间距门限和
  刚性距离先验。板心取灯条端点所构成四边形的两条对角线交点。
  配对沿主轴投影判断重叠，端点统一方向，避免旋转后依赖水平包围盒或正负倾角翻转。
- 装甲配对额外要求两灯条的较小/较大背景对比度至少 0.45
  （`armor.min_brightness_ratio`）。沿灯条方向和条间方向布置 3×3 个内部采样点，
  要求较暗灯条均值减条间均值至少 8（`armor.min_gap_contrast`）。两项也参与配对排序与
  质量评分，设为 0 可放宽相应门限；gap 门限为 0 时仍要求条间不比灯条亮。
  `Detect` 始终传入原始灰度帧；直接调用 `Armor::Detect` 不传 frame
  时仅检查几何，供已有几何消费者兼容使用，不会假造灰度证据。
- 跟踪时先过滤上一对灯条预测包围框附近的候选，再进行配对，仍保留形状检查。
  关联失败返回本帧装甲板无效，不直接换到最高分的其他灯条对。

`Detect` 与 `Track` 区分获取和跟踪阶段：

- Detect 全图按 `light.search_step` 获取，完整检查所有候选的灰度稳定性。
  灯连续确认默认 2 帧（远档 3 帧），新装甲板对默认确认 2 帧（`armor.confirm_frames`）。
- Track 以灯和装甲板各自的真实测量、`received` 时间差估计速度，第一次速度观测直接
  使用测量值，后续用 `track.velocity_alpha=0.5` 平滑。预测最多外推 50 ms
  （`track.prediction_max_ms`），每轴位移不超过关联半径的 4 倍，并限制在帧内。
  预测值只规划 ROI 和关联，不作为识别输出。发布坐标的 deadband 不参与速度估计。
- 灯优先匹配预测位置附近、尺寸和对比度相近的候选，按关联代价选择，避免亮度/面积
  更大的干扰物抢占。默认关联半径 `max(6 px, 1.5*上次最大边)`，由
  `track.association_min_px` / `track.association_k` 控制；宽高允许 0.5..2 倍变化
  （`track.min_size_ratio`），较小/较大对比度比至少 0.5（`track.min_contrast_ratio`）。
  装甲板还受自身的 `armor.association_*` 配对门限约束。
- 稳定跟踪逐帧保留灰度差、SNR 和几何检查；每 8 个跟踪帧完整复查一次
  （`track.validation_interval`，设为 1 可逐帧检查）。当前对比度或 SNR 降到最近
  完整复查值的 0.65 以下（`track.quality_drop_ratio`）时立即复查。
  完整复查仅处理选中的灯和两条灯条，最多 3 个候选，不对整个 ROI 的干扰物重做
  多阈值提取。新装甲板确认、已知装甲板短时丢失时也会触发完整复查。
- 已知装甲板缺失时本帧只输出仍然有效的灯心，默认连续缺失 2 帧
  （`armor.track_hold`）才解除旧灯条对的关联。随后搜索并连续确认新灯条对。
  没有已知装甲板时，常规帧只搜索灯附近；完整复查时扩展到灯上方的装甲板搜索窗。
- ROI 灯丢失或目标贴边时，同帧全图精扫，仍只关联原目标；不会让其他目标继承锁定。
  连续 3 帧失配后解除锁定，再进入 Detect 并重新确认新目标。短时丢失会扩大 ROI，
  大小变化、接收间隔超过 `roi.max_gap` 或 `Track::Reset()` 会清除全部预测与确认历史。
  `roi.full_scan_interval` 保留显式周期全图检查，默认 0 关闭。

窗口靠图像边缘时整体平移，保持尺寸。只用单调接收时间 `received` 估计速度和判断
间隔，原始 `source_pts` 和其有效标记原样传递。周期检查之间可能短暂保留新产生的
不稳定形状；提高检查频率会增加计算量。同位置、同形状、同灰度的物体仍无法靠这些
特征区分身份，关联门限与预测参数须实拍标定。

统一输出始终为本帧装甲板中心优先，否则为本帧灯心；两者都无效时不输出旧坐标。
同时提供灯、装甲板、窗口、灰度质量、全图/重扫标记、候选数和连续丢失帧数。
目标附带 `contrast`、`snr`、`stability`，装甲板另有 `brightness_ratio` 和 `gap_contrast`。
`stability_checked` 明确表示本帧是否实际进行多阈值检查；未检查时 stability 为 0，
quality 只含本帧 SNR 和配对结构分，不沿用历史稳定性作为本帧证据。
扫描另含 `phase`、`full_validation`、`stability_checks`；原生应用的 `op=detect_track`
状态行汇总各阶段及复查次数。保持 240 FPS 的完整链路能力仍需在目标和干扰场景测量。
扫描附带按灰度差、SNR、稳定性顺序过滤的拒绝计数；首次采样获取包含装甲精扫计数，
同帧重扫时报告最终扫描的计数。局部稳定性工作区不足同样明确返回错误。

现有相机只暴露 Y8，不包含色度，因此“绿色灯”按亮度与形状识别，无法区分绿色与
其他颜色的同形亮物。装甲板沿用 v1 的灯锚先验：未识别到本帧灯时不独立猜测装甲板。
阈值、曝光分离度、窗口尺寸和几何默认参数均为 **UNVALIDATED**，需要真实板端样本标定。
远档默认连续确认 3 帧，位移门限 `max(6 px, 1.5*上一候选最大边)`；
装甲锚定百分数默认为 0、1200、600，具体值以 Config 为准。
全图获取时 search_step=2 隔点扫描，灯上方装甲搜索窗 step=1 精扫，锁定后 step=1 ROI
扫描及贴边同帧全图复扫；未移植 v1 的 MMZ/VB 帧池、Kalman 状态树、保持旧坐标和录像逻辑。
标量实现保留了确定性参考口径，未宣称获得 v1 RVV 的板端速度。

工作区最大宽高为 65535，默认最多 16384 个游程和 256 个有效连通域；超过上限明确
返回 `kWorkspaceExhausted`，不静默截断。较小帧可以使用原有工作区，超过准备尺寸则拒收。
默认工作区约 1.5 MiB；每帧只清理实际使用的组件槽，不申请整图二值副本。
两个单独检测类消费 `Detect::Workspace::Scan()` 产生的候选，输入须遵守其几何和数值约定。

算法来源：[Guided_dart / k230_v1](https://github.com/hyperle/Guided_dart/tree/k230_v1)，
参考版本 `a7390262203f2565876aef1d2a8b628b2722d79f`，
主要参考 `src/detection/measure/roi_measure.cpp`、
`src/detection/armor/armor_{rules,geometry,windows}.cpp` 和
`include/detection/math/shape_metrics（上游历史路径）`。

验证应覆盖主机行为、component 生命周期、租约转移和 K230 交叉编译与链接；
实际识别质量、耗时和 DMA/流水线行为须单独在板端验证。
