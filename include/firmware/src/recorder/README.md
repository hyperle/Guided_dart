# 录像 / 图像证据

`src/recorder/recorder.cpp` 定义唯一执行终端 `Recorder : executor::Component`。
它接收识别完成的 `FrameLease`，由独立的 `executor::Tunner` 驱动；参数解析、
录像启停与编码由该组件组织，`BufferedStorage` 的独立写线程消费有界输出缓冲，
串行写图像与元数据。业务层不包含 K230、MPP 或 VENC 头文件。
识别侧通过 `FrameChannel::try_submit()` 移交租约，忙时立即返回，不复制整帧或等待文件 I/O。
组件与方法类位于 `src/recorder/`，构建目标 `dart_recording` 在 `src/CMakeLists.txt` 声明。
此目录保留共享配置、错误、格式/解码、编码算法和文件系统适配方法；下层不依赖业务层。

当前 `dart_app` 接收相机层保留亮度的灰度 Y8，默认 PGM 与可选 PNG 都原样保留
8 位灰度。显式选择 `--rec-format dbv|pbm` 时，录像器按 `--threshold` 生成二值副本，
该副本不反馈给识别器，也无法在回放时恢复灰度。DBV 的无损指打包后的二值位图。
灰度 PGM 在 640×480 下每帧约 307 KB，60 FPS 时约 18.4 MB/s，500 MB 上限约容纳
27 秒；实际保存速度与时长取决于输入和写盘。启动清单使用 `--rec-fps 30 --rec-format pngs`，
减少写盘带宽和逐帧建文件开销。PNG 保留全部灰度值，使用 Up 行滤波与 `Z_RLE` 快速压缩。
本次移植参考远程 `uart-rt-log` 的 `1bd15cfce`、`606b58423`、`b320f32f6`，按当前头文件/源码结构编译。

## 类与消费方式

包含 `src/recorder/record.hpp`、`src/recorder/frame_channel.hpp`，链接
`dart_recording`（别名 `dart_recorder`）。创建时传入 `Component::Spec`、录像配置、
通道、状态及可选 `WriteControl`，通过 `RecorderFactory::create()` 取得组件，
交给 `executor::Tunner` 独立执行。目录字符串、通道、状态和写控制须覆盖执行期。

| 文件 | 职责 |
|---|---|
| `src/recorder/recorder.cpp` | 执行组件，组织生命周期、会话、输入、编码与缓冲提交 |
| `src/recorder/record.hpp` | `Record` 校验、时间采样、超时，以及配置入口和线程安全状态 |
| `src/recorder/frame_channel.hpp` | 单帧租约交接、归还及有界拒收事件队列 |
| `src/recorder/encode.hpp` | `Encoder` 组织格式处理，复用下层 PNG/DBV 编码类与预分配工作区 |
| `src/recorder/buffered_storage.hpp/.cpp` | 固定容量输出队列、独立文件写线程、成功写入事件与关闭排空 |
| `src/recorder/storage.hpp` | `Storage` 持有会话和文件适配类；不创建线程 |

`initializing()` 准备通道与编码工作区；`capture_inputs()` 处理写请求、会话并取得帧；
`update()` 检查有效期及缓冲容量后编码；`publish_outputs()` 将输出复制到独立缓冲并完成归还；
`shutingdowm()` 关闭接收、处理在途帧，等待后台排空后关闭文件。
下层 `recorder/storage.hpp` 封装 POSIX 文件调用，`recorder/encoding/encoders.hpp` 封装编码算法。
文件写线程串行写图像与元数据；磁盘停顿由有界输出缓冲吸收，满时拒收新帧。

## 放行和所有权

```cpp
// 在识别完成后的回调中；frames 是与录像组件共享的 FrameChannel。
(void)frames.drain_releases();
const auto decision = frames.try_submit(std::move(lease));
// kAccepted 才移交所有权；其他结果仍由 Detector 释放 lease。
```

通道最多持有一张原图：空闲、采样命中且上次租约已归还时才接收。
编码器直接按 `FrameView::stride` 读取原图；PGM 在 Recorder 线程按 stride
将原图行复制到紧凑输出槽后归还，其他格式先编码再复制结果。写线程只访问槽位拥有的字节，
不持有相机租约。这里的零拷贝仅指识别到录像的租约交接。

`Config::queue_slots` / `--rec-queue N` 设置输出槽数，默认 32，范围 1..64，
包括当前正在写入的槽。每槽按编码器声明的最大输出大小预分配，运行中不扩容；
640×480 H.264 每槽最多 679,936 字节，32 槽约 20.75 MiB，另计编码工作区和文件缓冲。
32 槽在 30 FPS 下约覆盖 1.07 秒输入，只吸收短时抖动，不提高持续写盘吞吐。
满时保留既有 FIFO 数据，丢弃新输入；`try_submit()` 返回 `kBusy`，
CSV 记 `buffer_full`，累计到 `Stats::buffer_dropped`；拒收同时累计 `busy`，
已经接管租约后在编码前发现缓冲满则只累计 `buffer_dropped`。
编码前丢弃，避免 H.264/DBV 更新参考帧后再漏写；已经编码入队的帧按顺序写出。
日志中的 `rec_buffered`、`rec_buffer_high_water`、`rec_buffer_dropped` 分别表示
当前输出槽占用、占用峰值和缓冲满丢帧数；`rec_queued` 仍表示原图租约通道的占用。

`accepted` 统计通道已接管的输入租约，`encoded` 统计编码成功，`enqueued` 统计独立输出入队，
`written` 统计图像写入成功；后台尚未排空时计数可以不同。编码、入队复制和文件写入分别记录耗时，
平均值的分母依次为编码尝试、入队成功和写入尝试次数，失败写入也计入 IO 总耗时及峰值。
会话结束事件保存最终快照和 `drain_us`，由应用输出 `[record] op=recorder_stop`，
正常退出时停止采集后等待组件完成排空；暂停期间的排空在录像组件执行线程上等待。
`RecorderStatus` 使用固定 8 条生命周期事件队列，应用通过 `pop_session_event()` 消费，
未及时消费而满时丢新事件并累计 `session_events_dropped()`，不影响图像写入。

`Config::fps` 是目标录像帧率，默认 `60` 表示每秒选取 60 帧，与采集帧率独立。
`dart_app --rec-fps N` 把它注入 Recorder；未指定时沿用 `Config::fps` 的默认值。
原 `sample_every_n` / `--rec-every` 已由这个参数替代。
采样使用 `FrameView::received` 的单调微秒时间，每个 `1/fps` 时间区间最多选一帧，
不依赖未经验证的源 PTS。时间区间先于队列检查，采集停顿或队列忙后不突发补帧，
已采样的帧租约进入单帧通道。Recorder 开始编码前检查租约入队以来的单调时钟耗时，
达到超时预算就丢弃，不补写过期帧；实际落盘帧率取决于输入、队列和写盘速度。
采样命中后，录像用 `try_to_lock` 和固定槽做放行：
槽满或队列锁正被占用都立即返回 `kBusy`，不等待，计数器
`Stats::busy` 增加。

`Config::frame_timeout` / `--rec-timeout-ms N` 设置输入帧等待编码的超时毫秒数，范围为 0..60000。
默认 0 表示一个目标帧周期，60 FPS 时为 16,667 µs；超时从租约移交时刻开始计算，
不使用源 PTS，也不要求输入时间戳与写线程具有相同起点。过期帧记为 CSV `timed_out`，
累计到 `Stats::timed_out` 和应用日志的 `rec_timed_out`。超时只决定是否开始编码；
编码后排队的帧不再按该超时丢弃，避免破坏参考链。编码或当前帧写入失败记为 `write_error`；
事件 CSV 写入或关闭错误也停止本轮并保留系统错误和路径。达到容量上限或写盘失败后，
停止录像并将后续尚未写出的队列帧记为 `stopped`。正常进程退出排空已入队输出；
暂停写请求时停止接收，并将尚未开始的输出记为 `stopped`。

`FrameChannel` 不保存整帧副本，移交与拒收路径只处理租约和元数据。
取帧线程每轮调用 `drain_releases()`，即使录像暂停或正忙也继续归还；失败租约保留重试，
归还完成前不复用槽位。Recorder 只把租约送回通道，不直接调用相机释放。
关闭时先停止 Runner，随后由取帧线程排空归还通道，最后关闭相机。
停止 Runner 不等待取帧线程归还，因此不会形成双方等待。


## 文件和对应记录

每次 `start()` 创建 `output_directory/rec_NNNN/`：

`Config::keep_sessions` 为 0 时保留全部轮次；大于 0 时在新建轮次前清理最旧的
`rec_NNNN`，将新轮次计入保留数量。开机清单使用 `--rec-keep 0 --rec-fps 30 --rec-format pngs`，
每次复位后保留之前的目录；不设 90 秒时限，也不自动重启续录。
清理只处理录像器自己的文件名（兼容旧版逐帧文件），遇到未知文件停止
并报错；`last_system_error()` 和 `last_error_path()` 给出系统错误及失败路径。

```text
img/recording.pgm
logs/frames.csv
```

`dart_app` 默认使用 `--rec-format pgm`，启动清单显式选择 `pngs`。PGM/DBV/PBM/PNGS 每轮只有一个图像文件，
直到每轮 500 MB 上限或显式停止后关闭；不额外创建细碎分段，避免 FAT 簇分配浪费。
本固件 MTP 无法读取仍被写入器打开的文件，应在停止录像后拷贝。
显式 `--rec-format png|pngs|dbv|pbm` 可切换格式；PNG 仍使用分组的独立帧文件。
`.script/player records/rec_NNNN` 自动识别全部格式，支持逐帧还原及导出 PNG。

`pngs` 保存 `img/recording.pngs`：16 字节 little-endian 头为 `DPNG`、`uint16` 版本 1、
`uint16` 头长 16、`uint32` 宽、`uint32` 高，随后依次追加完整的 8 位灰度 PNG。
文件只打开一次，每帧编码在录像线程完成，不使用逐帧 stdio 小块写入。
录像路径不显式调用 `fflush` 或 `fsync`，也没有周期刷新任务。
CSV 和使用 stdio 的图像格式在缓冲写满时自然写出，正常关闭时由 `fclose`
写出剩余数据；PNG/PNGS 直接使用 `write`，关闭时执行 `close`。
缓冲写满和关闭文件仍可能等待文件系统；这不提供异常断电时的持久化保证。
`frames.csv` 的第一帧偏移为 16，各帧长度只计算 PNG 本身；总容量和 `Stats::bytes` 包含容器头。
`python3 tools/rec_extract_pngs.py records/rec_NNNN --strict` 可按 CSV 提取独立 PNG；
播放器也可直接读取容器并忽略未完成的尾帧。

### DBV1 布局与压缩

文件头恰好 12 字节：`DBV1` 四字节标识、little-endian `uint32` 宽、`uint32` 高。
黑白像素按行打包，最高位为最左像素，1 表示白，行末不足 8 位补零。
宽高只存一次；帧号和时间戳复用 CSV，不在每帧再写一份。

每条记录为 unsigned LEB128 前缀 `(payload_bytes << 2) | kind`，随后紧跟 payload。
前缀通常 1–3 字节，不再额外存固定帧头、PNG 头或整帧 CRC。

| kind | payload | 含义 |
|---|---|---|
| 0，长度 0 | 无 | 旧版重复帧记录，仅保留读取兼容；新录像不写此记录 |
| 0，长度非零 | 稀疏 XOR | 每个变化字节写入 LEB128 跳过字节数和 1 字节 XOR 掩码 |
| 1 | 原始位图 | 不可压缩时保存打包位图，避免压缩膨胀 |
| 2 | zlib 流 | 完整位图，level 9，无损 |
| 3 | zlib 流 | 与上一帧 XOR 后的位图，level 9，无损 |

稀疏 XOR 的跳过计数从位图开头或上一个变化字节的下一字节开始，掩码必须非零。
编码器比较完整位图压缩、XOR 压缩、稀疏 XOR 和原始位图，保留最小结果。
不额外比较当前帧与上一帧是否相同；每个已保存帧都写入非空图像数据，相同画面也经过正常编码。
第一帧必须为原始或压缩完整帧；不强插周期关键帧。任意跳转可能需要从最近完整帧
重放差分，顺序浏览缓存上一张还原结果，旧版重复帧记录无需重复解码。

压缩记录复用 zlib 自带 Adler-32，不叠加 CRC；原始位图、稀疏差分和重复记录不附校验码。
因此原始/稀疏数据的静默位翻转无法保证检出，差分依赖链也不会隔离前帧错误。
记录边界和压缩校验仍会检查；末尾写到一半的记录会提示并忽略。
缺失 CSV 时仍可从文件头和记录结构重建图片索引，但无法恢复源帧时间及检测对应关系。
编码缓冲和 zlib 状态一次分配后复用，内存不随录像增长。

独立 PNG 使用 8 位灰度、level 1、每 32 张分组和逐图关闭；旧 1-bit PNG 仍可回放。

`Config::max_file_bytes` 默认且最多为 **500 MB = 500,000,000 字节**，PNG 模式按
**本轮所有 PNG 文件实际大小的总和**计算，包括 PNG 头和压缩数据，不包含 CSV/目录分配开销。
DBV 按整个 `recording.dbv` 的字节数计费，包括仅写一次的 12 字节文件头。
PNGS 按整个 `recording.pngs` 计费，包括 16 字节容器头，容量不足时不写半张 PNG。
Recorder 先编码，写线程再检查下一张完整图片能否放入预算；放不下时设置 `limit_reached()`，
停止接收并丢弃尚未落盘的队列图片，不生成超限文件或半张 PNG，不自动新建轮次续录。
因此最终总量可能略小于 500 MB。`Admission::kAccepted` 表示成功接管租约；
全部格式的 CSV `accepted` 行只在图片实际成功写完后生成，队列中因停录未写的帧记为 `stopped`，
等待超时的帧记为 `timed_out`。输出帧号、偏移和文件预算只计算成功保存的图像，
丢帧不会产生索引空洞或提前耗尽容量。
单帧通道与编码工作区不会随录像增长。应用只停录像，继续识别和 IMU。
PGM/PBM 同样在写线程中检查完整帧容量，剩余空间放不下下一帧时停止录像。

`dart_app` 对 `/data` 及其子目录启用 **2 GB = 2,000,000,000 字节**的分区总占用上限，
通过 `Config::max_storage_bytes` 注入；统计 `statvfs` 的已分配块，包含以前所有轮次、日志、
分区其他文件和 FAT 分配开销，不按 PNG 逻辑大小估算。其他输出分区不启用这个总量上限。
库配置为 0 时仅检查物理剩余空间。

启动时查询录像分区空间，预算取每轮上限、物理剩余空间、总量上限剩余空间的较小值，
并在后两者内预留 64 MiB 给日志及文件系统；达到保护线时不新建轮次。
DBV/PNG/PNGS 及启用总量上限的旧 PGM/PBM 流最多每 32 张重新查询空间，独立 PNG 每张按至少 2 MiB 扣减保守分配预算，
不足时提前重新查询；实际空间不足或查询失败时停录。PNG 目录采用两层短名称分组，
每个 p 目录最多 32 张、每个 b 目录最多 32 个 p 目录，限制目录扫描成本。
单文件格式共用文件分配簇，缓存预算按实际记录字节扣减，每次查询为尚未刷新的 256 KiB stdio 缓冲和簇对齐预留空间。
PNG 的分组限制目录扫描成本；DBV 从源头减少小文件分配。以前轮次不自动删除。
总量保护只控制录像写入；停录后识别和 IMU 继续，其日志仍可使用预留空间。

K230 构建必须链接 SDK 的 `libk230_syscalls.a`：该库提供 RT-Smart 原生 `statfs/statvfs`
适配；仅使用当前工具链的 libc 会得到全零容量，导致录像启动报 `storage/EOVERFLOW`。

CSV 首行的 `fps`、`frame_timeout_ms`、`queue_slots`、`max_file_bytes`、`file_limit_bytes`
记录目标帧率、编码前超时（0 表示一个目标帧周期）、输出槽数、容量上限和本轮有效上限，格式版本为 2。
CSV 对活动会话内每次提交尝试写一行（有界队列满时计数丢行），包含输入序号、源帧号、源 PTS、PTS 时间基有效位、单调接收时间、放行决定和
对应文件名、`file_offset`（帧头起始字节偏移）、`file_bytes`（实际文件/帧长度）；PNG 的 `file_offset` 固定为 0，CSV 行可能按写入完成顺序到达；
DBV 第一条记录的 offset 为 0、bytes 包含文件头；后续条目的 offset/bytes 是记录前缀及负载的准确范围。
`image_index` 为从 1 开始的连续输出帧编号。降采样、忙丢弃、超时和非法帧也有行；
未保存的帧不带图像文件名、偏移、长度或输出帧号。`Config::event_sink` 在文件写线程中收到同一条 `FrameEvent`，可接入上层更快的文件/数据记录，不会在 VICAP
回调中做 I/O。`event_slots` 也是有界的，满时只丢 metadata 并累计
`Stats::event_dropped`，不会反向阻塞采集。
