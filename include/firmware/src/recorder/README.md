# 录像 / 图像证据

`dart::recorder::Recorder` 提供异步录像服务。它只接收相机 ACL 的
`FrameLease`，不包含 K230、MPP 或 VENC 头文件；提交时将输入帧拷贝为紧凑
Y8，后台线程默认写入灰度 PGM 连续流，也支持独立灰度 PNG 和二值 DBV/PBM。硬件 H.264 需要后续独立的 video ACL，
不会把厂商句柄带入录像服务。本模块是 `include/` 中的下层实现库，
包含线程管理、编码和具体文件系统操作；L4 的 `src/recording/` 只决定参数与启停。
构建目标在 `include/CMakeLists.txt` 中声明。

当前 `dart_app` 接收相机层保留亮度的灰度 Y8，默认 PGM 与可选 PNG 都原样保留
8 位灰度。显式选择 `--rec-format dbv|pbm` 时，录像器按 `--threshold` 生成二值副本，
该副本不反馈给识别器，也无法在回放时恢复灰度。DBV 的无损指打包后的二值位图。
灰度 PGM 在 640×480 下每帧约 307 KB，60 FPS 时约 18.4 MB/s，500 MB 上限约容纳
27 秒；实际保存速度与时长取决于输入和写盘。可用 8 位灰度 PNG 减少体积，其压缩耗时需上板测量。

## 类与消费方式

包含 `recorder/recorder.cpp`，并通过 `dart_recording`（别名 `dart_recorder`）
INTERFACE 目标取得相机 C 数据面、线程和 zlib 依赖。类外方法均为 `inline`，可供多个
翻译单元共同包含。`dart_recording_checks` 逐文件编译全部类。

| 文件 | 职责 |
|---|---|
| `recorder.cpp` | `Recorder`，采样放行、图像队列、相机租约释放及图像消费线程 |
| `async/event_worker.cpp` | `EventWorker`，有界元数据队列、消费线程及排空关闭 |
| `storage/session.cpp` | `Session`，轮次编号、目录创建与保留清理 |
| `storage/image_writer.cpp` | `ImageWriter`，帧拷贝、DBV/独立 PNG/旧连续流写入和容量检查 |
| `storage/image_files.cpp` | `ImageFiles`，PNG 分组路径、目录创建与逐帧可用空间预算 |
| `storage/storage_space.cpp` | `StorageSpace`，查询分区可用空间及配置的总占用预算 |
| `encoding/binary_format.cpp` | `BinaryFormat`，DBV1 文件头、变长记录前缀与边界检查 |
| `encoding/binary_encoder.cpp` | `BinaryEncoder`，1-bit 打包、完整帧/差分/稀疏差分自适应压缩 |
| `encoding/binary_decoder.cpp` | `BinaryDecoder`，按记录还原位图，供播放器和回归使用 |
| `encoding/png_encoder.cpp` | `PngEncoder`，8-bit 灰度、快速 DEFLATE 和 PNG 封装 |
| `storage/event_writer.cpp` | `EventWriter`，事件 CSV、缓冲与刷新 |
| `error.cpp` | `RecorderError`，错误状态、系统错误码与路径 |
| `config.cpp` | 录像配置、事件与事件回调的共享契约 |

图像队列槽位是 `Recorder` 的私有类型；释放队列直接保存租约，事件队列直接
保存事件。队列计数维护容量与有效区间，图像写入器不依赖队列类型。
`ImageWriter` 持有逐帧文件管理状态，`Session` 只负责轮次目录、保留策略与启动预算；
`BinaryEncoder` / `PngEncoder` 只做内存编码，不访问文件系统。
`EventWorker` 独立管理元数据队列和写线程；停止时先排空图像队列，再排空元数据队列，
确保图像写入成功后产生的事件仍能写出。`Recorder` 保留图像队列与相机租约的共同容量约束，
不再维护元数据线程的内部状态。

## 放行和所有权

```cpp
auto acquired = camera.acquire(std::chrono::milliseconds{0});
if (acquired && acquired->status == dart::camera::AcquireStatus::kFrame) {
    const auto decision = recorder.try_submit(std::move(acquired->lease));
    if (decision != dart::recorder::Admission::kAccepted) {
        // try_submit 未接管 lease；在采集线程释放。
        (void)acquired->lease.release();
    }
    recorder.drain_releases();
}
```

`Config::fps` 是目标录像帧率，默认 `60` 表示每秒选取 60 帧，与采集帧率独立。
`dart_app --rec-fps N` 把它注入 Recorder；未指定时沿用 `Config::fps` 的默认值。
原 `sample_every_n` / `--rec-every` 已由这个参数替代。
采样使用 `FrameView::received` 的单调微秒时间，每个 `1/fps` 时间区间最多选一帧，
不依赖未经验证的源 PTS。时间区间先于队列检查，采集停顿或队列忙后不突发补帧，
已采样的图像进入固定队列吸收短时写盘抖动。输入帧率不足或队列满时，实际落盘帧率会低于目标。
采样命中后，录像用 `try_to_lock` 和固定槽做放行：
槽满或队列锁正被占用都立即返回 `kBusy`，不等待，计数器
`Stats::busy` 增加。

`queue_slots` 分别限制已拷贝图像和待释放租约两个队列。`try_submit()` 只做有界内存
拷贝（640×480 实测约 0.2 ms），不做文件 I/O；拷贝后立即把租约交给 release 队列。
采集线程每拍提交前后调用 `drain_releases()`，保证相机 ACL 的调用仍然串行。
写线程只使用独立像素，不持有相机帧。应用使用固定 16 帧图像队列，640×480 Y8
共 4,915,200 字节，约可缓冲 267 ms 的 60 FPS 输入，覆盖实测 134 ms 的写盘停顿。
停止录像后也要先
drain，随后才能停止/销毁相机。同一个 Recorder 可以在 stop 并排空租约后再次 start，
复用缓冲并新建录像轮次；仍有租约时拒绝重启，避免丢失归还责任。
应用通过 `start_controlled(writes)` 在后台执行启停，暂停时采集与识别继续运行。
外部开关、runtime 接口和清理行为见 [写入控制](../../../../runtime/WRITE_CONTROL.md)。

## 文件和对应记录

每次 `start()` 创建 `output_directory/rec_NNNN/`：

`Config::keep_sessions` 为 0 时保留全部轮次；大于 0 时在新建轮次前清理最旧的
`rec_NNNN`，将新轮次计入保留数量。开机清单使用 `--rec-keep 0 --rec-fps 60`，
每次复位后保留之前的目录；不设 90 秒时限，也不自动重启续录。
清理只处理录像器自己的文件名（兼容旧版逐帧文件），遇到未知文件停止
并报错；`last_system_error()` 和 `last_error_path()` 给出系统错误及失败路径。

```text
img/recording.pgm
logs/frames.csv
```

`dart_app` 默认及启动清单使用 `--rec-format pgm`。PGM/DBV/PBM 每轮只有一个图像文件，
直到每轮 500 MB 上限或显式停止后关闭；不额外创建细碎分段，避免 FAT 簇分配浪费。
本固件 MTP 无法读取仍被写入器打开的文件，应在停止录像后拷贝。
显式 `--rec-format png|dbv|pbm` 可切换格式；PNG 仍使用分组的独立帧文件。
`.script/player records/rec_NNNN` 自动识别全部格式，支持逐帧还原及导出 PNG。

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

PNG 使用 8-bit 灰度、level 1、每 32 张分组和逐图关闭行为，旧 1-bit PNG 仍可回放。

`Config::max_file_bytes` 默认且最多为 **500 MB = 500,000,000 字节**，PNG 模式按
**本轮所有 PNG 文件实际大小的总和**计算，包括 PNG 头和压缩数据，不包含 CSV/目录分配开销。
DBV 按整个 `recording.dbv` 的字节数计费，包括仅写一次的 12 字节文件头。
写线程先编码，再检查下一张完整图片能否放入预算；放不下时设置 `limit_reached()`，
停止接收并丢弃尚未落盘的队列图片，不生成超限文件或半张 PNG，不自动新建轮次续录。
因此最终总量可能略小于 500 MB。`Admission::kAccepted` 表示成功拷贝进队列；
DBV/PNG 的 CSV `accepted` 行只在图片实际成功写完后生成，队列中因停录未写的帧记为 `stopped`。
固定队列与压缩缓冲不会随录像增长。应用只停录像，继续识别和 IMU。
旧连续流模式仍在提交时为固定帧大小预留空间。

`dart_app` 对 `/data` 及其子目录启用 **2 GB = 2,000,000,000 字节**的分区总占用上限，
通过 `Config::max_storage_bytes` 注入；统计 `statvfs` 的已分配块，包含以前所有轮次、日志、
分区其他文件和 FAT 分配开销，不按 PNG 逻辑大小估算。其他输出分区不启用这个总量上限。
库配置为 0 时仅检查物理剩余空间。

启动时查询录像分区空间，预算取每轮上限、物理剩余空间、总量上限剩余空间的较小值，
并在后两者内预留 64 MiB 给日志及文件系统；达到保护线时不新建轮次。
DBV/PNG 及启用总量上限的旧 PGM/PBM 流最多每 32 张重新查询空间，PNG/旧流期间每张按至少 2 MiB 扣减保守分配预算，
不足时提前重新查询；实际空间不足或查询失败时停录。PNG 目录采用两层短名称分组，
每个 p 目录最多 32 张、每个 b 目录最多 32 个 p 目录，限制目录扫描成本。
DBV 共用文件分配簇，缓存预算按实际记录字节扣减，每次查询为尚未刷新的 256 KiB stdio 缓冲和簇对齐预留空间。
PNG 的分组限制目录扫描成本；DBV 从源头减少小文件分配。以前轮次不自动删除。
总量保护只控制录像写入；停录后识别和 IMU 继续，其日志仍可使用预留空间。

K230 构建必须链接 SDK 的 `libk230_syscalls.a`：该库提供 RT-Smart 原生 `statfs/statvfs`
适配；仅使用当前工具链的 libc 会得到全零容量，导致录像启动报 `storage/EOVERFLOW`。

CSV 首行的 `fps`、`max_file_bytes`、`file_limit_bytes` 记录目标帧率、配置上限和本轮有效上限，格式版本为 2。
CSV 对每次提交尝试写一行（有界队列满时计数丢行），包含输入序号、源帧号、源 PTS、PTS 时间基有效位、单调接收时间、放行决定和
对应文件名、`file_offset`（帧头起始字节偏移）、`file_bytes`（实际文件/帧长度）；PNG 的 `file_offset` 固定为 0，CSV 行可能按写入完成顺序到达；
DBV 第一条记录的 offset 为 0、bytes 包含文件头；后续条目的 offset/bytes 是记录前缀及负载的准确范围。
`image_index` 仍为从 1 开始的输出帧编号。降采样、忙丢弃和非法帧也有行。`Config::event_sink` 在 metadata
写线程中收到同一条 `FrameEvent`，可接入上层更快的文件/数据记录，不会在 VICAP
回调中做 I/O。`event_slots` 也是有界的，满时只丢 metadata 并累计
`Stats::event_dropped`，不会反向阻塞采集。
