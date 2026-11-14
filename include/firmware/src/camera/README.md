# Camera ACL

上层通过 `ICamera`、`FrameLease` 和只读 `FrameView` 使用相机；帧格式由 `PixelFormat` 标明。
`dart_camera_types` 提供只读帧类型，`dart_camera_acl` 提供公共错误和租约实现，
两者均不依赖具体后端、C 采集实现或 SDK。
`dart_camera_k230_adapter` 供板级代码实例化 `Camera<N>`，承接 C ABI 与 SDK 编译配置；
启用 K230 时链接实际的 `dart_camera_k230` 数据面。C 数据面保留 `.c` / `.h`。

| 文件 | 类 / 职责 |
|---|---|
| `frame_view.hpp` | `FrameView`，L3 只读帧数据 |
| `interface.hpp` | `ICamera`、稳定输出尺寸、生命周期与有界取帧契约 |
| `frame_lease.cpp` | `FrameLease` 的移动与释放语义，以及后端槽位与受限构造入口 |
| `error.cpp` | 公共错误分类、上下文及有界诊断文本 |
| `k230/camera.hpp` | `Camera<N>` 及其 `Config`，K230 生命周期、曝光与 C ABI 适配 |
| `k230/error.hpp`、`k230/error_codes.h` | K230 原生错误码、SDK 位域翻译 |
| `k230/sensor.h`、`k230/sensor.c` | 内部 C 传感器操作表和选择入口 |
| `k230/exposure.c` | 曝光状态/范围检查及操作表分派 |
| `k230/sc038hgs.c` | SC038HGS 模式约束、曝光寄存器和 RAW10 灰度转换 |
| `k230/sc132gs.c` | SC132GS 默认模式/输出尺寸、经驱动 MPI 的曝光与模拟增益、亮度平面拷贝 |
| `k230/isp.c` | ISP 流水线的 VICAP 设备与 YUV420SP 通道配置 |

曝光设置通过 `Camera<N>::set_fixed_exposure_lines()` 完成：检查状态、查询后端
行数范围并钳位，后端负责曝光编码和帧长调整。请求值、范围和实际指令仅在诊断
上下文中保留，不再构造单独的规划结果或规划器。

```cpp
#include "camera/k230/camera.hpp"

using Camera = dart::camera::k230::Camera<4>;
Camera::Config config{};
config.initial_exposure_lines = 400;
auto created = Camera::create(config);
if (!created) {
    const auto& error = created.error(); // 直接持有错误值，无需解引用错误指针。
    // 根据 error.kind() / error.code() 决定恢复动作。
}
```

相机操作返回 `std::expected<T, CameraError>`。错误保存定长错误码、分类和上下文，
可平凡复制，不使用继承、虚函数或动态分配。原生错误码以及传感器模式、帧几何、
缓存、曝光和释放失败的专用诊断字段由公共错误值持有；K230 原生码的
module/level/errid 位域由适配层解释，写入错误值自有的定长诊断缓冲。
租约析构或移动覆盖时若释放失败，槽位用 `std::optional<CameraError>` 保存错误；
相机下一次受保护操作先重试释放，再继续工作。无帧和容量耗尽仍是正常取帧结果。

业务采集组件只依赖 `ICamera` 与公共相机库；算法和只读消费者按需使用
`FrameView` 与 `dart_camera_types`。每帧携带 `format`、真实宽、高和以字节计的 stride，
`valid()` 统一检查几何与缓冲长度，`row(y)` 取行首。`kY8` 每像素 1 字节灰度；
`kRgb565` 每像素一个小端 16 位字，R 在高 5 位（CanMV/OpenMV 内存布局）。
`frame_info()` 在工厂创建完成、启动采集前可用，给出尺寸、格式和 `pixel_aspect`
（单像素物理宽/高，ISP 非等比缩放时由后端给出，如 SC132GS 1080×1280→480×640 为 1.125）；
这些值在相机实例存续期间保持稳定。消费者按格式取舍：颜色识别只接受 `kRgb565`，
录像各编码器与 Wi-Fi JPEG 回传只接受 `kY8`，格式不符时由应用在启动阶段记录并关闭对应功能。
当前 K230 后端只输出 `kY8`：SC038HGS 为 RAW10 解包，SC132GS 为 ISP 输出的亮度平面；
SC132GS 的 RGB565 ISP 输出尚未接入，因此该相机上颜色识别仍按格式关闭。
租约持有期间像素有效，释放与相机操作归属采集线程，取帧/释放使用预分配存储。

曝光行数和 RAW10 DDR 布局属于 `camera::k230::Camera<N>::Config`，不出现在 `ICamera`
或通用应用参数中。`board/k230_arguments.hpp/.cpp` 解析原有 `--exp`、`--raw-layout`
和 `--ae` 选项；入口绑定配置到 `platform::CameraFactory` 的上下文，并持有它直到
`Application::run()` 返回。应用只调用工厂，不解释硬件配置。
具体类保留 `set_fixed_exposure_lines()`；公共采集接口不要求所有硬件提供按行曝光。

K230 工厂从固件可用模式中选择，CSI 来自 `CONFIG_MPP_SENSOR_DEFAULT_CSI`。
只有具体适配目标传播 `k230_rtos_app` 的 SDK 编译配置，C 后端也直接消费它。
RISC-V 模板适配器缺少此宏时编译失败，避免把 CSI2 板卡静默当作 CSI0 探测。
传感器操作表选择各型号的模式校验、曝光和像素转换；MPP、VB 池和 DMA 生命周期留在
K230 采集实现。当前注册 SC038HGS（MCM RAW10）和 SC132GS（ISP YUV420SP），
SC038HGS 的模式约束保持不变。
新增同平台传感器需提供实际驱动、操作实现及注册；其它平台可直接实现 `ICamera`。
硬件探测失败返回错误，应用停用采集并保留独立服务。
本次接口重构仅落盘，未运行主机测试、交叉构建或板端验证。

## SC038HGS / Dart V1

Dart V1 固件使用 CSI2 上的黑白 SC038HGS（640 × 480、RAW10 输入）。
后端使用 `VICAP_WORK_ONLY_MCM_MODE`，数据流为
`SC038HGS RAW10 → CSI/MCM → DDR → CPU 解包为灰度 Y8（RAW10 >> 2） → 识别/录像`。
不启动 ISP 图像处理，不加载 XML/JSON，不缩放；不支持的传感器、非原生输出尺寸
与自动曝光请求会明确报错。MCM 本身属于 ISP 子系统，但不经过 ISP 图像处理流水线。

本机 MPI 的 ONLY_MCM 仅接受 `VICAP_DUMP_YUV444` 选择器，并把 MCM 配置写成
默认格式/stride。`k230/mcm.c` 用进程内 `--wrap=ioctl`，仅在本相机初始化期间
将 MCM 配置改为 `VICAP_MCM_RAW10`、`width*2` 的保守 DMA 容量跨度和相机自建 VB 池。
本机 SDK 将该字节跨度误填入 MCM 输入宽度，造成首帧超时；初始化后、启流前，
后端校验寄存器符合已核查固件，再将 `0x90001204` 的输入宽度改回 640 像素，
回读确认。只改宽度，不改 DMA 地址、容量和中断；映射或配置检查失败时释放资源并报错。
此 dump 选择器直接取 MCM 缓冲，不进行 YUV 转换；返回帧仍被 SDK 错标为 YUV444，
具体标签为 `PIXEL_FORMAT_YVU_SEMIPLANAR_444`（29），不是 PACKAGE_444（42）。
stride 字段不可信。实测 RAW10 数据每行紧凑存储 800 字节，独立于保守 DMA 容量跨度。
帧验证使用此数据行跨度和所属 VB 池，原帧信息原样归还。
MCM 固定四槽环形数组，配置缓冲数固定为 4，不随 CPU 租约容量增长。
构建检查已核查的用户态/内核 VICAP 库 SHA256；清单由后端的
[`k230/mcm_abi.json`](k230/mcm_abi.json) 持有，正式构建与独立 RAW 探针共用。
更新 SDK 时必须重新核查私有接口，再更新这份清单。

`Camera<N>::Config::raw10_layout` / `--raw-layout` 默认 `packed10-lsb`：每 4 个 RAW10 像素
按低位连续打包为小端 40 位字。这与 CSI 线上 MIPI RAW10 的排列不同。
2026-10-20 实板渐变图全部 307,200 像素符合连续规律，8 帧短时到达约 96 FPS。
当前 K230 后端只接受这个已验证布局；通用解包工具仍保留三种候选格式供其它数据分析。

`c_api/raw10.c` 通过 `dart_camera_raw10_gray()` 执行 `raw10 >> 2`，输出线性灰度 Y8。
不做黑电平校正、gamma 或阈值二值化；每个公开租约拥有独立的预分配 CPU 灰度缓冲。
RAW dump 在转换完成后立即归还，录像持有灰度缓冲不会占用 DMA 环。
颜色识别不接受该灰度帧（见上）。默认 PGM 和可选 PNG 保留灰度，
显式选择 DBV/PBM 时才在录像器内按 `--threshold` 导出二值图。软件镜像在解包时完成。
C ABI 已升至 6，移除了相机二值阈值配置；相机消费者与 CanMV 解释器需一起重新编译。

应用默认通过曝光接口设置 **400 行**（`--exp 400`），K230 后端使用现有
`kd_mpi_sensor_reg_read/write`，将曝光编码 6400（`0x01900`）和帧长 `VTS=600`
放在同一寄存器组中提交，并读回核对。`--exp 0` 明确保留固件初始化的曝光。
无需修改 SDK 驱动默认值或重新烧录固件，也不依赖内部 Pixel CLK 的微秒换算。
像素时钟和 HTS 不变时，400 行曝光时间是原 128 行的 3.125 倍，帧长保持
`VTS=600`。`--rec-fps` 仅控制录像按接收时间抽帧。
开机清单使用 400 行曝光、VTS=600，为约 60 FPS 录像留出采集余量。

板端先验证 RAW10 排列、首帧、灰度图和稳定录像，再标定像素时钟、黑电平、
亮度响应与缓存一致性；主机模拟和交叉编译不能验证这些硬件行为。

## SC132GS / Dart V1 (RT-Smart)

The SC132GS image routes the sensor to CSI0 (4 lanes). The SDK probe prefers the
highest frame rate (640×480@240), so an automatic request re-probes the sensor's
default mode, 1080×1280@120. The backend mirrors the board-verified CanMV setup:
`VICAP_WORK_OFFLINE_MODE` with four VICAP-owned RAW input buffers; every ISP block
except AE, AWB/WB, 3DNR, AF and HDR; one YUV420SP channel at 480×640 with no crop
window, so the ISP scales the whole view (`pixel_aspect` 1.125). Outputs at the
native 1080 width are rejected: on CanMV they stalled the ISP and wedged stop().
This SDK ignores a caller-supplied channel pool, so the channel pool is VICAP's;
start() adopts it from the first frame and maps each block's luma plane (rows at a
16-pixel pitch). Each frame's luma is copied into the lease slot and the dump is
released at once, as on the RAW path. Mirror and AUTO exposure are rejected.

Exposure lines go through `kd_mpi_sensor_intg_time_set` (one line is 1/(120×1350) s,
about 6.17 µs; at most 1342 lines) and are read back; `--gain X` sets the analog
gain (1..28.5) through `kd_mpi_sensor_again_set`. Frame timing is unchanged.

Board results, 2026-11-14 (RTOS_K230D_Dart_V1_SC132GS image, `dart_app`):
capture held 120.0 fps for minutes with clean stop and repeated restarts. With
`--exp 400` and the initialized gain the luma range was 71..93; `--exp 1300
--gain 8` gave mean 142 (p1..p99 103..178) indoors. Recording to `/data`: PGM
reached ~16 fps (about 50–64 ms per 307 KB write), PNGS ~24 fps (~25 ms zlib per
frame); H.264 at `--rec-fps 60` wrote 10531 frames in 178.9 s (59.0 fps) in one
session, decoding with zero errors. Orientation, focus and color were not checked
against a known target.

## Exposure

The C++ and C exposure APIs use integer sensor lines; ABI version 5 rejects
configurations with the old microsecond semantics. `Camera<N>::Config::initial_exposure_lines`
defaults to zero (preserve initialization), while the application defaults to 400.
Startup and later adjustments use the same setter and line range, 4..65509.
The C++ adapter clamps requests; the C backend rejects values outside that range.

The SC038HGS backend chooses `VTS=max(600, lines+11)`, encodes exposure in
1/16-line units, and submits both values in Group 0. Reserved register bits and
sensor gain are preserved. A failed write, commit or readback faults the camera
until stop/deinit/init resets the sensor; startup failures roll back the stream.
The SDK's clock-dependent exposure/fps APIs and their timing cache are unused.
After changing frame length, the backend reports fps as unknown rather than
retaining the mode table's old rate. AUTO remains unsupported by the RAW backend.

## Ownership

Create one `dart_media_t` at startup and inject it into the camera. It owns global
VB initialization/exit. Do not coexist with independently initialized raw SDK
VB users, create multiple media owners, or destroy it before all consumers have
finished. This slice reserves DEV0/CHN0 and allows one attached camera per owner.
No application-level mutable global state is used to pretend these resources
are independent.

The camera owns a four-block RAW pool. During start, it uses one startup dump to
verify that pool, validates each existing block through the public VB handle
queries, and maps every block before returning to the application. The SDK's
documented pool/block handle encoding is used only for these startup queries;
no blocks are taken from VICAP or released through the VB API. Camera teardown
unmaps all blocks, deinitializes VICAP, then destroys its pool.
Acquisition invalidates the RAW DMA cache, converts into a per-slot CPU buffer,
and releases the RAW dump before returning a grayscale Y8 lease (`stride == width`).
No allocation, mapping or unmapping occurs in acquire/release.
The caller injects a lease capacity: up to `max_outstanding_frames` frames may be
outstanding at once, each carrying a distinct token, and release order is free. A
lease carries owner identity and token; stale/double/wrong-owner releases are
rejected. RAW release failure prevents publishing a frame; failed cleanup keeps
the dump for an explicit teardown retry and marks the camera faulted.
Stop/deinit return BUSY while any lease is outstanding. Copies of frame structs
are aliases, not additional references.

Capacity is bounded twice: by the ceiling in `error_codes.h`
(`DART_CAMERA_MAX_OUTSTANDING_FRAMES`) and by `channel_buffers`, which bounds CPU
grayscale lease slots. Requesting more is a configuration error. RAW DMA has a
separate fixed four-buffer ring.
When every slot is already held, acquire does not touch the driver at all: it
reports `DART_CAMERA_E_LEASE_EXHAUSTED`, the frame stays in the driver pool and is
recycled there. That is a drop, not a failure, so a pipeline counts it instead of
logging an error. On the C++ side the capacity is a compile-time parameter of the
factory (`dart::camera::k230::Camera<N>::create`), which sizes the lease slots statically and
removes the per-frame allocation; `capacity()` and `outstanding()` let a producer
apply its own bounded-drop policy before calling `acquire()`.

A lease must be released before the camera is destroyed. Its slot is owned by
the camera, so detachment does not extend slot lifetime. Accessing a lease after
camera destruction violates the lifetime contract. The C data plane refuses to
stop with active leases and retains handles when cleanup cannot complete.

Startup/shutdown may allocate/free. Shutdown order is release frame, camera
deinit, camera destroy, media destroy. Failed cleanup retains handles and pool
ownership; retry deinit, do not free or close the media owner. Internal rollback
failure sets FAULTED, preserves the primary and cleanup native codes and permits
cleanup retries. Objects must not be used after successful destroy.

The internal C ABI accepts zero sensor width/height/fps to select a supported
mode; zero output width/height follows that mode. Resolved geometry is validated
before allocation. A zero fps value means no rate request, and drivers may report
an unknown rate. Explicit internal diagnostic requests require a match for each
requested value. CSI port numbering is 0/1/2 and comes from firmware board configuration.
Output width must be a multiple of 32, height must be even, and output cannot
upscale. HDR is unsupported. Automatic selection cannot replace a missing sensor
driver or its initialization register table.

`received_us` is CLOCK_MONOTONIC receipt time, not exposure time. `source_pts`
is the untouched SDK timestamp with SOURCE_PTS_UNVALIDATED set. Its timebase,
sampling epoch and relationship to IMU time must be established on the board
before it enters control or tracking.

## Build

Host:

```sh
cmake -S . -B /tmp/dart-camera-host -DDART_CAMERA_K230=OFF
cmake --build /tmp/dart-camera-host --parallel
```

Board targets use installed SDK headers/libraries and an explicit toolchain:

```sh
cmake -S . -B /tmp/dart-camera-cross \
  -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-riscv64-musl.cmake \
  -DK230_TOOLCHAIN_DIR=/path/to/riscv64-musl/bin \
  -DK230_SDK_ROOT=/mnt/mydata/kRTOSSDK \
  -DDART_CAMERA_K230=ON
cmake --build /tmp/dart-camera-cross \
  --target dart_app
```

`K230_MPP_LIB_DIR` and `K230_LINK_SCRIPT` are explicit override points for other
SDK builds. The C++ ACL uses C++23 and `-fno-exceptions -fno-rtti`.
No C++23 fallback implementation is introduced. The v1 probe targets are removed.

## Validation

Host tests check C11 layout, C++23 expected, configuration bounds, fractional
clamping, microseconds-to-seconds conversion, invalid/nonfinite ranges, frame
lease transitions and adapter error propagation. Explicit test fakes are not
MPP compatibility code and cannot validate DMA or SDK rollback behavior.

On 2026-10-10, all four GNU 13.3 host tests passed. The installed RISC-V musl
GCC 12.0.1 toolchain compiled and statically linked the C camera probe, real C++
adapter probe and expected probe with the local SDK. Actual board execution is
**BLOCKED: hardware validation pending**.

On 2026-10-18, all 11 current host tests and the ASan/UBSan camera pool regression
passed, and the board application linked against the updated SDK. The pool
regression now covers monochrome YUV, padded Y stride, independent leases,
unknown-clock exposure errors, scene/parse/init rollback, restart and cleanup
failure retries. Run it with `bash .script/check-camera-pool.sh`.

On 2026-10-19, the SC038HGS mono XML/JSON scene and OFFLINE YUV420 application
completed a 90-second board run: 7183 frames acquired and 785 PGM images written,
with zero acquisition, detection, write or release errors. Camera stop/destroy
succeeded and the next launcher cycle acquired frames again. Returned 640x480
Y images are very dark (sample range 9..19); exposure and image quality remain
unvalidated. The observed application rate is not a sensor clock measurement.
Evidence: `records/camera-yuv-bringup-2026-10-19/RESULT.md`.

The preceding dated YUV board results describe the former ISP path. On 2026-10-19,
the RAW10/binary path passed host tests, the ASan/UBSan lifecycle regression and
K230 cross linking. Real RAW10 packing, DMA behavior and image quality remain
pending board validation; those YUV results do not validate this path.

Board acceptance must verify SDK pool compatibility, cache freshness under
DMA, stable frame timing, actual exposure/line quantization, warm restart and
cleanup/release failures. Correlate raw PTS with a known timing reference and
inspect real images; compile/link success and a single luma sample are not
evidence of visual correctness or performance.
