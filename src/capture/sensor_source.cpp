#include "capture/sensor_source.hpp"

#include <cstdio>
#include <unistd.h>

#include "core/check.hpp"
#include "core/log.hpp"
#include "core/mpp_map.hpp"

namespace dart {
namespace {

constexpr k_vicap_dev kDev = VICAP_DEV_ID_0;
constexpr k_vicap_chn kChn = VICAP_CHN_ID_0;

// 板端没有shell，不要进行错误进fatal的逻辑处理，而是为错误编号，在日志中打印进入哪个进程，出现什么错误码
k_s32 ok(k_s32 ret, const char *what) {
    if (ret != K_SUCCESS)
        die(what, ret);
    return ret;
}

} // namespace

SensorSource::SensorSource(const Config &cfg) : cfg_(cfg) {
    // ① 探测传感器：适配表按 (宽, 高, fps) 精确匹配；匹配不上会静默落到别的模式，
    //    所以下面一定要回读——绝不相信请求值。
    k_vicap_probe_config probe{};
    probe.csi_num = static_cast<k_u32>(cfg_.csi);
    probe.width = cfg_.sensor_width;
    probe.height = cfg_.sensor_height;
    probe.fps = cfg_.sensor_fps;
    // 板端无 shell，以下每条进度行都是排障用的锚点：卡在哪一步，日志就停在哪一行
    log_line("vicap: 探测 sensor %ux%u@%u csi%d ...\n", cfg_.sensor_width, cfg_.sensor_height, cfg_.sensor_fps,
                cfg_.csi);
    if (kd_mpi_sensor_adapt_get(&probe, &sensor_) != 0)
        die("sensor 探测失败：该 (宽,高,fps) 不在适配表里");
    ok(kd_mpi_vicap_get_sensor_info(sensor_.sensor_type, &sensor_), "get_sensor_info");

    acq_w_ = sensor_.width;
    acq_h_ = sensor_.height;
    acq_fps_ = sensor_.fps;
    log_line("vicap: 实际模式 %ux%u@%u type=%d\n", acq_w_, acq_h_, acq_fps_, static_cast<int>(sensor_.sensor_type));

    // ② VB：所有硬件缓冲的总池，必须最先建
    k_vb_config vb{};
    vb.max_pool_cnt = 64;
    ok(kd_mpi_vb_set_config(&vb), "vb_set_config");
    ok(kd_mpi_vb_init(), "vb_init");
    log_line("vicap: VB 就绪\n");

    // ③ 设备：采集窗口 = 传感器实际模式。OFFLINE + 通道缩放器是板端验证过的组合
    //    （双通道/ONLINE 会在若干帧后永久 NOTREADY，已放弃）。
    k_vicap_dev_attr dev{};
    dev.acq_win.width = acq_w_;
    dev.acq_win.height = acq_h_;
    dev.mode = VICAP_WORK_OFFLINE_MODE;
    dev.buffer_num = cfg_.cap_buffers;
    dev.buffer_size = VB_ALIGN_UP(static_cast<k_u64>(acq_w_) * acq_h_ * 2, 4096);
    dev.buffer_pool_id = VB_INVALID_POOLID;
    dev.pipe_ctrl.data = 0xFFFFFFFF; // ISP 全模块打开
    // 下面四位必须逐位覆盖，不能只靠 0xFFFFFFFF：
    //   ahdr_enable 默认会被置 1，而 gc2093 这几档都是 SENSOR_MODE_LINEAR（线性），
    //   ISP 在 HDR 配置下去连 sensor 会直接失败（板端实测：kd_mpi_isp_connect vsi connect failed）。
    dev.pipe_ctrl.bits.ae_enable = (cfg_.exposure_us == 0) ? K_TRUE : K_FALSE; // 固定曝光时关 AE
    dev.pipe_ctrl.bits.awb_enable = K_TRUE;
    dev.pipe_ctrl.bits.ahdr_enable = K_FALSE;
    dev.pipe_ctrl.bits.dnr3_enable = K_TRUE;
    dev.sensor_info = sensor_;
    ok(kd_mpi_vicap_set_dev_attr(kDev, dev), "vicap set_dev_attr");
    log_line("vicap: dev 配置完成（缓冲 %u x %uB）\n", cfg_.cap_buffers, static_cast<unsigned>(dev.buffer_size));

    // ④ CHN0：缩放到识别/录像画幅（1280x720 -> 640x360，精确 2:1）
    k_vicap_chn_attr chn{};
    chn.out_win.width = cfg_.width;
    chn.out_win.height = cfg_.height;
    chn.crop_win = dev.acq_win;
    chn.scale_win = chn.out_win;
    chn.crop_enable = K_FALSE;
    chn.scale_enable = K_TRUE;
    chn.chn_enable = K_TRUE;
    chn.pix_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
    chn.buffer_num = cfg_.chn_buffers;
    chn.buffer_size = VB_ALIGN_UP(static_cast<k_u64>(cfg_.width) * cfg_.height * 3 / 2, 4096);
    chn.alignment = 12;
    chn.buffer_pool_id = VB_INVALID_POOLID;
    ok(kd_mpi_vicap_set_chn_attr(kDev, kChn, chn), "vicap set_chn_attr");
    log_line("vicap: CHN0 配置 %ux%u（缩放器开）\n", cfg_.width, cfg_.height);
    ok(kd_mpi_vicap_init(kDev), "vicap init");

    // ⑤ 拿 sensor 句柄：固定曝光/增益都从这里下发
    k_vicap_sensor_attr sattr{};
    sattr.dev_num = kDev;
    ok(kd_mpi_vicap_get_sensor_fd(&sattr), "vicap get_sensor_fd");
    sensor_fd_ = sattr.sensor_fd;
    log_line("vicap: init 完成 (sensor fd=%d)\n", sensor_fd_);
}

SensorSource::~SensorSource() {
    stop();
    kd_mpi_vicap_deinit(kDev);
    kd_mpi_vb_exit(); // VB 是全局的：必须等 FramePool/Recorder 都析构完（靠 main 里的声明顺序）
}

void SensorSource::start() {
    log_line("vicap: 启动取流 ...\n");
    ok(kd_mpi_vicap_start_stream(kDev), "vicap start_stream");
    started_ = true;
    apply_exposure(); // 曝光必须等流出起来再下发
}

// 固定曝光下发（内部）：k_sensor_intg_time 的单位是秒（不是微秒）
static bool set_intg_time(k_s32 fd, uint32_t us) {
    const k_sensor_intg_time intg = [] (uint32_t v) {
        k_sensor_intg_time t{};
        t.intg_time[0] = static_cast<float>(v) / 1000000.0f;
        return t;
    }(us);
    if (kd_mpi_sensor_intg_time_set(fd, intg) != 0) {
        log_line("曝光: 下发 %u us 失败\n", us);
        return false;
    }
    return true;
}

void SensorSource::apply_exposure() {
    if (cfg_.exposure_us == 0) // 0 = 交回 AE，不手工下发
        return;

    if (!set_intg_time(sensor_fd_, cfg_.exposure_us))
        die("设置固定曝光失败");

    k_sensor_exposure_time_range range{};
    if (kd_mpi_sensor_get_exposure_time_range(sensor_fd_, &range) == 0)
        log_line("曝光 %u us（该模式允许 %.0f~%.0f us），AE 已关\n", cfg_.exposure_us, range.min_intg_time_us,
                    range.max_intg_time_us);
}

void SensorSource::stop() {
    if (!started_)
        return;
    kd_mpi_vicap_stop_stream(kDev);
    started_ = false;
}

SensorSource::RawFrame SensorSource::capture() {
    RawFrame raw;

    // 有界取帧：拿不到就立刻返回空帧，由调用方跳过这一拍。
    // 绝不在这里死等 —— MPP 一旦不出帧，死循环会把日志和退出路径一起堵死
    // （板端实测：卡在这一行，150 秒里一个字都打不出来，录像文件停在 0 字节）。
    const k_s32 ret = kd_mpi_vicap_dump_frame(kDev, kChn, VICAP_DUMP_YUV, &raw.info_, 200);
    if (ret != K_SUCCESS) {
        if (dump_fails_ < 3 || (dump_fails_ % 200) == 0) // 头几次 + 之后约每 20 秒一条
            log_line("vicap: 取帧失败 ret=0x%x（连续 %u 次）\n", static_cast<unsigned>(ret), dump_fails_ + 1);
        ++dump_fails_;
        return raw; // y == nullptr，调用方这一拍跳过
    }
    dump_fails_ = 0;
    raw.owner_ = this;

    const k_video_frame &vf = raw.info_.v_frame;
    raw.width = vf.width;
    raw.height = vf.height;
    raw.stride = vf.stride[0];
    raw.mod_id = raw.info_.mod_id;          // 池帧要照抄这个值
    raw.pts = raw.info_.v_frame.pts;        // 取证：源帧时间戳
    // GraphicsUtils::binarize 按紧凑排列（stride == width）写，VICAP 一旦加了行填充
    // 会静默输出斜切图，所以这一条假设必须在这里当面验证（640 宽本来就是 32 对齐，正常不会触发）
    check(raw.stride == raw.width, "VICAP 帧 stride != width：二值化要求紧凑排列");
    // 映射策略（对齐老工程 vision.c 的板端验证做法）：
    //   * **持久**映射，绝不逐帧 munmap；
    //   * 用 **cached** 映射：二值化要逐字节读这 230KB，non-cache 读是每字节十几次总线
    //     周期，会把 11ms 的帧预算吃掉一大块；
    //   * ISP/VICAP 是 DMA 写入方，所以**每次读之前必须作废 cache 行**（否则读到上一帧）。
    raw.map_size_ = static_cast<uint32_t>(static_cast<k_u64>(raw.stride) * raw.height);
    raw.map_ = mpp_map_persist_cached(vf.phys_addr[0], raw.map_size_);
    if (raw.map_ == nullptr) {
        kd_mpi_vicap_dump_release(kDev, kChn, &raw.info_); // 拿不到 VA 就立刻还帧
        return raw;
    }
    mpp_invalidate(vf.phys_addr[0], raw.map_, raw.map_size_); // DMA 刚写完 → 旧行全作废
    raw.y = static_cast<const uint8_t *>(raw.map_);

    if (frame_count_++ == 0) // 第一帧到手：确认 MPP 真的在出帧（带实际 stride）
        log_line("vicap: 第一帧到手 %ux%u stride=%u\n", raw.width, raw.height, raw.stride);
    return raw;
}

SensorSource::RawFrame::RawFrame(RawFrame &&o) noexcept
    : y(o.y), width(o.width), height(o.height), stride(o.stride), pts(o.pts), owner_(o.owner_),
      info_(o.info_), map_(o.map_), map_size_(o.map_size_) {
    o.y = nullptr;
    o.owner_ = nullptr;
    o.map_ = nullptr;
}

SensorSource::RawFrame::~RawFrame() {
    if (!owner_)
        return;
    // 只还帧，不解映射：VA 由 mpp_map_persist 统一持有到进程退出
    kd_mpi_vicap_dump_release(kDev, kChn, &info_);
}

} // namespace dart
