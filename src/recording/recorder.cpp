#include "record/recorder.hpp"

#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <cstdlib>
#include <vector>
#include <cstring>

#include "core/check.hpp"
#include "core/log.hpp"
#include "core/mmz.hpp"
#include "core/mpp_map.hpp"

#include "mpi_sys_api.h"
#include "mpi_vb_api.h"

namespace dart {
namespace {

constexpr uint32_t kMaxPacks = 32;    // 单次取流最大包数（预分配，老工程同款）
constexpr uint64_t kVencStatusMs = 5000; // 一直取不到码流时，每 5 秒问一次编码器内部状态
constexpr uint32_t kLabelMaxChars = 40; // 640 宽的一半（8x8 点阵）

constexpr int32_t kSendToMs = 1000;   // **有界**送帧：编码器不收就丢这一帧，绝不把录像线程钉死
constexpr int32_t kGetToMs = 100;     // 稳态立刻返回（送一帧必出一包）
constexpr uint32_t kBurstMax = 4;     // 送帧后最多连取几次流（编码器可能攒了多帧）
constexpr int32_t kGetEndToMs = 1000; // 收尾：等编码器把最后几帧吐出来
constexpr uint64_t kFlushMs = 1000;   // 码流落盘周期（老工程 FS_FLUSH_MS）
constexpr uint64_t kInflightMaxMs = 700;  // 单块在途超过这么久还没回码流 → 认定编码器丢了它，强收

char g_file_buf[1 << 20];           // 落盘缓冲：SD 卡抖动不至于反压到编码取流
k_venc_pack g_packs[kMaxPacks];     // 包数组预分配，避免每次取流 malloc

uint64_t mono_ms() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000 + static_cast<uint64_t>(ts.tv_nsec) / 1000000;
}

// 8x8 点阵字模：行优先，bit7 = 最左像素，字形占 5x7。只收录标注用得到的字符，
// 不链 freetype（它要字体文件、体积大，而这里每帧只画 40 个字符）。
struct Glyph {
    char    c;
    uint8_t rows[8];
};

constexpr Glyph kGlyphs[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00}},
    {'0', {0x70, 0x88, 0x98, 0xA8, 0xC8, 0x88, 0x70, 0x00}},
    {'1', {0x20, 0x60, 0x20, 0x20, 0x20, 0x20, 0x70, 0x00}},
    {'2', {0x70, 0x88, 0x08, 0x10, 0x20, 0x40, 0xF8, 0x00}},
    {'3', {0x70, 0x88, 0x08, 0x30, 0x08, 0x88, 0x70, 0x00}},
    {'4', {0x30, 0x50, 0x90, 0xF8, 0x10, 0x10, 0x10, 0x00}},
    {'5', {0xF8, 0x80, 0xF0, 0x08, 0x08, 0x88, 0x70, 0x00}},
    {'6', {0x30, 0x40, 0x80, 0xF0, 0x88, 0x88, 0x70, 0x00}},
    {'7', {0xF8, 0x08, 0x10, 0x20, 0x40, 0x40, 0x40, 0x00}},
    {'8', {0x70, 0x88, 0x88, 0x70, 0x88, 0x88, 0x70, 0x00}},
    {'9', {0x70, 0x88, 0x88, 0x78, 0x08, 0x10, 0x60, 0x00}},
    {'D', {0xF0, 0x88, 0x88, 0x88, 0x88, 0x88, 0xF0, 0x00}},
    {'F', {0xF8, 0x80, 0x80, 0xF0, 0x80, 0x80, 0x80, 0x00}},
    {'R', {0xF0, 0x88, 0x88, 0xF0, 0xA0, 0x90, 0x88, 0x00}},
    {'T', {0xF8, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00}},
    {'X', {0x88, 0x88, 0x50, 0x20, 0x50, 0x88, 0x88, 0x00}},
    {'Y', {0x88, 0x88, 0x50, 0x20, 0x20, 0x20, 0x20, 0x00}},
};

const uint8_t *glyph_rows(char c) {
    for (const Glyph &g : kGlyphs)
        if (g.c == c)
            return g.rows;
    return kGlyphs[0].rows; // 未知字符按空格留白
}

// 把一行文字烧到 Y 平面左上角（8 像素高），只写白字。行长由 annotate() 保证 ≤ 半行。
void draw_label(uint8_t *pixels, uint32_t stride, const char *text) {
    const size_t len = std::strlen(text);

    for (size_t i = 0; i < len; ++i) {
        const uint8_t *rows = glyph_rows(text[i]);
        for (uint32_t r = 0; r < 8; ++r) {
            if (!rows[r])
                continue;
            uint8_t *p = pixels + static_cast<size_t>(r) * stride + i * 8;
            for (uint32_t b = 0; b < 8; ++b)
                if (rows[r] & (0x80u >> b))
                    p[b] = 255;
        }
    }
}

} // namespace

Recorder::Recorder(const Config &cfg, FramePool &pool) : cfg_(cfg), pool_(pool) {
    // 预留帧数必须跟着帧池大小走：上一版硬编码 4，而 --mem-slim 把帧池砍到 6 槽，
    // 于是"空闲 > 4"永远不成立 → 每一帧都被 submit() 丢掉（录像 0 帧、队列丢 7900+）。
    reserve_frames_ = cfg.frame_slots / 4;
    if (reserve_frames_ < 1)
        reserve_frames_ = 1;
}

Recorder::~Recorder() {
    stop();
}

bool Recorder::start() {
    const uint64_t t0 = mono_ms();

    // ① VENC 输出池（码流从这里出）：块大小照老工程（码流包不会超过一帧原始大小）
    out_blk_bytes_ = static_cast<uint32_t>(
        VB_ALIGN_UP(static_cast<k_u64>(cfg_.width) * cfg_.height, 4096));
    // 建池之前先量一次 MMZ：编码器的内部缓冲（VPU 固件工作区/参考帧）也从这里出，
    // "收帧不吐码流"最可能就是被我们自己的池饿死，这个数字是判据。
    in_blk_cnt_ = cfg_.rec_in_blocks > kInBlkMax ? kInBlkMax : cfg_.rec_in_blocks;
    log_line("录像: MMZ 余量(建池前) %u MB（单块最大 %u MB）\n", mmz_free_mb(), mmz_max_block_mb());

    venc_pool_id_ = kd_mpi_vb_create_pool_ex(out_blk_bytes_, cfg_.rec_out_blocks, VB_REMAP_MODE_NOCACHE);
    if (venc_pool_id_ == static_cast<int32_t>(VB_INVALID_POOLID)) {
        log_line("录像: 创建 VENC 输出池失败（%u 字节 x %u），本次不录像\n", out_blk_bytes_,
                 cfg_.rec_out_blocks);
        return false;
    }

    // ③ 输入块：启动时一次取满，运行期永不再 get_block（探针要用块内存做合成帧，先建块）
    y_bytes_ = cfg_.width * cfg_.height; // 池帧保证 stride == width（紧凑排列）
    enc_h_ = cfg_.height;
    in_blk_bytes_ = static_cast<uint32_t>(VB_ALIGN_UP(static_cast<k_u64>(y_bytes_) * 3 / 2, 4096));
    if (!prepare_in_blocks()) {
        teardown_venc();
        return false;
    }

    // ② 通道：cfg_.venc_chn 可能被别的子系统占着（K230 上显示/图传会用 chn0），
    //    依次试几个通道；全都不行就放弃录像，**不杀进程**。
    if (cfg_.venc_probe)
        probe_encoder(); // 逐个候选配置试编，并把能出码流的那套参数采纳下来

    chn_ = cfg_.venc_chn;
    bool chn_ok = chn_ready_;
    for (uint32_t k = 0; k < 4 && !chn_ok; ++k) {
        const uint32_t chn = (cfg_.venc_chn + k) % 4;
        if (setup_chn(chn, adopt_mjpeg_ ? K_PT_JPEG : K_PT_H264, enc_h_, cfg_.sensor_fps,
                      adopt_mjpeg_ ? 0 : cfg_.gop, adopt_mjpeg_ ? 0 : cfg_.bitrate_kbps,
                      adopt_mjpeg_ ? 80 : 0, cfg_.venc_intbuf_mb)) {
            chn_ = chn;
            chn_ok = true;
        }
    }
    if (!chn_ok) {
        log_line("录像: 没有可用的 VENC 通道（试过 %u..%u），本次不录像\n", cfg_.venc_chn,
                 (cfg_.venc_chn + 3) % 4);
        // 上一版在这里直接销毁输出池 —— 而自检留下的卡死通道还挂着这个池，
        // 于是 kd_mpi_vb_destory_pool() 把主线程堵死（状态行一行都没有）。
        // 有残留通道就什么都不拆，交给内核、下次上电自清。
        if (probe_left_ > 0) {
            skip_teardown_ = true;
            log_line("录像: 自检留有 %u 个卡死通道 → **不销毁任何池**（销毁会把主线程堵死）\n", probe_left_);
        } else {
            kd_mpi_vb_destory_pool(static_cast<k_u32>(venc_pool_id_));
        }
        venc_pool_id_ = -1;
        return false;
    }
    log_line("录像: VENC chn%u %ux%u @%ufps %s intbuf=%uMB\n", chn_, cfg_.width, enc_h_, cfg_.sensor_fps,
             adopt_mjpeg_ ? "MJPEG(fixqp q=80)" : "H264/HIGH/CBR", cfg_.venc_intbuf_mb);

    // ④ 文件 + 参数集：start_chn 之后参数集只出现一次，这里必须抓（抓不到就在取流路径里补）
    if (!open_file()) {
        teardown_venc();
        return false;
    }
    const uint32_t got = drain_once(200);
    log_line("录像: start_chn 首批码流 %lu 字节（视频包 %u，参数集 %u 字节）\n",
             static_cast<unsigned long>(bytes()), got, hdr_len_);

    last_status_ms_ = mono_ms();
    last_flush_ms_ = mono_ms();
    running_ = true;
    thread_ = std::thread([this] { run(); });
    log_line("录像: 启动完成 %llums（chn%u chunk %u 字节 x %u）\n",
             static_cast<unsigned long long>(mono_ms() - t0), chn_, in_blk_bytes_, in_blk_cnt_);
    return true;
}

// 建一个 VENC 通道（attach 输出池 → create → start）。失败返回 false 并清理干净。
bool Recorder::setup_chn(uint32_t chn, k_payload_type type, uint32_t enc_h, uint32_t fps, uint32_t gop,
                         uint32_t kbps, uint32_t q_factor, uint32_t intbuf_mb) {
    if (kd_mpi_venc_attach_vb_pool(chn, static_cast<k_u32>(venc_pool_id_)) != K_SUCCESS) {
        log_line("录像: chn%u attach_vb_pool 失败\n", chn);
        return false;
    }

    k_venc_chn_attr attr{};
    attr.venc_attr.type = type;
    attr.venc_attr.pic_width = cfg_.width;
    attr.venc_attr.pic_height = enc_h;
    if (type == K_PT_H264) {
        attr.venc_attr.profile = VENC_PROFILE_H264_HIGH; // 与老工程（板端验证过）一致
        attr.rc_attr.rc_mode = K_VENC_RC_MODE_CBR;
        attr.rc_attr.cbr.src_frame_rate = fps;
        attr.rc_attr.cbr.dst_frame_rate = fps;
        attr.rc_attr.cbr.bit_rate = kbps;
        attr.rc_attr.cbr.gop = gop;
    } else {
        attr.rc_attr.rc_mode = K_VENC_RC_MODE_MJPEG_FIXQP;
        attr.rc_attr.mjpeg_fixqp.src_frame_rate = fps ? fps : 30;
        attr.rc_attr.mjpeg_fixqp.dst_frame_rate = fps ? fps : 30;
        attr.rc_attr.mjpeg_fixqp.q_factor = q_factor ? q_factor : 80;
    }

    k_s32 rc = kd_mpi_venc_create_chn(chn, &attr);
    if (rc != K_SUCCESS) {
        log_line("录像: chn%u create_chn 失败 ret=0x%x（被占？参数被拒？）\n", chn,
                 static_cast<unsigned>(rc));
        kd_mpi_venc_detach_vb_pool(chn);
        return false;
    }
    // VPU 固件的工作区（intbuf）：文档原话 "configure internal memory used for vpu firmware"，
    // 官方 sample 里是可选参数。内核字符串里有 "No intbuf space for master save area" /
    // "Failed to set internal buffer size" —— 编码器"收帧不吐码流"很可能就是它没配。
    if (intbuf_mb > 0) {
        const k_s32 rb = kd_mpi_venc_set_intbuf_size(chn, intbuf_mb << 20);
        log_line("录像: chn%u set_intbuf_size(%u MB) ret=0x%x\n", chn, intbuf_mb,
                 static_cast<unsigned>(rb));
    }

    rc = kd_mpi_venc_start_chn(chn);
    if (rc != K_SUCCESS) {
        log_line("录像: chn%u start_chn 失败 rc=0x%x\n", chn, static_cast<unsigned>(rc));
        kd_mpi_venc_destroy_chn(chn);
        kd_mpi_venc_detach_vb_pool(chn);
        return false;
    }
    return true;
}

void Recorder::teardown_venc() {
    if (venc_pool_id_ < 0)
        return;
    // 每一步都留痕：这四步里任何一步都可能把内核带走（板端实测给卡死的编码器送过帧
    // 之后 stop/destroy 会让整机重启），日志的最后一行就是凶手。
    log_line("拆链: stop_chn(%u) 前\n", chn_);
    kd_mpi_venc_stop_chn(chn_);
    log_line("拆链: stop_chn 返回，destroy_chn 前\n");
    kd_mpi_venc_destroy_chn(chn_);
    log_line("拆链: destroy_chn 返回，detach_vb_pool 前\n");
    kd_mpi_venc_detach_vb_pool(chn_);
    log_line("拆链: detach 返回，销毁输出池前\n");
    kd_mpi_vb_destory_pool(static_cast<k_u32>(venc_pool_id_));
    log_line("拆链: 输出池已销毁\n");
    venc_pool_id_ = -1;
}

void Recorder::stop() {
    if (stopped_)
        return;
    stopped_ = true;

    if (running_) { // 叫停录像线程并等它把队列里剩的帧做完
        {
            std::lock_guard<std::mutex> lk(q_mtx_);
            running_ = false;
        }
        q_cv_.notify_all();
        if (thread_.joinable())
            thread_.join();
    }

    maybe_flush(true);
    if (file_) {
        std::fclose(file_);
        file_ = nullptr;
    }

    // 安全阀：给编码器送过帧、却一个字节码流都没出（编码器是死的）时**不拆链**——
    // 那种状态下 stop/destroy/detach 会把整机重启（板端实测 4 次重启都打在这里）。
    // 进程退出后通道/池由内核留着，下次上电自然清掉；诊断轮次里这比"重启板子"划算得多。
    if (frames_.load(std::memory_order_relaxed) > 0 && bytes_.load(std::memory_order_relaxed) == 0) {
        skip_teardown_ = true;
        log_line("录像: 送过 %lu 帧但一字节码流都没出 → **退出时不拆链**（保留 VENC 通道与 VB 池）\n",
                 static_cast<unsigned long>(frames_.load(std::memory_order_relaxed)));
    }

    if (skip_teardown_) {
        return;
    }
    teardown_venc(); // stop → destroy_chn → detach_vb_pool → 销毁输出池

    // 输入块：应用持有的引用必须还掉（上一版从没还过），池才销毁得干净
    for (uint32_t i = 0; i < in_blk_cnt_; ++i) {
        if (in_[i].h != VB_INVALID_HANDLE) {
            kd_mpi_vb_release_block(in_[i].h);
            in_[i].h = VB_INVALID_HANDLE;
        }
    }
    if (in_pool_id_ >= 0) {
        kd_mpi_vb_destory_pool(static_cast<k_u32>(in_pool_id_));
        in_pool_id_ = -1;
    }
    free_n_ = 0;
    inflight_n_ = 0;
}

// 接上一帧：只入队。识别链路的节拍绝不因为编码器或 SD 卡而等待 ——
// 队列满就丢这一帧（老工程 record_take_frame() 同款语义）。
void Recorder::submit(Frame *f) {
    // 识别优先：池里空闲帧不够了就直接不录，绝不和识别抢帧
    if (pool_.free_count() <= reserve_frames_) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        pool_.release(f);
        return;
    }

    std::lock_guard<std::mutex> lk(q_mtx_);
    if (!running_ || q_count_ == kQueueDepth) {
        dropped_.fetch_add(1, std::memory_order_relaxed);
        pool_.release(f);
        return;
    }

    q_[(q_head_ + q_count_) % kQueueDepth] = f;
    ++q_count_;
    q_cv_.notify_one();
}

// 建输入块池：一次取满 in_blk_cnt_ 块，全部填好帧描述并进空闲栈。
// 非 cache 持久映射：CPU 写完硬件立刻可见，不需要任何 flush。
bool Recorder::prepare_in_blocks() {
    in_pool_id_ = kd_mpi_vb_create_pool_ex(in_blk_bytes_, in_blk_cnt_, VB_REMAP_MODE_NOCACHE);
    if (in_pool_id_ == static_cast<k_s32>(VB_INVALID_POOLID)) {
        log_line("录像: 创建输入块池失败（%u x %u 字节），本次不录像\n", in_blk_cnt_, in_blk_bytes_);
        return false;
    }

    for (uint32_t i = 0; i < in_blk_cnt_; ++i) {
        InBlk &b = in_[i];
        b.h = kd_mpi_vb_get_block(static_cast<k_u32>(in_pool_id_), in_blk_bytes_, nullptr);
        if (b.h == VB_INVALID_HANDLE) {
            log_line("录像: 输入块 %u 申请失败（池里没块了），本次不录像\n", i);
            return false;
        }
        b.phys = kd_mpi_vb_handle_to_phyaddr(b.h);
        b.va = static_cast<uint8_t *>(mpp_map_persist(b.phys, in_blk_bytes_));
        if (b.va == nullptr) {
            log_line("录像: 输入块 %u 映射失败，本次不录像\n", i);
            return false;
        }

        // UV 平面恒 128：编码器看到的就是黑白画面。只填这一次 —— 每帧再 memset 115KB
        // 非 cache 内存纯属浪费。
        std::memset(b.va + y_bytes_, 128, in_blk_bytes_ - y_bytes_); // UV 恒 128，只填一次

        // 帧描述填法照抄老工程 record.c 的 blk_setup_frame()（板端验证过）：
        //   * 不填 virt_addr[]：塞用户空间指针进去会让驱动拿到非法地址
        //   * NV12 只填 stride[0]，stride[1] 留 0
        //   * mod_id 照抄源帧（VENC 收 VICAP 帧时期望的值），每帧在 send 侧刷新
        k_video_frame &vf = b.info.v_frame;
        vf.width = cfg_.width;
        vf.height = enc_h_;
        vf.pixel_format = PIXEL_FORMAT_YUV_SEMIPLANAR_420;
        vf.stride[0] = cfg_.width;
        vf.phys_addr[0] = b.phys;
        vf.phys_addr[1] = b.phys + static_cast<k_u64>(cfg_.width) * enc_h_;
        b.info.pool_id = static_cast<k_u32>(in_pool_id_);
        b.info.mod_id = pool_.mod_id();

        free_[free_n_++] = static_cast<int16_t>(i);
    }

    log_line("录像: 输入块 %u x %u 字节（启动时一次取满，运行期不再 get_block）\n", in_blk_cnt_,
             in_blk_bytes_);
    return true;
}

// 录像线程：拷进空闲输入块 → 有界送帧 → 取流回收块 → 落盘。阻塞只发生在这里。
void Recorder::run() {
    for (;;) {
        Frame *f = nullptr;
        {
            std::unique_lock<std::mutex> lk(q_mtx_);
            q_cv_.wait(lk, [this] { return q_count_ > 0 || !running_; });
            if (q_count_ == 0 && !running_)
                break; // 队列空且已叫停 → 收工
            f = q_[q_head_];
            q_head_ = (q_head_ + 1) % kQueueDepth;
            --q_count_;
        }

        const uint64_t seq = f->seq;
        if (copy_and_send(f))
            frames_.fetch_add(1, std::memory_order_relaxed);
        pool_.release(f); // 池帧立刻归还：录像攥着它的时间越短，识别可用的帧越多

        // 只对前几次打点：用"尝试次数"而不是计数器之和 —— 块用光时两个计数器都不涨，
        // 用它们做条件会变成每帧一行 + 每行 fsync，把 SD 卡写穿（看着像卡死）。
        if (++attempts_ <= 3)
            log_line("录像: 帧 %lu 送帧 ret=%d（成功 %lu 失败 %lu 无块 %lu）\n",
                     static_cast<unsigned long>(seq), last_ret_,
                     static_cast<unsigned long>(frames_.load(std::memory_order_relaxed)),
                     static_cast<unsigned long>(send_fail_.load(std::memory_order_relaxed)),
                     static_cast<unsigned long>(blk_empty_.load(std::memory_order_relaxed)));

        // 取流：送一帧出一包，取到 n 个视频包就回收 n 个在途块（块与包严格 1:1）。
        // 第一次可能有界等待，之后连取几次都非阻塞：编码器攒了多帧时一次收干净，
        // 免得下面的超时判定把"其实已经产出、只是还没取"的块误判成丢帧。
        if (inflight_n_ > 0) {
            for (uint32_t i = 0; i < kBurstMax; ++i) {
                const uint32_t n = drain_once(i == 0 ? kGetToMs : 0);
                if (n == 0)
                    break;
                if (n > inflight_n_)
                    underflow_.fetch_add(n - inflight_n_, std::memory_order_relaxed);
                reclaim(n);
            }
        }

        // 兜底：编码器**静默丢掉**某一帧时（它不吐码流，应用无从得知），那块永远回不来，
        // 攒够 6 块录像就永久停摆。按"单块在途时长"判超时，超时就强收最老的那些块
        // （宁可坏一帧，也不让录像停）。每次强收都留一行日志。
        // 一直没有任何码流时，定期问一次编码器内部状态：cur_packs / PicCnt / end_of_stream
        // 能把"编码器根本没在编"和"编了但我们取不到"这两件事分开。
        if (bytes_.load(std::memory_order_relaxed) == 0 && mono_ms() - last_status_ms_ >= kVencStatusMs) {
            last_status_ms_ = mono_ms();
            log_venc_status("无码流");
            // 同一时刻量一次 MMZ：编码器的内部缓冲也从这里出，余量为 0 就是被饿死了
            log_line("录像: MMZ 余量(卡住时) %u MB（单块最大 %u MB）\n", mmz_free_mb(), mmz_max_block_mb());
        }

        if (inflight_n_ > 0 && mono_ms() - inflight_ms_[0] >= kInflightMaxMs) {
            uint32_t n = 0;
            const uint64_t now = mono_ms();
            while (inflight_n_ > 0 && now - inflight_ms_[0] >= kInflightMaxMs) {
                reclaim(1);
                ++n;
            }
            reclaimed_.fetch_add(n, std::memory_order_relaxed);
            log_line("录像: 在途块超时 %llums 未回码流 → 强收 %u 块（编码器丢帧？在途剩 %u）\n",
                     static_cast<unsigned long long>(kInflightMaxMs), n, inflight_n_);
        }

        maybe_flush(false);
    }

    // 收尾：把编码器里剩的码流取干净，否则文件尾丢帧
    for (uint32_t i = 0; i < 64 && inflight_n_ > 0; ++i) {
        const uint32_t n = drain_once(kGetEndToMs);
        if (n == 0)
            break;
        reclaim(n);
    }
    if (inflight_n_ > 0)
        reclaim(inflight_n_); // 收尾强制清零，保证 stop() 时块账目干净
    maybe_flush(true);

    log_line("录像: 线程收工（送 %lu 帧，失败 %lu，无块 %lu，丢队列 %lu，强收 %lu，%.2f MB）\n",
             static_cast<unsigned long>(frames_.load(std::memory_order_relaxed)),
             static_cast<unsigned long>(send_fail_.load(std::memory_order_relaxed)),
             static_cast<unsigned long>(blk_empty_.load(std::memory_order_relaxed)),
             static_cast<unsigned long>(dropped_.load(std::memory_order_relaxed)),
             static_cast<unsigned long>(reclaimed_.load(std::memory_order_relaxed)),
             static_cast<double>(bytes_.load(std::memory_order_relaxed)) / (1024 * 1024));
}

// 把这一帧拷进空闲输入块再送编码器。成功即进在途队列 —— 块要等它的码流回来才允许复用。
bool Recorder::copy_and_send(Frame *src) {
    if (free_n_ == 0) { // 6 块全在途：这一帧不录（编码器还没吐码流）
        blk_empty_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    const int16_t bi = free_[--free_n_];
    InBlk &b = in_[bi];

    // 只拷要编码的那 enc_h_ 行（探针可能把编码高度降到 16 对齐的值），紧凑排列一块拷完
    std::memcpy(b.va, src->view.pixels, static_cast<size_t>(cfg_.width) * enc_h_);
    annotate(b.va, src);

    const k_mod_id mid = pool_.mod_id(); // 第一帧之后才拿到源帧的值，幂等刷新
    if (b.info.mod_id != mid)
        b.info.mod_id = mid;

    last_ret_ = kd_mpi_venc_send_frame(chn_, &b.info, kSendToMs);
    if (last_ret_ != K_SUCCESS) {
        const uint64_t nf = send_fail_.fetch_add(1, std::memory_order_relaxed) + 1;
        if (nf <= 5 || (nf % 100) == 0)
            log_line("录像: send_frame 失败 ret=0x%x（第 %lu 次，帧 %lu）\n",
                     static_cast<unsigned>(last_ret_), static_cast<unsigned long>(nf),
                     static_cast<unsigned long>(src->seq));
        free_[free_n_++] = bi; // 没送出去：块立刻收回
        return false;
    }

    inflight_[inflight_n_] = bi;
    inflight_ms_[inflight_n_] = mono_ms();
    ++inflight_n_;
    return true;
}

// 造一帧合成图像：横向渐变 + 移动竖条（编码器需要有内容变化才看得出在编）
void Recorder::fill_synth(uint32_t bi, uint32_t frame_no) {
    uint8_t *p = in_[bi].va;
    const uint32_t w = cfg_.width;
    for (uint32_t y = 0; y < enc_h_; ++y) {
        uint8_t *row = p + static_cast<size_t>(y) * w;
        for (uint32_t x = 0; x < w; ++x)
            row[x] = static_cast<uint8_t>((x + y + frame_no * 7u) & 0xffu);
    }
    const uint32_t bar = (frame_no * 37u) % (w > 64 ? w - 64 : 1);
    for (uint32_t y = 0; y < enc_h_; ++y) {
        uint8_t *row = p + static_cast<size_t>(y) * w + bar;
        for (uint32_t x = 0; x < 64 && bar + x < w; ++x)
            row[x] = 255;
    }
}

// 编码器自检：候选配置逐个建通道试编，每个都打 query_status，看谁能真的吐出码流。
// 板端只看日志——这一轮的结论完全靠它。
bool Recorder::probe_encoder() {
    struct Cand {
        const char *tag;
        k_payload_type type;
        uint32_t enc_h;
        uint32_t fps;
        uint32_t gop;
        uint32_t kbps;
        uint32_t qf;
        uint32_t intbuf_mb;
    };
    const uint32_t h16 = cfg_.height & ~15u; // 360 -> 352（16 的倍数）
    struct Cand all[] = {
        {"H264+intbuf16", K_PT_H264, cfg_.height, cfg_.sensor_fps, cfg_.gop, cfg_.bitrate_kbps, 0, 16},
        {"H264+intbuf4", K_PT_H264, cfg_.height, cfg_.sensor_fps, cfg_.gop, cfg_.bitrate_kbps, 0, 4},
        {"H264+intbuf16+高352", K_PT_H264, h16 ? h16 : cfg_.height, cfg_.sensor_fps, cfg_.gop,
         cfg_.bitrate_kbps, 0, 16},
        {"MJPEG+intbuf16", K_PT_JPEG, cfg_.height, 30, 0, 0, 80, 16},
    };
    // 这块固件上 VENC **只允许一个通道存在**（第二个 create_chn 一律 K_ERR_NOBUF=13），
    // 所以"一轮试多个候选"实际上只有第一个真的被试到。用 --venc-mode 指定这一轮试哪一档：
    //   m = 只试 MJPEG（同一条手工送帧路径、不同编码器：用来判断手工路径本身是否可用）
    //   h = 只试 H264（无 intbuf）
    //   i = 只试 H264+intbuf16
    //   0 = 按上面的顺序全试（第一个失败后剩余的会因 NOBUF 建不起来，属预期）
    std::vector<Cand> cands;
    if (cfg_.venc_mode == 'm')
        cands.push_back(all[3]);
    else if (cfg_.venc_mode == 'h')
        cands.push_back({"H264", K_PT_H264, cfg_.height, cfg_.sensor_fps, cfg_.gop, cfg_.bitrate_kbps, 0, 0});
    else if (cfg_.venc_mode == 'i')
        cands.push_back(all[0]);
    else
        for (const Cand &c : all)
            cands.push_back(c);

    log_line("录像: ---- 编码器自检开始（%u 个候选，各自独占一个通道；mode=%c）----\n",
             static_cast<unsigned>(cands.size()), cfg_.venc_mode ? cfg_.venc_mode : '0');

    for (uint32_t ci = 0; ci < cands.size(); ++ci) {
        const Cand &c = cands[ci];
        const uint32_t w = cfg_.width;
        const uint32_t chn = (cfg_.venc_chn + ci) % 4;
        if (!setup_chn(chn, c.type, c.enc_h, c.fps, c.gop, c.kbps, c.qf, c.intbuf_mb)) {
            log_line("录像: 自检[%s] chn%u 建通道失败\n", c.tag, chn);
            continue; // 没建起来 = 通道干净，可以继续用别的通道
        }

        enc_h_ = c.enc_h;
        const uint32_t nsend = in_blk_cnt_ < 2 ? in_blk_cnt_ : 2; // 少送几帧，少留残留
        uint32_t sent = 0;
        for (uint32_t i = 0; i < nsend; ++i) {
            fill_synth(i, i);
            k_video_frame_info vi = in_[i].info;
            vi.v_frame.height = c.enc_h;
            vi.v_frame.phys_addr[1] = vi.v_frame.phys_addr[0] + static_cast<k_u64>(w) * c.enc_h;
            const k_s32 rc = kd_mpi_venc_send_frame(chn, &vi, 500);
            if (rc != K_SUCCESS) {
                log_line("录像: 自检[%s] chn%u 第 %u 帧送帧失败 ret=0x%x\n", c.tag, chn, i + 1,
                         static_cast<unsigned>(rc));
                break;
            }
            ++sent;
        }

        uint32_t got = 0, got_bytes = 0;
        k_venc_chn_status st{};
        for (uint32_t i = 0; i < 20; ++i) {
            k_venc_stream stream{};
            k_venc_pack pk[kMaxPacks];
            stream.pack = pk;
            stream.pack_cnt = kMaxPacks;
            if (kd_mpi_venc_get_stream(chn, &stream, 100) == K_SUCCESS) {
                ++got;
                for (uint32_t j = 0; j < stream.pack_cnt && j < kMaxPacks; ++j)
                    got_bytes += stream.pack[j].len;
                kd_mpi_venc_release_stream(chn, &stream);
                break;
            }
            if (i == 10)
                kd_mpi_venc_query_status(chn, &st);
        }
        if (got == 0)
            kd_mpi_venc_query_status(chn, &st);

        log_line("录像: 自检[%s] chn%u %ux%u@%ufps intbuf=%uMB 送 %u 帧 → 码流批次 %u 字节 %u；"
                 "query: cur_packs=%u PicCnt=%u PicBytes=%u eos=%d\n",
                 c.tag, chn, w, c.enc_h, c.fps, c.intbuf_mb, sent, got, got_bytes,
                 static_cast<unsigned>(st.cur_packs), static_cast<unsigned>(st.stream_info.u32PicCnt),
                 static_cast<unsigned>(st.stream_info.u32PicBytesNum), static_cast<int>(st.end_of_stream));

        if (got_bytes > 0) {
            chn_ = chn;
            if (c.type == K_PT_JPEG)
                adopt_mjpeg_ = true;
            log_line("录像: 自检结论 → 采用 [%s]（chn%u，高度 %u）\n", c.tag, chn_, enc_h_);
            chn_ready_ = true; // 这个通道已经建好并 start 过了，主链路直接用它
            return true;
        }
        ++probe_left_;
        log_line("录像: 自检[%s] 无码流 → **不拆这个通道**（拆链会重启整机），换下一个候选\n", c.tag);
        // 注意：故意不调用 stop/destroy/detach —— 实测这条路径会把内核带走
    }

    log_line("录像: 编码器自检结束：**没有任何候选吐出过码流**（编码器子系统/固件层面的问题）\n");
    return false;
}

// 问一次编码器内部状态。板端只看日志，这一行是区分下面两种情形的唯一手段：
//   * cur_packs=0 且 PicCnt=0 → 编码器**根本没在编**（内部资源/固件/帧描述问题）
//   * cur_packs>0 → 编出来了，是我们的取流/回收逻辑没拿到
void Recorder::log_venc_status(const char *why) {
    k_venc_chn_status st{};
    const k_s32 rc = kd_mpi_venc_query_status(chn_, &st);
    if (rc != K_SUCCESS) {
        log_line("录像: query_status(%s) 失败 ret=0x%x\n", why, static_cast<unsigned>(rc));
        return;
    }
    log_line("录像: 编码器状态(%s) cur_packs=%u end_of_stream=%d PicCnt=%u PicBytes=%u StartQp=%u MeanQp=%u"
             "（已送出 %lu 帧，在途 %u，收到码流 %lu 字节）\n",
             why, static_cast<unsigned>(st.cur_packs), static_cast<int>(st.end_of_stream),
             static_cast<unsigned>(st.stream_info.u32PicCnt),
             static_cast<unsigned>(st.stream_info.u32PicBytesNum),
             static_cast<unsigned>(st.stream_info.u32StartQp),
             static_cast<unsigned>(st.stream_info.u32MeanQp),
             static_cast<unsigned long>(frames_.load(std::memory_order_relaxed)), inflight_n_,
             static_cast<unsigned long>(bytes_.load(std::memory_order_relaxed)));
}

// 取一次码流并落盘。**故意不用 query_status 预检/限长** —— 老工程 record.c 的原话：
// SDK 文档说 get_stream 的包可能是 frame 也可能是 header，而 header 可能不计进
// cur_packs，按 cur_packs 申请长度就会把参数集截掉。固定申请 kMaxPacks，长度以返回值为准。
uint32_t Recorder::drain_once(int32_t timeout_ms) {
    k_venc_stream stream{};
    stream.pack_cnt = kMaxPacks;
    stream.pack = g_packs;

    if (kd_mpi_venc_get_stream(chn_, &stream, timeout_ms) != K_SUCCESS)
        return 0;

    const uint32_t nvideo = write_packs(stream);
    kd_mpi_venc_release_stream(chn_, &stream);
    return nvideo;
}

// 码流包 → 文件。返回视频包数（参数集包不计）。
uint32_t Recorder::write_packs(const k_venc_stream &stream) {
    uint32_t nvideo = 0;

    for (uint32_t i = 0; i < stream.pack_cnt && i < kMaxPacks; ++i) {
        const k_venc_pack &pack = stream.pack[i];
        if (pack.len == 0)
            continue;

        const uint8_t *data = map_pack(pack);
        if (data == nullptr)
            continue;

        if (pack.type == K_VENC_HEADER && !adopt_mjpeg_) {
            // SPS/PPS 只在码流开始时出现一次：缓存下来，每个 I 帧前重发
            // （老工程同款；单文件从中间截断/断电也能解）
            if (pack.len <= sizeof(hdr_)) {
                std::memcpy(hdr_, data, pack.len);
                hdr_len_ = pack.len;
            }
            write_bytes(data, pack.len);
            continue;
        }

        if (pack.type == K_VENC_I_FRAME && hdr_len_ > 0 && !adopt_mjpeg_)
            write_bytes(hdr_, hdr_len_);
        write_bytes(data, pack.len);
        ++nvideo;

        if (bytes_.load(std::memory_order_relaxed) == pack.len) // 第一个视频包：编码器真的在出货
            log_line("录像: 首个%s包 %u 字节（type=%d）\n", pack.type == K_VENC_I_FRAME ? " I" : "P",
                     static_cast<unsigned>(pack.len), static_cast<int>(pack.type));
    }
    return nvideo;
}

// 码流包躺在输出池的某个块里。归一化到块基址 → **按块大小**取持久 cache 映射 →
// 读之前 invalidate。同一物理块每包长度都不同，逐包 map/unmap 正是老工程
// "内核被打挂" 的那条路，这里一次系统调用都不做。
const uint8_t *Recorder::map_pack(const k_venc_pack &pack) {
    const k_vb_blk_handle h = kd_mpi_vb_phyaddr_to_handle(pack.phys_addr);
    if (h != VB_INVALID_HANDLE) {
        const k_u64 base = kd_mpi_vb_handle_to_phyaddr(h);
        if (base != 0 && base <= pack.phys_addr &&
            (pack.phys_addr - base) + pack.len <= out_blk_bytes_) {
            uint8_t *va = static_cast<uint8_t *>(mpp_map_persist_cached(base, out_blk_bytes_));
            if (va == nullptr)
                return nullptr;
            mpp_invalidate(base, va, out_blk_bytes_);
            return va + (pack.phys_addr - base);
        }
        if (!warned_pack_addr_) { // 归一化失败：地址不在已知块内，退回按原址+长度映射
            warned_pack_addr_ = true;
            log_line("录像: 码流包不在输出池块内 phys=0x%llx len=%u（退回直接映射）\n",
                     static_cast<unsigned long long>(pack.phys_addr), static_cast<unsigned>(pack.len));
        }
    }
    return static_cast<const uint8_t *>(mpp_map_persist(pack.phys_addr, pack.len));
}

void Recorder::write_bytes(const uint8_t *data, uint32_t len) {
    if (file_ == nullptr || len == 0)
        return;
    if (std::fwrite(data, 1, len, file_) != len) {
        io_fail_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    bytes_.fetch_add(len, std::memory_order_relaxed);
}

// 在途块 → 空闲块。必须 FIFO：送进去的帧与出来的包严格同序，收回的必须是最早那块。
void Recorder::reclaim(uint32_t n) {
    while (n-- > 0 && inflight_n_ > 0 && free_n_ < in_blk_cnt_) {
        free_[free_n_++] = inflight_[0];
        for (uint32_t i = 1; i < inflight_n_; ++i) {
            inflight_[i - 1] = inflight_[i];
            inflight_ms_[i - 1] = inflight_ms_[i];
        }
        --inflight_n_;
    }
}

// 每 kFlushMs 落一次盘：录像文件是全缓冲的，不 flush 断电就全丢；fsync 保证内核页
// 缓存也落盘（老工程同款 1 秒 1 次，避免每帧一次小写把 SD 卡写爆）。
void Recorder::maybe_flush(bool force) {
    if (file_ == nullptr)
        return;
    const uint64_t now = mono_ms();
    if (!force && now - last_flush_ms_ < kFlushMs)
        return;
    last_flush_ms_ = now;
    if (std::fflush(file_) != 0) {
        io_fail_.fetch_add(1, std::memory_order_relaxed);
        return;
    }
    fsync(fileno(file_));
}

// 左上角半行关键数据：帧号 / 时刻(ms) / 目标中心 / ROI 状态 / 识别耗时(us)
void Recorder::annotate(uint8_t *dst, const Frame *f) {
    char line[kLabelMaxChars + 8];
    std::snprintf(line, sizeof(line), "F%06lu T%09lu X%04d Y%04d R%u D%05u",
                  static_cast<unsigned long>(f->seq), static_cast<unsigned long>(f->mono_ms), f->result.cx,
                  f->result.cy, static_cast<unsigned>(f->result.roi), f->result.cost_us % 100000u);
    draw_label(dst, f->view.stride, line);
}

bool Recorder::open_file() {
    mkdir(cfg_.out_dir, 0777); // 新卡上没有这个目录；已存在时返回 EEXIST，无需处理

    char path[160];
    for (uint32_t i = 1; i <= 9999; ++i) {
        std::snprintf(path, sizeof(path), "%s/rec_%04u.%s", cfg_.out_dir, i,
                      adopt_mjpeg_ ? "mjpg" : "h264");
        if (access(path, F_OK) != 0)
            break; // 文件名已被占用就顺延
    }

    file_ = std::fopen(path, "wb");
    if (file_ == nullptr) {
        // 只读/没挂载的 /sdcard 只让"录像"这一路失效，绝不能因此杀掉整个程序
        log_line("录像: 打不开 %s（/sdcard 挂载了吗、是不是只读），本次不录像\n", path);
        return false;
    }
    std::setvbuf(file_, g_file_buf, _IOFBF, sizeof(g_file_buf));
    log_line("录像文件: %s\n", path);
    return true;
}

} // namespace dart
