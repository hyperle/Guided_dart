// ============================================================================
// main —— 全流程编排（唯一知道"顺序"的地方）：
//   ① 从传感器取帧          SensorSource::capture
//   ② 二值化进池帧          GraphicsUtils::binarize（RVV 向量）
//   ③ 识别当前帧            IDetector::detect（接口，实现由使用方提供）
//   ④ 上一帧交给录像        Recorder::submit（烧入左上角关键数据 → VENC → 归还池）
//   ⑤ 识别器转到下一帧
//
// 每段的实现都在各自类里；本文件只有循环和统计。
// ============================================================================
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <memory>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include "capture/sensor_source.hpp"
#include "core/config.hpp"
#include "core/log.hpp"
#include "core/mmz.hpp"
#include "core/mpp_map.hpp"
#include "detection/config.hpp"
#include "detection/graphics_utils.hpp"
#include "detection/pipeline.hpp"
#include "evidence/evidence_writer.hpp"
#include "record/recorder.hpp"
#include "vision/detector.hpp"
#include "vision/frame_pool.hpp"

namespace {

using dart::log_fault;
using dart::log_line;
using dart::log_path;
using dart::log_reopen;

volatile std::sig_atomic_t g_run = 1;
pid_t                      g_watchdog = -1;
int                        g_lock_fd = -1;
bool                       g_det_on = true; // --det off → 用 DetectorStub（开销基线）

void on_stop(int) { // SIGINT/SIGALRM 都走这里：让主循环干净退出，析构才会 deinit VICAP/VB
    g_run = 0;
}

void kill_watchdog() {
    if (g_watchdog > 0) {
        kill(g_watchdog, SIGKILL);
        g_watchdog = -1;
    }
}

// 崩溃也留痕：板端没有 shell，"进程静默消失"是最难查的失败。
// 处理器里只用 write()（log_fault），退出码 2 与正常退出 0 区分开。
void on_fault(int sig) {
    // 上一版这里把指针算错，单位数信号打印成 "04\n"（前缀丢了、还多个 0）——
    // 崩溃留痕本身坏掉，等于把最贵的证据扔了。手工拼，两位右对齐。
    char buf[24];
    const char *prefix = "FAULT: signal ";
    for (int i = 0; i < 14; ++i)
        buf[i] = prefix[i];
    const int s = sig < 0 ? 0 : (sig % 100);
    buf[14] = (s < 10) ? ' ' : static_cast<char>('0' + s / 10);
    buf[15] = static_cast<char>('0' + s % 10);
    buf[16] = '\n';
    log_fault(buf, 17);

    kill_watchdog(); // 否则它会一直打心跳，把"进程死了"伪装成"板子活着"
    _exit(2);
}

// 看门狗子进程：每 2 秒一行心跳。用来区分两种失败——
//   * 只有本进程卡住：心跳继续
//   * 整机冻住（内核级）：心跳和主进程一起停
// 父进程一死（含被 SIGKILL）它必须自己退：否则下次读日志会看到"心跳还在、
// 程序早没了"，正好把结论判反。
pid_t spawn_watchdog() {
    const pid_t parent = getpid();
    const pid_t pid = fork();
    if (pid != 0) {
        g_watchdog = pid;
        std::atexit(kill_watchdog); // 正常退出与 die() 都会走到
        return pid;
    }

    log_reopen(); // 子进程用自己的 O_APPEND fd，别和父进程共享文件偏移
    for (;;) {
        sleep(2);
        // 父进程没了就自杀。两个判据都要保守：RT-Smart 上 kill(pid,0) 可能直接返回
        // 不支持的错误（上一版就因此秒退，心跳只剩一行），只有 ESRCH 才算"真没了"。
        const pid_t pp = getppid();
        const bool pp_gone = (pp > 1 && pp != parent);
        bool kill_gone = false;
        if (kill(parent, 0) != 0 && errno == ESRCH)
            kill_gone = true;
        if (pp_gone || kill_gone)
            _exit(0);
        log_line("看门狗: 心跳\n");
    }
}

// 单实例守卫：VENC / VB / VICAP 都是独占资源。历史上"多个实例同时上电"把 VB 占死、
// 后面每个都撞 FATAL，日志里只剩一堆互相矛盾的失败。用 fcntl 写锁 —— 进程一死锁
// 自动释放，不会留陈旧锁；且**失败即放行**（只有明确的 EACCES/EAGAIN 才认定
// "已有实例"，fcntl 不被支持时照常启动）。
void single_instance_guard() {
    const char *path = "/sdcard/app/self_guiding_dart.lock";
    const int fd = open(path, O_RDWR | O_CREAT, 0666);
    if (fd < 0) {
        log_line("实例守卫: 打不开 %s，跳过（本地日志路径回退会兜住）\n", path);
        return;
    }

    struct flock fl {};
    fl.l_type = F_WRLCK;
    fl.l_whence = SEEK_SET;
    if (fcntl(fd, F_SETLK, &fl) != 0) {
        const int e = errno;
        if (e == EACCES || e == EAGAIN) {
            log_line("FATAL: 已有实例在跑（%s 被写锁占住），两个实例会抢 VENC/VB/VICAP\n", path);
            std::exit(1);
        }
        close(fd);
        log_line("实例守卫: fcntl 不支持(rc=%d errno=%d)，跳过\n", -1, e);
        return;
    }
    g_lock_fd = fd; // 保持打开：进程结束内核自动释放，不需要 unlink
}

uint64_t mono_ms(); // 定义在下面（取证通道要先用到）

// ---- 不依赖编码器的取证通道：PBM 图 + 原始 PGM + 每帧 CSV ----
// 为什么要它：VENC 那一路在这块板子/固件上不可用（手工送帧两条编码路径都拿不到输出，
// PicCnt 恒为 0），但"二值化出来的图长什么样 / 每帧识别结果"才是标定识别器真正需要的。
// 关键设计：**文件 I/O 全在 EvidenceWriter 的写线程里**（板端实测在视觉线程里存一张
// 28.8KB 的 PBM 会把循环从 90fps 拖到 45fps），这里只做打包入队。
dart::EvidenceWriter *g_ev = nullptr;
uint64_t              g_last_pts = 0;
uint64_t              g_ev_busy_prev = 0; // 上一秒写线程忙时（算增量）
uint64_t              g_pbm_us_max = 0;  // 单次"打包+入队"最坏耗时（us）
uint64_t              g_pbm_us_sum = 0;
uint32_t              g_pbm_us_cnt = 0;
uint32_t              g_exp_us = 0; // 当前曝光（固定值，来自 config.hpp，写进 CSV 备查）

void emit_csv(const dart::Frame *f, double fps, int pbm_saved) {
    if (g_ev == nullptr)
        return;
    // d_pts_us：源帧时间戳的间隔（微秒）。它才是"传感器真实帧间隔"，用它算出的帧率
    // 才是真帧率 —— 自己数循环次数只能证明"我消费了多少帧"。
    const uint64_t d = (g_last_pts != 0 && f->src_pts > g_last_pts) ? (f->src_pts - g_last_pts) : 0;
    g_last_pts = f->src_pts;
    char line[320];
    const int n = std::snprintf(line, sizeof(line),
                               "%lu,%lu,%llu,%llu,%u,%d,%d,%u,%u,%.1f,%d,%.1f,%.3f,%u,%u,%u,%u,%u\n",
                               static_cast<unsigned long>(f->seq), static_cast<unsigned long>(f->mono_ms),
                               static_cast<unsigned long long>(f->src_pts),
                               static_cast<unsigned long long>(d), static_cast<unsigned>(g_exp_us),
                               f->result.cx, f->result.cy, static_cast<unsigned>(f->result.roi),
                               f->result.cost_us, fps, pbm_saved,
                               // ↓ 跟踪层产出（启动/跟踪/丢失 + 尺度 + 本帧实际扫描窗口的四个角）：
                               //   标定"确认要几帧、ROI 开到多大、尺度跟不跟得住"就靠这几列。
                               //   角坐标是**闭区间**：roi_x1/roi_y1 是框内最后一个像素
                               static_cast<double>(f->result.radius),
                               static_cast<double>(f->result.circ),
                               static_cast<unsigned>(f->result.state),
                               static_cast<unsigned>(f->result.roi_x0),
                               static_cast<unsigned>(f->result.roi_y0),
                               static_cast<unsigned>(f->result.roi_x1),
                               static_cast<unsigned>(f->result.roi_y1));
    if (n > 0)
        g_ev->submit_csv(line, static_cast<uint32_t>(n));
}

uint64_t mono_us() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000 + static_cast<uint64_t>(ts.tv_nsec) / 1000;
}

// ---- 识别/跟踪链路的状态行与"状态迁移"取证 ----
// 为什么单独成一段：跟踪出问题时，板端能看到的只有日志。"什么时候、"为什么"切状态"
// 是最值钱的两条信息（比"当前状态是什么"重要得多），所以每次迁移单独打一行，
// 并带上迁移时的现场（候选数/质心/尺度/ROI/丢帧数）。
dart::detection::DetectionPipeline *g_det_pipe = nullptr;
uint8_t                             g_det_state_prev = 0;
uint64_t                            g_det_scan_sum = 0, g_det_scan_max = 0, g_det_total_max = 0;
uint32_t                            g_det_cnt = 0;
uint32_t                            g_dark_frames = 0;  // 连续"全图零候选"的帧数（见 kDarkHintFrames）
uint8_t                             g_thr_hint = 128;   // 当前二值化阈值（提示行用；main 启动时填）
uint64_t                            g_det_frame_no = 0; // 最近一次状态迁移时的帧号
uint64_t                            g_det_transitions = 0;       // 本秒
uint64_t                            g_det_transitions_total = 0; // 全程

// 连续多少帧"全图一个候选都没有"就提示一次（≈3.3s @90fps）。
// 为什么要有它：二值图全空时，日志里只有"命中 0 未命中 N"，而后处理/文件/播放器全都是对的 ——
// 现象和原因之间隔着一层（板端为此白跑了两轮）。这条提示把"先去看 亮度 行的 max"直说出来。
constexpr uint32_t kDarkHintFrames = 300;

// 最近一帧的装甲板结果（1Hz 状态行 / 取证用；逐帧结果本身在 DetectResult.armor 里）
dart::ArmorTarget g_det_armor_last{};

void note_det_result(const dart::Frame *f) {
    if (g_det_pipe == nullptr)
        return;
    const dart::DetectResult    &r = f->result;
    g_det_armor_last = r.armor;
    const dart::detection::DetectionStats &st = g_det_pipe->stats();

    // 全图扫描帧且一个候选都没有 → 说明整张二值图没有超过阈值的像素
    if (st.cands_last == 0 && r.roi_x0 == 0 && r.roi_y0 == 0 && r.state == 0) {
        ++g_dark_frames;
        if (g_dark_frames == kDarkHintFrames)
            log_line("提示: 连续 %u 帧全图零候选（二值图全空）。先看上面 `亮度 ... max` 那行："
                     "max 上不到阈值 %u 就是进光不够 —— 加光源，或调大 --exp（当前 %u us）重跑；"
                     "若 max 已经打到 255 且白斑明显变大，则是过曝，往回收。\n",
                     static_cast<unsigned>(g_dark_frames), static_cast<unsigned>(g_thr_hint),
                     static_cast<unsigned>(g_exp_us));
    } else {
        g_dark_frames = 0;
    }

    ++g_det_cnt;
    g_det_scan_sum += st.last_scan_us;
    if (st.last_scan_us > g_det_scan_max)
        g_det_scan_max = st.last_scan_us;
    if (r.cost_us > g_det_total_max)
        g_det_total_max = r.cost_us;

    if (r.state == g_det_state_prev)
        return;

    ++g_det_transitions;
    ++g_det_transitions_total;
    const char *from = dart::detection::to_string(static_cast<dart::detection::TrackState>(g_det_state_prev));
    const char *to = dart::detection::to_string(static_cast<dart::detection::TrackState>(r.state));
    log_line("识别: 状态 %s → %s（第%lu帧，距上次迁移%lu帧）：中心(%d,%d) r=%.1fpx ROI (%u,%u)-(%u,%u) "
             "全图候选%llu 连续丢失%u 帧 | 累计 确认%llu 丢失%llu 重捕%llu 复位%llu 拒闪%llu KF拒%llu 超框%llu\n",
             from, to, static_cast<unsigned long>(f->seq),
             static_cast<unsigned long>(f->seq - g_det_frame_no), r.cx, r.cy,
             static_cast<double>(r.radius), static_cast<unsigned>(r.roi_x0),
             static_cast<unsigned>(r.roi_y0), static_cast<unsigned>(r.roi_x1),
             static_cast<unsigned>(r.roi_y1), static_cast<unsigned long long>(st.cands_last),
             g_det_pipe->tracker().lost_frames(),
             static_cast<unsigned long long>(st.tracker.confirm_gained),
             static_cast<unsigned long long>(st.tracker.to_lost),
             static_cast<unsigned long long>(st.tracker.to_tracking),
             static_cast<unsigned long long>(st.tracker.to_startup),
             static_cast<unsigned long long>(st.tracker.confirm_reject),
             static_cast<unsigned long long>(st.tracker.kf_reject),
             static_cast<unsigned long long>(st.tracker.out_of_roi));
    g_det_state_prev = r.state;
    g_det_frame_no = f->seq;
}

// 1Hz：识别/跟踪的状态行（与录像那条状态行分开打，避免一行长到看不清）
void emit_det_status() {
    if (g_det_pipe == nullptr)
        return;
    const dart::detection::DetectionStats &st = g_det_pipe->stats();
    log_line("识别: 状态=%s | 全图帧 %llu ROI帧 %llu | 命中 %llu 未命中 %llu | 扫描 avg%lluus max%lluus "
             "整链 max%lluus | 本秒 迁移 %llu | 累计 确认 %llu 丢失 %llu 重捕(软%llu/硬%llu) 复位 %llu "
             "拒闪 %llu 贴边跳 %llu 全图确认 %llu KF拒 %llu 超框 %llu | 滑窗 %u/%u 帧\n",
             dart::detection::to_string(st.state), static_cast<unsigned long long>(st.tracker.full_scans),
             static_cast<unsigned long long>(st.tracker.roi_scans),
             static_cast<unsigned long long>(st.hits), static_cast<unsigned long long>(st.misses),
             static_cast<unsigned long long>(g_det_cnt ? g_det_scan_sum / g_det_cnt : 0),
             static_cast<unsigned long long>(g_det_scan_max),
             static_cast<unsigned long long>(g_det_total_max),
             static_cast<unsigned long long>(g_det_transitions),
             static_cast<unsigned long long>(st.tracker.confirm_gained),
             static_cast<unsigned long long>(st.tracker.to_lost),
             static_cast<unsigned long long>(st.tracker.reacquire_soft),
             static_cast<unsigned long long>(st.tracker.reacquire_hard),
             static_cast<unsigned long long>(st.tracker.to_startup),
             static_cast<unsigned long long>(st.tracker.confirm_reject),
             static_cast<unsigned long long>(st.tracker.confirm_border_skip),
             static_cast<unsigned long long>(st.tracker.full_confirm),
             static_cast<unsigned long long>(st.tracker.kf_reject),
             static_cast<unsigned long long>(st.tracker.out_of_roi),
             static_cast<unsigned>(st.tracker.confirm_frames), g_det_pipe->config().confirm.window);
    // 装甲板那一路（第二路输出）：锚=%d 远档=%d 灯尺=%d 块 %u 条 %u 对 %u 先验否 %u
    // 窗口 %uppx 耗时 %uus；mode: 0 无 / 1 本帧实测 / 2 保持(带年龄)
    const dart::ArmorTarget &at = g_det_armor_last;
    log_line("装甲板: mode=%u(年龄%u) 板心=(%d,%d) 条长 %u/%u | %s 窗%u 认亲失败%u | "
             "锚 %u 远档 %u 灯尺 %d 块 %u 条 %u 对 %u 先验否 %u 窗口 %u px 耗时 %u us\n",
             static_cast<unsigned>(at.mode), static_cast<unsigned>(at.held), at.cx, at.cy,
             static_cast<unsigned>(at.a_len), static_cast<unsigned>(at.b_len),
             st.armor_trace.mode ? "track" : "detect",
             static_cast<unsigned>(st.armor_trace.windows),
             static_cast<unsigned>(st.armor_trace.assoc_fail),
             st.armor_trace.anchored ? 1u : 0u, st.armor_trace.far ? 1u : 0u,
             st.armor_trace.scale, st.armor_trace.blobs, st.armor_trace.bars,
             st.armor_trace.pairs, st.armor_trace.rejected_prior, st.armor_trace.window_px,
             st.armor_trace.cost_us);
    g_det_scan_sum = 0;
    g_det_scan_max = 0;
    g_det_total_max = 0;
    g_det_cnt = 0;
    g_det_transitions = 0;
}

uint64_t mono_ms() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000 + static_cast<uint64_t>(ts.tv_nsec) / 1000000;
}

// 识别器占位：永远返回"无目标"。--det off 时用它，用来对照"识别链路本身的开销"。
class DetectorStub : public dart::IDetector {
public:
    dart::DetectResult detect(const dart::GrayFrame &) override { return {}; }
};

// 采集与识别参数从命令行覆盖，默认值在 core/config.hpp 与 detection/config.hpp。
// 上板时 launcher 清单里直接写，改一行重启即生效，不必重编重传：
//   /sdcard/app/self_guiding_dart --acq 1920x1080 --fps 30 --roi-kp 6 &
// --seconds N：跑 N 秒后自动干净退出。launcher 清单里前台项是**顺序执行**的，
//   于是「一次上电连测四档传感器模式」变成可能（每档一条前台项）。
// 识别/跟踪的旋钮都放在这里，是为了让"确认要几帧、ROI 开多大、尺度跟不跟得住"
// 这些只能在板端看出来的问题，能靠改一行参数复现/对比，而不是重编固件。
void parse_args(int argc, char **argv, dart::Config &cfg, dart::detection::DetectionConfig &dcfg) {
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--acq") && i + 1 < argc) {
            unsigned w = 0, h = 0;
            if (std::sscanf(argv[++i], "%ux%u", &w, &h) == 2) {
                cfg.sensor_width = w;
                cfg.sensor_height = h;
            }
        } else if (!std::strcmp(argv[i], "--fps") && i + 1 < argc) {
            cfg.sensor_fps = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--frame") && i + 1 < argc) {
            // 识别/录像画幅（VICAP CHN0 输出 = 帧池 = 编码尺寸）。改一行即可验证别的档位。
            unsigned w = 0, h = 0;
            if (std::sscanf(argv[++i], "%ux%u", &w, &h) == 2) {
                cfg.width = w;
                cfg.height = h;
            }
        } else if (!std::strcmp(argv[i], "--exp") && i + 1 < argc) {
            // 固定曝光（微秒）；0 = 交回 AE。默认值在 core/config.hpp::exposure_us，
            // 这个开关只为"一轮试一档曝光"存在（不用重编重传）。
            cfg.exposure_us = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--thr") && i + 1 < argc) {
            cfg.threshold = static_cast<uint8_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--pbm") && i + 1 < argc) {
            cfg.pbm_every = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--raw") && i + 1 < argc) {
            cfg.raw_every = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--csv")) {
            cfg.csv = true;
        } else if (!std::strcmp(argv[i], "--venc-mode") && i + 1 < argc) {
            cfg.venc_mode = argv[++i][0];
            cfg.venc_probe = true;
        } else if (!std::strcmp(argv[i], "--venc-intbuf") && i + 1 < argc) {
            cfg.venc_intbuf_mb = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--venc-probe")) {
            cfg.venc_probe = true;
        } else if (!std::strcmp(argv[i], "--venc-chn") && i + 1 < argc) {
            cfg.venc_chn = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--out") && i + 1 < argc) {
            cfg.out_dir = argv[++i];
        } else if (!std::strcmp(argv[i], "--seconds") && i + 1 < argc) {
            cfg.run_seconds = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--vision") && i + 1 < argc) {
            cfg.vision_seconds = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--mem-slim")) {
            // 让内存给编码器：VENC 内部缓冲也从 MMZ 出，池吃太满时编码器会"收帧不吐码流"
            cfg.frame_slots = 6;
            cfg.cap_buffers = 4;
            cfg.chn_buffers = 4;
            cfg.rec_in_blocks = 3;
            cfg.rec_out_blocks = 4;
        } else if (!std::strcmp(argv[i], "--no-record")) {
            cfg.no_record = true;
        }
        // ---------------- 识别/跟踪（工况 1 与工况 2）----------------
        else if (!std::strcmp(argv[i], "--det") && i + 1 < argc) {
            // off = 换回 DetectorStub（恒无目标），用来量"识别链路本身"的开销基线
            g_det_on = std::strcmp(argv[++i], "off") != 0;
        } else if (!std::strcmp(argv[i], "--no-track")) {
            // 只跑启动阶段（全图 RVV 粗筛 + 3 帧滑窗确认），不开 ROI/滤波：
            // 怀疑 ROI 门控或滤波是回归元凶时，先关它看现象是否消失
            dcfg.enable_tracking = false;
        } else if (!std::strcmp(argv[i], "--det-topk") && i + 1 < argc) {
            dcfg.scan.top_k = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--det-min-area") && i + 1 < argc) {
            dcfg.scan.min_area = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--det-tile") && i + 1 < argc) {
            dcfg.scan.tile = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--armor-off")) {
            // 关掉装甲板那一路（绿灯照跑）：用来做 A/B 对照，看它是不是在帮倒忙
            dcfg.armor.enable = false;
        } else if (!std::strcmp(argv[i], "--armor-hold") && i + 1 < argc) {
            dcfg.armor.hold = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--armor-aspect") && i + 1 < argc) {
            dcfg.armor.aspect_min = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--armor-len-k") && i + 1 < argc) {
            // 灯条长度门限 = 灯尺(2r) 的倍数：远→近自动缩放，这两个数定范围
            dcfg.armor.len_min_k = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--armor-len-kmax") && i + 1 < argc) {
            dcfg.armor.len_max_k = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--armor-roi-w") && i + 1 < argc) {
            dcfg.armor.roi_w_k = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--armor-roi-h") && i + 1 < argc) {
            dcfg.armor.roi_h_k = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--armor-span") && i + 2 < argc) {
            // 刚性几何先验：两灯条间距 / 灯尺 的允许范围（用日志"比值"填，才真正收紧）
            dcfg.armor.span_lo_k = static_cast<float>(std::atof(argv[++i]));
            dcfg.armor.span_hi_k = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--armor-circ-far") && i + 1 < argc) {
            // 远档的圆度上限：灯条"不像圆"（3px 也成立的判据）
            dcfg.armor.circ_max_far = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--circ-weight") && i + 1 < argc) {
            // 圆度在排序里的占比：score = 面积 × 圆度^w（0 = 退回"纯面积"）
            const float cw = static_cast<float>(std::atof(argv[++i]));
            dcfg.scan.circ_weight = cw;
            dcfg.measure.circ_weight = cw;
        } else if (!std::strcmp(argv[i], "--min-circ") && i + 1 < argc) {
            // 圆度硬门限（0 = 不筛）；发光体场景 0.4~0.6 可直接丢掉细长反光
            const float mc = static_cast<float>(std::atof(argv[++i]));
            dcfg.scan.min_circularity = mc;
            dcfg.measure.min_circularity = mc;
        } else if (!std::strcmp(argv[i], "--confirm-window") && i + 1 < argc) {
            dcfg.confirm.window = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--confirm-hits") && i + 1 < argc) {
            dcfg.confirm.min_hits = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--confirm-accel") && i + 1 < argc) {
            dcfg.confirm.max_accel_px = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--confirm-gate") && i + 1 < argc) {
            dcfg.confirm.gate_px = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--roi-kp") && i + 1 < argc) {
            dcfg.roi.kp = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--roi-margin") && i + 1 < argc) {
            dcfg.roi.margin = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--roi-ksigma") && i + 1 < argc) {
            // 0 = 严格用规格公式 W = kp·ŝ + B_margin
            dcfg.roi.k_sigma = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--kf-rfar") && i + 1 < argc) {
            dcfg.kf.r_scale_far = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--kf-rnear") && i + 1 < argc) {
            dcfg.kf.r_scale_near = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--kf-sfar") && i + 1 < argc) {
            dcfg.kf.s_far = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--kf-snear") && i + 1 < argc) {
            dcfg.kf.s_near = static_cast<float>(std::atof(argv[++i]));
        } else if (!std::strcmp(argv[i], "--lost-after") && i + 1 < argc) {
            dcfg.tracker.lost_after = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else if (!std::strcmp(argv[i], "--rescan-after") && i + 1 < argc) {
            dcfg.tracker.rescan_after = static_cast<uint32_t>(std::atoi(argv[++i]));
        } else {
            // **不认识的参数必须打出来**：以前这里是静默跳过，于是"改了一行参数却没生效"
            // 会完全没有痕迹（板端 round13 之前就踩过：把曝光写在别的应用里，
            // 命令行再写错一个开关名，两边都无声无息）。宁可多一行日志。
            log_line("启动: 忽略无法识别的参数 '%s'（typo? 见 src/startup.list 的旋钮清单）\n", argv[i]);
        }
    }
}

} // namespace

int main(int argc, char **argv) {
    using namespace dart;

    // 第一件事用 raw write：不经 stdio、不碰堆。板子上没有 shell，日志是唯一窗口，
    // 这一行用来区分「内核没把进程拉起来」和「进来了但卡在初始化里」。
    static const char kEnterMain[] = "self_guiding_dart: enter main\n";
    (void)!write(STDOUT_FILENO, kEnterMain, sizeof(kEnterMain) - 1);

    // 之后的每一行都走 log_line：同时写日志文件（每行 fsync）和 stdout。
    // 日志文件有三级兜底路径，见 core/log.cpp —— 打不开也要留下证据。
    log_line("self_guiding_dart: 日志开始\n");
    log_line("self_guiding_dart: 日志文件 %s\n", log_path());

    Config cfg; // 工况与内存账：core/config.hpp
    dart::detection::DetectionConfig dcfg; // 识别/跟踪参数：detection/config.hpp
    parse_args(argc, argv, cfg, dcfg);
    dcfg.sanitize();
    log_line("self_guiding_dart: 请求 %ux%u@%u, 画幅 %ux%u venc_chn=%u, 输出 %s\n", cfg.sensor_width,
             cfg.sensor_height, cfg.sensor_fps, cfg.width, cfg.height, cfg.venc_chn, cfg.out_dir);
    {
        char det_buf[768];
        dart::detection::DetectionPipeline::describe(dcfg, det_buf, sizeof(det_buf));
        log_line("self_guiding_dart: 识别/跟踪参数 %s\n", det_buf);
    }

    single_instance_guard();

    // RVV 自检放在碰 MPP 之前：如果向量单元真的有问题，日志会停在"自检开始"这一行，
    // 而不会和采集初始化混在一起（板端实测两个程序都在首次执行 RVV 时静默停住）。
    log_line("self_guiding_dart: RVV 自检开始\n");
    const bool rvv_ok = GraphicsUtils::selftest();
    log_line("self_guiding_dart: RVV 自检 %s\n", rvv_ok ? "PASS" : "FAIL");

    // 识别层的自检同样放在碰 MPP 之前：RVV 全图粗筛的向量路径 vs 标量参考逐位比对
    // （板端没有 RISC-V 模拟器，这是唯一能证明向量代码没写错的办法），
    // 外加 ROI 测量器的解析校验（窗口绝对坐标/面积换算写错的话，这里就会 FAIL）。
    if (g_det_on) {
        const bool det_ok = dart::detection::DetectionPipeline::selftest();
        log_line("self_guiding_dart: 识别自检 %s（RVV 粗筛 vs 标量 + ROI 测量解析）\n",
                 det_ok ? "PASS" : "FAIL");
    }

    spawn_watchdog(); // 先立心跳，后面的冻结才有判据（pid 记在 g_watchdog，由 atexit 收）

    log_line("self_guiding_dart: MMZ 余量(开机基线) %u MB（单块最大 %u MB）\n", mmz_free_mb(),
             mmz_max_block_mb());

    // 识别器：启动态全图 RVV 粗筛 + 3 帧滑窗确认 → 跟踪态动态 ROI + 尺度自适应卡尔曼。
    // **堆分配**：DetectionPipeline 约 7.8KB（跟踪器里有个 8 槽滑窗环 + 6×6 协方差），
    // 而 main 的栈上已经放了 SensorSource/FramePool/Recorder，再压 8KB 就是在赌栈够大 ——
    // 病历 6-A 的教训（栈上放大对象 → 整机重启）就在隔壁，取证通道的 EvidenceWriter 也是同样的做法。
    // 声明顺序 = 析构逆序：pipeline → Recorder → FramePool → SensorSource（VB 由最后一个退出）
    SensorSource sensor(cfg);
    FramePool    pool(cfg.width, cfg.height, cfg.frame_slots);
    // --det off：换回占位识别器（恒无目标），把"识别链路的开销与副作用"从现象里摘出去
    DetectorStub                                          detector_stub;
    std::unique_ptr<dart::detection::DetectionPipeline>   detector_owned;
    if (g_det_on)
        detector_owned.reset(new dart::detection::DetectionPipeline(dcfg));
    dart::IDetector &detector = g_det_on ? static_cast<dart::IDetector &>(*detector_owned)
                                         : static_cast<dart::IDetector &>(detector_stub);
    if (g_det_on)
        g_det_pipe = detector_owned.get(); // 1Hz 状态行/迁移取证要读它
    Recorder recorder(cfg, pool);

    std::signal(SIGINT, on_stop);
    std::signal(SIGSEGV, on_fault); // 崩溃留痕：静默死掉查不到东西
    std::signal(SIGBUS, on_fault);
    std::signal(SIGILL, on_fault);
    std::signal(SIGFPE, on_fault);
    std::signal(SIGABRT, on_fault);
    if (cfg.run_seconds) { // 诊断/限时跑：到点用 SIGALRM 让主循环干净退出（析构会 deinit VICAP）
        std::signal(SIGALRM, on_stop);
        alarm(cfg.run_seconds);
        log_line("self_guiding_dart: %u 秒后自动退出\n", cfg.run_seconds);
    }
    log_line("sensor %ux%u@%u -> %ux%u, 帧池 %u x %u KB\n", sensor.width(), sensor.height(), sensor.fps(),
             pool.width(), pool.height(), pool.slots(),
             static_cast<unsigned>(pool.width() * pool.height() * 3 / 2 / 1024));

    sensor.start();
    g_exp_us = cfg.exposure_us; // 固定曝光：只在启动时按配置下发一次（sensor_source.apply_exposure）
    g_thr_hint = cfg.threshold; // 提示行要用（"max 上不到阈值"里那个阈值）
    mkdir(cfg.out_dir, 0777);
    std::unique_ptr<dart::EvidenceWriter> ev;
    if (cfg.csv || cfg.pbm_every || cfg.raw_every) {
        ev.reset(new dart::EvidenceWriter(cfg.out_dir)); // ~1.4MB 缓冲，必须堆分配
        g_ev = ev.get();
    }

    // 阶段划分：先纯视觉跑 vision_seconds 秒（不碰 VENC），拿到真实帧率后再开录像。
    // 录像只是旁路：建不起来（通道被占、池不够、/sdcard 只读）就打日志继续跑识别，
    // 绝不因为录像初始化失败把整个进程带走 —— 上一轮"死得有点多"很可能就是这条。
    const uint64_t vision_end = cfg.vision_seconds ? mono_ms() + cfg.vision_seconds * 1000ull : 0;
    // 限时靠主循环自己判（板端实测 SIGALRM 不会来，alarm() 那条只当额外保险）
    const uint64_t run_end = cfg.run_seconds ? mono_ms() + cfg.run_seconds * 1000ull : 0;
    bool recording = false;
    bool record_tried = false;
    if (cfg.vision_seconds == 0 && !cfg.no_record) {
        recording = recorder.start();
        record_tried = true;
    }

    Frame   *prev = nullptr; // 上一帧：等这一拍识别完就交给录像
    uint64_t frames = 0, starved = 0, noframe = 0;
    uint64_t report_t = mono_ms(), report_f = 0;

    while (g_run) {
        if (run_end != 0 && mono_ms() >= run_end) {
            log_line("self_guiding_dart: 到达 --seconds %u，开始干净退出\n", cfg.run_seconds);
            break;
        }

        const uint64_t n = frames + 1; // 本拍帧号（调试打点用）
        const bool trace = (n <= 3);   // 只对前 3 拍逐步打点，定位停在哪一步

        if (!recording && !record_tried && cfg.vision_seconds && mono_ms() >= vision_end) {
            log_line("阶段2: 纯视觉结束，开始录像\n");
            record_tried = true;
            if (cfg.no_record)
                log_line("阶段2: --no-record，保持纯视觉\n");
            else
                recording = recorder.start();
        }

        // 帧池空（录像还没还帧）就丢这一拍：识别链路的节拍优先于录像
        Frame *cur = pool.acquire();
        if (cur == nullptr) {
            ++starved;      // 池被录像攥住（正常不该发生：见 Recorder 的预留门限）
            usleep(200);    // 让出 CPU，绝不空转刷 MPP
            continue;
        }

        { // ①② 取帧 → 二值化进池帧；raw 出作用域即归还 VICAP
            if (trace)
                log_line("帧%lu: 取帧前\n", static_cast<unsigned long>(n));
            auto raw = sensor.capture();
            if (trace)
                log_line("帧%lu: 取帧后 y=%s\n", static_cast<unsigned long>(n),
                         raw.y != nullptr ? "有帧" : "空帧");
            if (raw.y == nullptr) { // 这一拍没帧（capture 内部已限频打印错误码）
                pool.release(cur);
                ++noframe;
                usleep(1000); // 绝不能空转：VICAP 一旦 NOTREADY，dump 会立刻返回错误
                continue;
            }
            cur->src_pts = raw.pts;      // 取证：源帧时间戳（量真实帧间隔，不数循环）
            pool.set_mod_id(raw.mod_id); // 池帧照抄源 VICAP 帧的 mod_id（幂等）
            // 原始 Y 平面（标定阈值/曝光要和二值图对着看）：必须在 raw 出作用域之前存
            if (g_ev != nullptr && cfg.raw_every && (n % cfg.raw_every) == 0) {
                g_ev->submit_pgm(n, raw.y, raw.width, raw.height, raw.stride);
                // 亮度统计：曝光合不合适，看这两个数就够（板端 250us 时均值仅 0.3/255）
                uint64_t sum = 0;
                uint8_t mn = 255, mx = 0;
                const size_t plane = static_cast<size_t>(raw.stride) * raw.height;
                for (size_t i = 0; i < plane; ++i) {
                    const uint8_t v = raw.y[i];
                    sum += v;
                    if (v < mn) mn = v;
                    if (v > mx) mx = v;
                }
                log_line("取证: raw%06lu.pgm 亮度 均值%.1f min%u max%u（曝光 %u us）\n",
                         static_cast<unsigned long>(n), static_cast<double>(sum) / static_cast<double>(plane),
                         static_cast<unsigned>(mn), static_cast<unsigned>(mx),
                         static_cast<unsigned>(g_exp_us));
            }
            GraphicsUtils::binarize(raw.y, cur->view.pixels, raw.width, raw.height, cfg.threshold);
            if (trace)
                log_line("帧%lu: 二值化完成\n", static_cast<unsigned long>(n));
        }

        cur->seq = ++frames;
        cur->mono_ms = mono_ms();

        // ③ 识别当前帧。耗时用**微秒**钟量：毫秒粒度下"启动阶段全图粗筛 ~0.15ms"会一律
        // 显示成 0（DEBUG_GUIDE 环节 5 记过这个坑），那样就没法判断它到底占了多少预算。
        const uint64_t t0 = mono_us();
        cur->result = detector.detect(cur->view);
        cur->result.cost_us = static_cast<uint32_t>(mono_us() - t0);
        note_det_result(cur); // 状态迁移取证（每次迁移一行，带现场）
        if (trace)
            log_line("帧%lu: 识别完成\n", static_cast<unsigned long>(n));

        { // 取证通道：PBM/PGM 只做打包入队，I/O 在写线程（不占视觉链路）
            const uint64_t now = mono_ms();
            const double fps_now = (now > report_t)
                                       ? static_cast<double>(frames - report_f) * 1000.0 /
                                             static_cast<double>(now - report_t)
                                       : 0.0;
            int pbm_saved = 0;
            if (g_ev != nullptr && cfg.pbm_every && (frames % cfg.pbm_every) == 1) {
                const uint64_t t_pack = mono_us();
                g_ev->submit_pbm(frames, cur->view.pixels, cur->view.width, cur->view.height,
                                 cur->view.stride);
                const uint64_t dt = mono_us() - t_pack;
                if (dt > g_pbm_us_max)
                    g_pbm_us_max = dt;
                g_pbm_us_sum += dt;
                ++g_pbm_us_cnt;
                pbm_saved = 1;
            }
            if (cfg.csv)
                emit_csv(cur, fps_now, pbm_saved);
        }

        if (prev != nullptr && recording) {
            if (trace)
                log_line("帧%lu: 送录像前\n", static_cast<unsigned long>(n));
            recorder.submit(prev); // ④ 上一帧交给录像线程（入队即交出所有权，不阻塞）
            if (trace)
                log_line("帧%lu: 送录像后 (recorded=%lu 失败=%lu)\n", static_cast<unsigned long>(n),
                         static_cast<unsigned long>(recorder.frames()),
                         static_cast<unsigned long>(recorder.send_fail()));
        }

        else if (prev != nullptr) {
            pool.release(prev); // 纯视觉阶段没有录像接手，帧必须自己还池
        }

        prev = cur; // ⑤ 识别器转到下一帧

        if (trace)
            log_line("帧%lu: 回到循环顶\n", static_cast<unsigned long>(n));

        if (mono_ms() - report_t >= 1000) { // 约 1Hz 状态行
            const uint64_t now = mono_ms();
            log_line("%.1f fps | 无帧 %lu | 丢拍 %lu | 编码器 %lu 帧 %.2f MB | 送帧失败 %lu | 无块 %lu | 队列丢 %lu | 强收 %lu | 出包多 %lu | IO错 %lu | 识别 %u us | 在途 %u | PBM %llu 丢 %llu 打包max%lluus avg%lluus | 写线程忙 %lluus/s\n",
                     static_cast<double>(frames - report_f) * 1000.0 / static_cast<double>(now - report_t),
                     static_cast<unsigned long>(noframe), static_cast<unsigned long>(starved),
                     static_cast<unsigned long>(recorder.frames()),
                     static_cast<double>(recorder.bytes()) / (1024 * 1024),
                     static_cast<unsigned long>(recorder.send_fail()),
                     static_cast<unsigned long>(recorder.blk_empty()),
                     static_cast<unsigned long>(recorder.dropped()),
                     static_cast<unsigned long>(recorder.reclaimed()),
                     static_cast<unsigned long>(recorder.underflow()),
                     static_cast<unsigned long>(recorder.io_fail()), cur->result.cost_us,
                     recorder.inflight(),
                     static_cast<unsigned long long>(g_ev ? g_ev->written_pbm() : 0),
                     static_cast<unsigned long long>(g_ev ? g_ev->dropped() : 0),
                     static_cast<unsigned long long>(g_pbm_us_max),
                     static_cast<unsigned long long>(g_pbm_us_cnt ? g_pbm_us_sum / g_pbm_us_cnt : 0),
                     static_cast<unsigned long long>(g_ev ? g_ev->busy_us() - g_ev_busy_prev : 0));
            g_ev_busy_prev = g_ev ? g_ev->busy_us() : 0;
            g_pbm_us_max = 0;
            g_pbm_us_sum = 0;
            g_pbm_us_cnt = 0;
            report_t = now;
            report_f = frames;
            emit_det_status(); // 识别/跟踪那条状态行（与上面这条分开打）
        }
    }

    if (prev != nullptr)
        pool.release(prev);
    if (g_det_pipe != nullptr) { // 识别链路的总结：上板复盘时它比"最后一帧的状态行"有用
        const dart::detection::DetectionStats &st = g_det_pipe->stats();
        log_line("识别总结: 处理 %llu 帧，命中 %llu 未命中 %llu | 全图扫描 %llu 帧 ROI 扫描 %llu 帧 | "
                 "确认 %llu 丢失 %llu 重捕(软%llu/硬%llu) 复位 %llu | 拒闪 %llu 贴边跳 %llu 全图确认 %llu KF拒 %llu 超框 %llu | "
                 "状态迁移 %llu 次，末态 %s\n",
                 static_cast<unsigned long long>(st.frames), static_cast<unsigned long long>(st.hits),
                 static_cast<unsigned long long>(st.misses),
                 static_cast<unsigned long long>(st.tracker.full_scans),
                 static_cast<unsigned long long>(st.tracker.roi_scans),
                 static_cast<unsigned long long>(st.tracker.confirm_gained),
                 static_cast<unsigned long long>(st.tracker.to_lost),
                 static_cast<unsigned long long>(st.tracker.reacquire_soft),
                 static_cast<unsigned long long>(st.tracker.reacquire_hard),
                 static_cast<unsigned long long>(st.tracker.to_startup),
                 static_cast<unsigned long long>(st.tracker.confirm_reject),
                 static_cast<unsigned long long>(st.tracker.confirm_border_skip),
                 static_cast<unsigned long long>(st.tracker.full_confirm),
                 static_cast<unsigned long long>(st.tracker.kf_reject),
                 static_cast<unsigned long long>(st.tracker.out_of_roi),
                 static_cast<unsigned long long>(g_det_transitions_total),
                 dart::detection::to_string(st.state));
        g_det_pipe = nullptr;
    }
    if (g_ev != nullptr) {
        log_line("取证: PBM %llu 张 / PGM %llu 张，队列丢 %llu，写失败 %llu\n",
                 static_cast<unsigned long long>(g_ev->written_pbm()),
                 static_cast<unsigned long long>(g_ev->written_pgm()),
                 static_cast<unsigned long long>(g_ev->dropped()),
                 static_cast<unsigned long long>(g_ev->failed()));
        g_ev = nullptr;
        ev.reset(); // 等写线程收尾（flush CSV / 写完队列）
    }
    // 先把总结行打出来：stop() 里如果触发拆链，板子可能重启，日志会停在那里
    log_line("退出(拆链前)：采集 %lu 帧，无帧 %lu 拍，编码器录像 %lu 帧 %.2f MB（失败 %lu，无块 %lu，队列丢 %lu，强收 %lu，出包多 %lu，IO错 %lu），丢拍 %lu\n",
             static_cast<unsigned long>(frames), static_cast<unsigned long>(noframe),
             static_cast<unsigned long>(recorder.frames()),
             static_cast<double>(recorder.bytes()) / (1024 * 1024),
             static_cast<unsigned long>(recorder.send_fail()),
             static_cast<unsigned long>(recorder.blk_empty()),
             static_cast<unsigned long>(recorder.dropped()),
             static_cast<unsigned long>(recorder.reclaimed()),
             static_cast<unsigned long>(recorder.underflow()),
             static_cast<unsigned long>(recorder.io_fail()), static_cast<unsigned long>(starved));
    recorder.stop();
    sensor.stop();
    mpp_map_shutdown(); // 线程都 join 之后才统一解映射（老工程铁律）
    kill_watchdog();

    if (!recording && record_tried && !cfg.no_record)
        log_line("退出：录像链路本轮从未起来（原因见上面 '录像: …失败' 那几行）\n");
    log_line("退出：采集 %lu 帧，无帧 %lu 拍，编码器录像 %lu 帧 %.2f MB（失败 %lu，无块 %lu，队列丢 %lu，强收 %lu，出包多 %lu，IO错 %lu），丢拍 %lu；取证通道另计（见上一行 '取证: PBM …'）\n",
             static_cast<unsigned long>(frames), static_cast<unsigned long>(noframe),
             static_cast<unsigned long>(recorder.frames()),
             static_cast<double>(recorder.bytes()) / (1024 * 1024),
             static_cast<unsigned long>(recorder.send_fail()),
             static_cast<unsigned long>(recorder.blk_empty()),
             static_cast<unsigned long>(recorder.dropped()),
             static_cast<unsigned long>(recorder.reclaimed()),
             static_cast<unsigned long>(recorder.underflow()),
             static_cast<unsigned long>(recorder.io_fail()), static_cast<unsigned long>(starved));
    return 0;
}
