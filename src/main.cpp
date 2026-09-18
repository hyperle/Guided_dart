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
#include "detection/graphics_utils.hpp"
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
    char line[224];
    const int n = std::snprintf(line, sizeof(line), "%lu,%lu,%llu,%llu,%u,%d,%d,%u,%u,%.1f,%d\n",
                               static_cast<unsigned long>(f->seq), static_cast<unsigned long>(f->mono_ms),
                               static_cast<unsigned long long>(f->src_pts),
                               static_cast<unsigned long long>(d), static_cast<unsigned>(g_exp_us),
                               f->result.cx, f->result.cy, static_cast<unsigned>(f->result.roi),
                               f->result.cost_us, fps, pbm_saved);
    if (n > 0)
        g_ev->submit_csv(line, static_cast<uint32_t>(n));
}

uint64_t mono_us() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000 + static_cast<uint64_t>(ts.tv_nsec) / 1000;
}

uint64_t mono_ms() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000 + static_cast<uint64_t>(ts.tv_nsec) / 1000000;
}

// 识别器占位：永远返回"无目标"。写识别器时照 dart::IDetector 实现一个类替换掉它即可，
// 流程、内存、录像都不用动。
class DetectorStub : public dart::IDetector {
public:
    dart::DetectResult detect(const dart::GrayFrame &) override { return {}; }
};

// 采集参数从命令行覆盖，默认值在 core/config.hpp。
// 上板时 launcher 清单里直接写，改一行重启即生效，不必重编重传：
//   /sdcard/app/self_guiding_dart --acq 1920x1080 --fps 30 &
// --seconds N：跑 N 秒后自动干净退出。launcher 清单里前台项是**顺序执行**的，
//   于是「一次上电连测四档传感器模式」变成可能（每档一条前台项）。
void parse_args(int argc, char **argv, dart::Config &cfg) {
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
    parse_args(argc, argv, cfg);
    log_line("self_guiding_dart: 请求 %ux%u@%u, 画幅 %ux%u venc_chn=%u, 输出 %s\n", cfg.sensor_width,
             cfg.sensor_height, cfg.sensor_fps, cfg.width, cfg.height, cfg.venc_chn, cfg.out_dir);

    single_instance_guard();

    // RVV 自检放在碰 MPP 之前：如果向量单元真的有问题，日志会停在"自检开始"这一行，
    // 而不会和采集初始化混在一起（板端实测两个程序都在首次执行 RVV 时静默停住）。
    log_line("self_guiding_dart: RVV 自检开始\n");
    const bool rvv_ok = GraphicsUtils::selftest();
    log_line("self_guiding_dart: RVV 自检 %s\n", rvv_ok ? "PASS" : "FAIL");

    spawn_watchdog(); // 先立心跳，后面的冻结才有判据（pid 记在 g_watchdog，由 atexit 收）

    log_line("self_guiding_dart: MMZ 余量(开机基线) %u MB（单块最大 %u MB）\n", mmz_free_mb(),
             mmz_max_block_mb());

    // 声明顺序 = 析构逆序：Recorder → FramePool → SensorSource（VB 由最后一个退出）
    SensorSource sensor(cfg);
    FramePool    pool(cfg.width, cfg.height, cfg.frame_slots);
    DetectorStub detector; // ← 换成你的识别器（实现 dart::IDetector 即可）
    Recorder     recorder(cfg, pool);

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

        const uint64_t t0 = mono_ms();
        cur->result = detector.detect(cur->view); // ③ 识别当前帧
        cur->result.cost_us = static_cast<uint32_t>((mono_ms() - t0) * 1000);
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
        }
    }

    if (prev != nullptr)
        pool.release(prev);
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
