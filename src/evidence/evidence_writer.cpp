#include "evidence/evidence_writer.hpp"

#include <ctype.h>
#include <dirent.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <ctime>
#include <chrono>
#include <cstring>

#include "core/log.hpp"

namespace dart {
namespace {

uint64_t busy_clock_us() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000 + static_cast<uint64_t>(ts.tv_nsec) / 1000;
}

void put_u32_dec(char *&p, uint32_t v) {
    char tmp[12];
    int n = 0;
    if (v == 0)
        tmp[n++] = '0';
    while (v > 0) {
        tmp[n++] = static_cast<char>('0' + (v % 10));
        v /= 10;
    }
    while (n > 0)
        *p++ = tmp[--n];
}

// PBM(P4)：1bit/像素、行末按字节对齐，bit=1 表示**黑**（与 graphics_utils::save_pbm 一致）
// 8 像素一次成型、无分支：板端量到逐像素带分支的版本一次打包要 1.4ms（230K 次迭代），
// 这里把内层换成 8 路比较+或，省掉每次迭代的分支预测失败。
uint32_t pack_pbm(uint8_t *dst, const uint8_t *bin, uint32_t w, uint32_t h, uint32_t stride) {
    char *p = reinterpret_cast<char *>(dst);
    *p++ = 'P';
    *p++ = '4';
    *p++ = '\n';
    put_u32_dec(p, w);
    *p++ = ' ';
    put_u32_dec(p, h);
    *p++ = '\n';
    const uint32_t hdr = static_cast<uint32_t>(p - reinterpret_cast<char *>(dst));

    const uint32_t row_bytes = (w + 7u) / 8u;
    for (uint32_t y = 0; y < h; ++y) {
        const uint8_t *src = bin + static_cast<size_t>(y) * stride;
        uint8_t *out = dst + hdr + static_cast<size_t>(y) * row_bytes;
        uint32_t x = 0;
        for (; x + 8 <= w; x += 8) {
            const uint8_t *q = src + x;
            out[x >> 3] = static_cast<uint8_t>(((q[0] <= 127) ? 0x80u : 0u) | ((q[1] <= 127) ? 0x40u : 0u) |
                                               ((q[2] <= 127) ? 0x20u : 0u) | ((q[3] <= 127) ? 0x10u : 0u) |
                                               ((q[4] <= 127) ? 0x08u : 0u) | ((q[5] <= 127) ? 0x04u : 0u) |
                                               ((q[6] <= 127) ? 0x02u : 0u) | ((q[7] <= 127) ? 0x01u : 0u));
        }
        for (; x < w; ++x) { // 行尾不足 8 像素
            const uint32_t byte = x >> 3;
            if ((x & 7u) == 0)
                out[byte] = 0;
            if (src[x] <= 127)
                out[byte] |= static_cast<uint8_t>(0x80u >> (x & 7u));
        }
    }
    return hdr + row_bytes * h;
}

uint32_t pack_pgm(uint8_t *dst, const uint8_t *y_plane, uint32_t w, uint32_t h, uint32_t stride) {
    char *p = reinterpret_cast<char *>(dst);
    *p++ = 'P';
    *p++ = '5';
    *p++ = '\n';
    put_u32_dec(p, w);
    *p++ = ' ';
    put_u32_dec(p, h);
    *p++ = '\n';
    *p++ = '2';
    *p++ = '5';
    *p++ = '5';
    *p++ = '\n';
    const uint32_t hdr = static_cast<uint32_t>(p - reinterpret_cast<char *>(dst));

    uint8_t *out = dst + hdr;
    if (stride == w) {
        std::memcpy(out, y_plane, static_cast<size_t>(w) * h); // 紧凑：一次拷完
    } else {
        for (uint32_t y = 0; y < h; ++y)
            std::memcpy(out + static_cast<size_t>(y) * w, y_plane + static_cast<size_t>(y) * stride, w);
    }
    return hdr + w * h;
}

} // namespace

// 注意：本对象约 640KB（PBM 4 槽 + PGM 2 槽），**必须堆分配**，不要放栈上。
//
// 目录布局（板端实测一轮 90s 会产生 800+ 张 PBM + 84 张 PGM，混在一起时 frames.csv
// 会被埋掉、按时间排序还找不到 —— 而且板上 RTC 没设，vfat 会把所有文件写成同一个时间戳）：
//
//   <out_dir>/frames.csv      每帧一行，**放根目录**，一眼就能看到
//   <out_dir>/img/f%06lu.pbm  二值图（新）
//   <out_dir>/img/raw%06lu.pgm 原始 Y 平面（新）
//
// 想改这个布局：主机侧 scripts/{pbm_review,detect_review}.py 里有对应的解析（都兼容旧布局）。
EvidenceWriter::EvidenceWriter(const char *dir) {
    std::snprintf(dir_, sizeof(dir_), "%s", dir != nullptr ? dir : "");
    if (dir_[0] != '/') { // 路径必须绝对：上一版 dir_ 被冲空后写出过 /f000001.pbm
        log_line("取证: 目录参数非法('%s')，取证通道关闭\n", dir_);
        return;
    }
    if (mkdir(dir_, 0777) != 0 && errno != EEXIST)
        log_line("取证: mkdir %s 失败 errno=%d（继续试开文件）\n", dir_, errno);

    // 图像子目录：先建出来，失败也不致命（后面 open 会再报一次具体路径）。
    // 路径拼不下就**直接关掉取证通道**：截断过的路径会写到别的目录，
    // 那种"文件不知道去哪了"比"没有文件"难查得多（上一版 dir_ 被冲空就写出过 /f000001.pbm）。
    const int n_img = std::snprintf(img_dir_, sizeof(img_dir_), "%s/img", dir_);
    if (n_img <= 0 || n_img >= static_cast<int>(sizeof(img_dir_))) {
        log_line("取证: 输出目录过长('%s')，取证通道关闭\n", dir_);
        dir_[0] = '\0';
        return;
    }
    if (mkdir(img_dir_, 0777) != 0 && errno != EEXIST)
        log_line("取证: mkdir %s 失败 errno=%d（图像会写不进去）\n", img_dir_, errno);
    else
        clear_images(); // 图像不跨轮复用：清掉上一轮的，保证 img/ 与 frames.csv 是同一轮

    char path[224];
    std::snprintf(path, sizeof(path), "%s/frames.csv", dir_);
    // 上一轮的 CSV 改名保留一代：板上是 launcher 自启，**多上一次电就多跑一轮**，
    // 而 frames.csv 是每次启动截断重写的 —— 板端 round13 实测：日志里攒了三轮，
    // CSV/图像却只剩最后一轮的（前两轮的数据被静默覆盖）。
    // 图像不跟着轮转（img/ 里 600+ 个文件重命名/删除的失败面太大）：**图随 CSV 一起清**，
    // 这样"某一轮的 CSV + 同一轮的图"永远配得上（配对错比没有更难查）。
    char prev_csv[224];
    std::snprintf(prev_csv, sizeof(prev_csv), "%s/frames_prev.csv", dir_);
    if (rename(path, prev_csv) == 0)
        log_line("取证: 上一轮 CSV 保留为 %s（其图像已随本轮清空，别配对分析）\n", prev_csv);

    csv_fd_ = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (csv_fd_ < 0) {
        log_line("取证: %s 打不开，取证通道关闭\n", path);
        return;
    }
    // 表头必须与 main.cpp 的 emit_csv 列**顺序一致**（末尾 6 列是识别/跟踪层加的：
    // 等效半径、状态 0启动/1跟踪/2丢失、本帧实际扫描窗口的**四个角像素坐标**）。
    // 主机侧 py 脚本用 DictReader 按列名取值，所以加列是安全的，但改名会让脚本静默取不到值
    // （老 CSV 用 roi_x/roi_y/roi_w/roi_h，脚本里仍兼容，能读旧归档）。
    const char *hdr =
        "seq,mono_ms,src_pts,d_pts_us,exp_us,cx,cy,roi,cost_us,captured_fps,pbm,"
        "radius,circ,state,roi_x0,roi_y0,roi_x1,roi_y1\n";
    (void)!write(csv_fd_, hdr, std::strlen(hdr));
    fsync(csv_fd_);
    log_line("取证: %s（每帧一行，写线程负责落盘）\n", path);
    log_line("取证: 图像目录 %s（PBM/PGM）\n", img_dir_);
    enabled_ = true;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, kThreadStack); // 栈上限 256KB：写线程里不许放大对象
    thread_started_ = (pthread_create(&thread_, &attr, &EvidenceWriter::thread_entry, this) == 0);
    pthread_attr_destroy(&attr);
    if (!thread_started_)
        log_line("取证: 写线程创建失败，取证通道关闭\n");
}

// 清空图像目录里的 f*.pbm / raw*.pgm（只删这两种前缀，别动别人的东西）。
// 失败不致命：写文件时会按名字覆盖，最坏情况是旧图留下一部分 —— 所以失败要打日志。
void EvidenceWriter::clear_images() {
    DIR *d = opendir(img_dir_);
    if (d == nullptr)
        return;
    int removed = 0, failed = 0;
    struct dirent *e;
    while ((e = readdir(d)) != nullptr) {
        const char *n = e->d_name;
        const bool mine = (n[0] == 'f' && std::isdigit(static_cast<unsigned char>(n[1]))) ||
                          (std::strncmp(n, "raw", 3) == 0 && std::isdigit(static_cast<unsigned char>(n[3])));
        if (!mine)
            continue;
        char p[512]; // 176(img_dir_) + 1 + 255(readdir 名字上限) + 1：留够，绝不截断路径
        const int np = std::snprintf(p, sizeof(p), "%s/%s", img_dir_, n);
        if (np <= 0 || np >= static_cast<int>(sizeof(p))) {
            ++failed; // 拼不下就跳过并计数（截断过的路径会删错文件）
            continue;
        }
        if (unlink(p) == 0)
            ++removed;
        else
            ++failed;
    }
    closedir(d);
    if (removed || failed)
        log_line("取证: 图像目录清空 %d 张（失败 %d）\n", removed, failed);
}

EvidenceWriter::~EvidenceWriter() {
    if (thread_started_) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            stop_ = true;
        }
        cv_.notify_all();
        pthread_join(thread_, nullptr);
    }
    if (csv_fd_ >= 0) {
        close(csv_fd_);
        csv_fd_ = -1;
    }
}

void EvidenceWriter::submit_pbm(uint64_t seq, const uint8_t *bin, uint32_t w, uint32_t h,
                                uint32_t stride) {
    if (!enabled_ || bin == nullptr)
        return;
    const uint32_t need = 11 + ((w + 7u) / 8u) * h;
    if (need > kPbmMax) {
        log_line("取证: PBM 太大(%u > %u)，丢弃\n", need, static_cast<unsigned>(kPbmMax));
        return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    const uint32_t idx = (pbm_head_ + pbm_count_) % kPbmSlots;
    if (pbm_count_ == kPbmSlots || pbm_busy_[idx]) {
        ++dropped_; // 队列满：丢这一张，绝不阻塞视觉链路
        return;
    }
    PbmSlot &slot = pbm_[idx]; // 直接在槽里打包，不做任何大尺寸拷贝
    slot.len = pack_pbm(slot.data, bin, w, h, stride);
    std::snprintf(slot.path, sizeof(slot.path), "%s/f%06llu.pbm", img_dir_,
                  static_cast<unsigned long long>(seq));
    ++pbm_count_;
    cv_.notify_one();
}

void EvidenceWriter::submit_pgm(uint64_t seq, const uint8_t *y_plane, uint32_t w, uint32_t h,
                               uint32_t stride) {
    if (!enabled_ || y_plane == nullptr)
        return;
    const uint32_t need = 16 + w * h;
    if (need > kPgmMax) {
        log_line("取证: PGM 太大(%u > %u)，丢弃\n", need, static_cast<unsigned>(kPgmMax));
        return;
    }

    std::lock_guard<std::mutex> lk(mtx_);
    const uint32_t idx = (pgm_head_ + pgm_count_) % kPgmSlots;
    if (pgm_count_ == kPgmSlots || pgm_busy_[idx]) {
        ++dropped_;
        return;
    }
    PgmSlot &slot = pgm_[idx];
    slot.len = pack_pgm(slot.data, y_plane, w, h, stride);
    std::snprintf(slot.path, sizeof(slot.path), "%s/raw%06llu.pgm", img_dir_,
                  static_cast<unsigned long long>(seq));
    ++pgm_count_;
    cv_.notify_one();
}

void EvidenceWriter::submit_csv(const char *line, uint32_t len) {
    if (!enabled_ || line == nullptr || len == 0)
        return;
    std::lock_guard<std::mutex> lk(mtx_);
    if (csv_len_ + len > sizeof(csv_)) {
        ++dropped_; // 极端情况（写线程被卡住很久）：宁可丢行也不阻塞
        return;
    }
    std::memcpy(csv_ + csv_len_, line, len);
    csv_len_ += len;
}

bool EvidenceWriter::write_slot(const char *path, const uint8_t *data, uint32_t len,
                                const char *what) {
    std::FILE *f = std::fopen(path, "wb");
    if (f == nullptr) {
        ++failed_;
        log_line("取证: %s 打不开 %s\n", what, path);
        return false;
    }
    const bool ok = std::fwrite(data, 1, len, f) == len;
    if (std::fclose(f) != 0 || !ok) {
        ++failed_;
        log_line("取证: %s 写失败 %s\n", what, path);
        return false;
    }
    return true;
}

void *EvidenceWriter::thread_entry(void *self) {
    static_cast<EvidenceWriter *>(self)->run();
    return nullptr;
}

// 写线程：只拿索引、直接对着环形槽写文件 —— 全程没有大尺寸局部变量（栈上限 256KB）
void EvidenceWriter::run() {
    for (;;) {
        bool        stop = false;
        bool        have_pbm = false, have_pgm = false;
        uint32_t    pbm_idx = 0, pgm_idx = 0;
        uint32_t    csv_n = 0;

        {
            std::unique_lock<std::mutex> lk(mtx_);
            cv_.wait_for(lk, std::chrono::milliseconds(1000), [this] {
                return stop_ || pbm_count_ > 0 || pgm_count_ > 0;
            });
            stop = stop_;
            if (pbm_count_ > 0) {
                pbm_idx = pbm_head_;
                pbm_busy_[pbm_idx] = true;
                pbm_head_ = (pbm_head_ + 1) % kPbmSlots;
                --pbm_count_;
                have_pbm = true;
            }
            if (pgm_count_ > 0) {
                pgm_idx = pgm_head_;
                pgm_busy_[pgm_idx] = true;
                pgm_head_ = (pgm_head_ + 1) % kPgmSlots;
                --pgm_count_;
                have_pgm = true;
            }
            if (csv_len_ > 0) {
                csv_n = csv_len_;
                std::memcpy(csv_flush_, csv_, csv_n);
                csv_len_ = 0;
            }
        }

        // I/O 只在这条线程：视觉线程只负责打包入队
        const uint64_t t_io = busy_clock_us();
        if (have_pbm) {
            if (write_slot(pbm_[pbm_idx].path, pbm_[pbm_idx].data, pbm_[pbm_idx].len, "PBM"))
                ++written_pbm_;
            std::lock_guard<std::mutex> lk(mtx_);
            pbm_busy_[pbm_idx] = false;
        }
        if (have_pgm) {
            if (write_slot(pgm_[pgm_idx].path, pgm_[pgm_idx].data, pgm_[pgm_idx].len, "PGM"))
                ++written_pgm_;
            std::lock_guard<std::mutex> lk(mtx_);
            pgm_busy_[pgm_idx] = false;
        }
        if (csv_n > 0 && csv_fd_ >= 0) {
            (void)!write(csv_fd_, csv_flush_, csv_n);
            fsync(csv_fd_);
        }
        busy_us_ += busy_clock_us() - t_io; // 这一轮 I/O 实际占了多少 CPU/挂起时间
        if (stop && !have_pbm && !have_pgm && csv_n == 0)
            break;
    }
}

} // namespace dart
