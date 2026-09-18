#include "evidence/evidence_writer.hpp"

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
EvidenceWriter::EvidenceWriter(const char *dir) {
    std::snprintf(dir_, sizeof(dir_), "%s", dir != nullptr ? dir : "");
    if (dir_[0] != '/') { // 路径必须绝对：上一版 dir_ 被冲空后写出过 /f000001.pbm
        log_line("取证: 目录参数非法('%s')，取证通道关闭\n", dir_);
        return;
    }
    if (mkdir(dir_, 0777) != 0 && errno != EEXIST)
        log_line("取证: mkdir %s 失败 errno=%d（继续试开文件）\n", dir_, errno);

    char path[224];
    std::snprintf(path, sizeof(path), "%s/frames.csv", dir_);
    csv_fd_ = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (csv_fd_ < 0) {
        log_line("取证: frames.csv 打不开，取证通道关闭\n");
        return;
    }
    const char *hdr = "seq,mono_ms,src_pts,d_pts_us,exp_us,cx,cy,roi,cost_us,captured_fps,pbm\n";
    (void)!write(csv_fd_, hdr, std::strlen(hdr));
    fsync(csv_fd_);
    log_line("取证: %s（写线程负责落盘）\n", path);
    enabled_ = true;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, kThreadStack); // 栈上限 256KB：写线程里不许放大对象
    thread_started_ = (pthread_create(&thread_, &attr, &EvidenceWriter::thread_entry, this) == 0);
    pthread_attr_destroy(&attr);
    if (!thread_started_)
        log_line("取证: 写线程创建失败，取证通道关闭\n");
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
    std::snprintf(slot.path, sizeof(slot.path), "%s/f%06llu.pbm", dir_,
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
    std::snprintf(slot.path, sizeof(slot.path), "%s/raw%06llu.pgm", dir_,
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
