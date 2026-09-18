#pragma once

#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <pthread.h>
#include <stdio.h>

namespace dart {

// 取证落盘：**所有文件 I/O 都在写线程里**，视觉线程只做打包/拷贝入队。
//
// 为什么必须这样（板端实测）：在视觉线程里直接 fopen/fwrite/fclose 一张 28.8KB 的 PBM，
// SD 卡会把它拖住 10~100ms。日志里 pts 间隔 3577/4077 次是 11ms(=90fps)，剩下那几百次
// 全是 22~78ms 的停顿，且都落在"存图那一帧"上 —— 整条链路从 90fps 掉到 45fps。
//
// 两条硬约束（都是踩过的坑）：
//   1) 写线程**栈上不许放大对象**。上一版把 2MB 的 Item 放在写线程栈上，板端直接栈溢出：
//      dir_ 被冲成空串（路径成了 /f000001.pbm）、整机重启两次。现在改成"元素留在环形槽里、
//      写线程只拿索引直接对着槽写文件"，并用 pthread 把写线程栈显式限到 256KB ——
//      将来谁再在写线程里放大对象，主机侧测试会当场炸掉。
//   2) 队列满就丢样本并计数，绝不阻塞视觉链路（与录像链路"识别优先"同一条原则）。
class EvidenceWriter {
public:
    static constexpr uint32_t kPbmSlots = 4;
    static constexpr uint32_t kPgmSlots = 2;
    static constexpr uint32_t kPbmMax = 11 + 4096 * 8;     // 640x360 1bit = 28.8KB，留一倍余量
    static constexpr uint32_t kPgmMax = 16 + 1024 * 1024;  // 640x360 8bit = 230KB；1280x720 也装得下
    static constexpr uint32_t kCsvChunk = 32768;
    static constexpr size_t   kThreadStack = 256 * 1024;

    explicit EvidenceWriter(const char *dir);
    ~EvidenceWriter();

    EvidenceWriter(const EvidenceWriter &) = delete;
    EvidenceWriter &operator=(const EvidenceWriter &) = delete;

    void submit_pbm(uint64_t seq, const uint8_t *bin, uint32_t w, uint32_t h, uint32_t stride);
    void submit_pgm(uint64_t seq, const uint8_t *y, uint32_t w, uint32_t h, uint32_t stride);
    void submit_csv(const char *line, uint32_t len);

    uint64_t written_pbm() const { return written_pbm_; }
    uint64_t written_pgm() const { return written_pgm_; }
    uint64_t dropped() const { return dropped_; }
    uint64_t failed() const { return failed_; }
    // 写线程累计 I/O 忙时（us）：用来判断"每帧一个文件"的 FAT 元数据代价是否值得再优化
    uint64_t busy_us() const { return busy_us_; }
    bool     enabled() const { return enabled_; }

private:
    struct PbmSlot {
        char     path[192];
        uint32_t len = 0;
        uint8_t  data[kPbmMax];
    };
    struct PgmSlot {
        char     path[192];
        uint32_t len = 0;
        uint8_t  data[kPgmMax];
    };

    static void *thread_entry(void *self);
    void run();
    bool write_slot(const char *path, const uint8_t *data, uint32_t len, const char *what);

    pthread_t               thread_{};
    bool                    thread_started_ = false;
    std::mutex              mtx_;
    std::condition_variable cv_;
    bool                    stop_ = false;
    bool                    enabled_ = false;

    PbmSlot  pbm_[kPbmSlots];
    bool     pbm_busy_[kPbmSlots] = {};
    uint32_t pbm_head_ = 0, pbm_count_ = 0;

    PgmSlot  pgm_[kPgmSlots];
    bool     pgm_busy_[kPgmSlots] = {};
    uint32_t pgm_head_ = 0, pgm_count_ = 0;

    char     csv_[kCsvChunk];
    char     csv_flush_[kCsvChunk]; // 写线程的搬运缓冲（成员，不放栈上）
    uint32_t csv_len_ = 0;

    char     dir_[160];
    int      csv_fd_ = -1;

    uint64_t busy_us_ = 0; // 写线程累计 I/O 忙时（us）
    uint64_t written_pbm_ = 0;
    uint64_t written_pgm_ = 0;
    uint64_t dropped_ = 0;
    uint64_t failed_ = 0;
};

} // namespace dart
