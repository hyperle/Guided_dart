#pragma once

#include <cstddef>
#include <cstdint>

namespace dart {

// ---------------------------------------------------------------------------
// K230D 内存规划（实际部署板卡是 K230D，当前测试板 K230 容量更大，但一律按 K230D 约束设计）
//
//   DDR 128MB
//   ├── RT-Smart 内核 + 用户态   68MB（其中用户堆仅 16MB）
//   └── MMZ                      60MB  ← 硬件缓冲只能从这里出，这是真正的天花板
//
// 本程序占 MMZ 的账（640x360 单通道，stride 32 对齐后 640）：
//   VICAP dev 缓冲   1280x720x2B x 6  = 10.5MB
//   VICAP chn 缓冲    640x360x1.5B x 8 = 2.76MB
//   帧池             640x360x1.5B x 16 = 5.53MB   ← 二值化帧与 VENC 输入是同一块内存（零拷贝）
//   VENC 内部 + 输出池               ≈ 4MB
//   合计 ≈ 22.8MB = MMZ 的 38%，余量留给 KPU 模型与其他模块（K230D 上模型动辄 10MB+）
//
// 帧池上限由编译期断言守住（见文件末尾），改 frame_slots 不会悄悄吃光 MMZ。
// ---------------------------------------------------------------------------

struct Config {
    // 传感器模式。gc2093/CSI2 只有四档：1080p@30、1080p@60、1280x960@90、1280x720@90。
    // 选 1280x720@90：90fps 只有 1280 宽这两档，帧率高 3 倍、延迟低 3 倍。
    uint32_t sensor_width = 1280;
    uint32_t sensor_height = 720;
    uint32_t sensor_fps = 90;
    int      csi = 2; // 庐山派摄像头挂在 CSI2

    // 识别与录像统一画幅：单通道 CHN0（双通道方案在板端被证伪）。
    // 1280x720 -> 640x360 是精确 2:1，不产生几何畸变。
    uint32_t width = 640;
    uint32_t height = 360;

    uint8_t threshold = 128; // 二值化阈值

    // 固定曝光（微秒）。>0 表示关掉 AE 用这个值；填 0 则交回 ISP 自动曝光。
    // 绿灯识别必须固定曝光，否则 AE 会把阈值化基准带着一起漂。这里是唯一入口，
    // 不做命令行开关、也不做自动扫描（按使用方要求：曝光由这里定死）。
    uint32_t exposure_us = 250;

    uint32_t frame_slots = 16;  // 帧池槽数
    uint32_t cap_buffers = 6;   // VICAP dev 缓冲数（老应用板端验证过的值）
    uint32_t chn_buffers = 8;   // VICAP chn 缓冲数
    // 录像侧块数（--mem-slim 会调小）：MMZ 是 VICAP/VB/VENC 内部缓冲共用的，
    // 编码器"收帧不吐码流"很可能就是被我们的池饿死，用这两个数来让位。
    uint32_t rec_in_blocks = 6;  // 录像输入块（老工程 BLK_N=6）
    uint32_t rec_out_blocks = 8; // VENC 码流输出池块数（老工程 OUT_BUF_N=8）

    // 录像（黑白 H.264，左上角烧入关键数据）
    const char *out_dir = "/sdcard/dart";
    uint32_t    bitrate_kbps = 2000;
    uint32_t    gop = 90;      // 1 秒一个 I 帧
    uint32_t    venc_chn = 0;  // VENC 通道号（--venc-chn 可换；K230 上别的子系统可能占 0）
    // 不依赖编码器的取证通道：每 N 帧存一张 PBM 二值图 + 每帧识别结果写 CSV
    uint32_t    pbm_every = 0;      // --pbm N（0=关）
    uint32_t    raw_every = 0;      // --raw N：每 N 帧存一张原始 Y 平面 PGM（标定阈值/曝光用）
    bool        csv = false;        // --csv
    char        venc_mode = 0;      // --venc-mode: 0=多候选自检, 'm'=只试MJPEG, 'h'=只试H264, 'i'=H264+intbuf
    bool        venc_probe = false; // --venc-probe：正式建链前把候选编码配置逐个试编
    uint32_t    venc_intbuf_mb = 0; // >0 时在 create_chn 后调用 kd_mpi_venc_set_intbuf_size
                                    // （文档原话：configure internal memory used for vpu firmware）

    // 限时运行秒数（--seconds N，0 = 一直跑）。launcher 清单里前台项顺序执行，
    // 靠它就能"一次上电连测多档传感器模式"。
    uint32_t run_seconds = 0;

    // 诊断用（--vision N）：先不碰 VENC、纯视觉跑 N 秒拿到真实帧率，再进录像。
    // 录像那条链路在板端会把整机冻住，所以关键数据必须抢在它前面落盘。
    uint32_t vision_seconds = 0;
    bool     no_record = false; // --no-record：完全不启动录像（不碰 VENC）
};

// VB 块按 YUV420SP 计（二值图只占 Y 平面，UV 恒为 128 —— 这样帧池内存能直接喂 VENC）
constexpr size_t vb_block_bytes(uint32_t width, uint32_t height) {
    return static_cast<size_t>((width + 31u) & ~31u) * height * 3 / 2;
}

constexpr uint32_t kFrameSlotsCeiling = 64; // 64 x 345KB = 22MB，仍在 MMZ 余量内
inline constexpr Config kDefaultConfig{};

static_assert(vb_block_bytes(kDefaultConfig.width, kDefaultConfig.height) * kFrameSlotsCeiling < (24u << 20),
              "帧池上限超过 24MB：K230D 的 MMZ 只有 60MB，还要留给 KPU 模型");

} // namespace dart
