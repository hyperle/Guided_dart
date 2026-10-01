#pragma once

#include "detection/measure/roi_measure.hpp"

namespace dart::detection {

// 装甲板只需要“给我窗口内的 blob”，不需要知道 ROI 测量器如何实现。
class IBlobSource {
public:
    using BlobInfo = RunLengthMeasurer::BlobInfo;

    virtual ~IBlobSource() = default;
    virtual uint32_t collect(const GrayFrame &frame, const RoiWindow &window, uint32_t min_area,
                             BlobInfo *out, uint32_t cap) = 0;
};

// 默认适配器：复用已有 RLE/并查集实现，保持板端行为不变。
class RunLengthBlobSource final : public IBlobSource {
public:
    explicit RunLengthBlobSource(RunLengthMeasurer &measurer) : measurer_(measurer) {}

    uint32_t collect(const GrayFrame &frame, const RoiWindow &window, uint32_t min_area,
                     BlobInfo *out, uint32_t cap) override {
        return measurer_.collect(frame, window, min_area, out, cap);
    }

private:
    RunLengthMeasurer &measurer_;
};

} // namespace dart::detection
