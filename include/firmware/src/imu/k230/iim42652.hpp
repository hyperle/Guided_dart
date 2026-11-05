#pragma once

#include <expected>

#include "spi/iim42652.h"

#include "imu/interface.hpp"

namespace dart::imu::k230 {
class Iim42652 final : public IImu {
public:
    explicit Iim42652(const dart_iim42652_config_t& config) noexcept
        : config_(config) {}
    ~Iim42652() override;

    Iim42652(const Iim42652&) = delete;
    Iim42652& operator=(const Iim42652&) = delete;
    Iim42652(Iim42652&&) = delete;
    Iim42652& operator=(Iim42652&&) = delete;

    std::expected<void, ErrorCode> initialize() noexcept;
    std::expected<Sample, ErrorCode> read_sample() noexcept override;
    void deinitialize() noexcept;
    const dart_iim42652_diagnostic_t& initialization_diagnostic() const noexcept;

private:
    dart_iim42652_config_t config_{};
    dart_iim42652_t* sensor_ = nullptr;
    dart_iim42652_diagnostic_t diagnostic_{};

private:
    static ErrorCode convert_error(dart_iim42652_status_t status) noexcept;
};

} // namespace dart::imu::k230
