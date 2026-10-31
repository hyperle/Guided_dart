module;
#include <expected>

#include "spi/iim42652.h"

export module dart.imu.k230.iim42652;

export import dart.imu.interface;

export namespace dart::imu::k230 {

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
    const dart_iim42652_diagnostic_t& initialization_diagnostic() const noexcept {
        return diagnostic_;
    }

private:
    dart_iim42652_config_t config_{};
    dart_iim42652_t* sensor_ = nullptr;
    dart_iim42652_diagnostic_t diagnostic_{};

private:
    static ErrorCode convert_error(dart_iim42652_status_t status) noexcept {
        switch (status) {
        case DART_IIM42652_ALREADY_INITIALIZED: return ErrorCode::kAlreadyInitialized;
        case DART_IIM42652_NOT_INITIALIZED: return ErrorCode::kNotInitialized;
        case DART_IIM42652_NO_MEMORY: return ErrorCode::kNoMemory;
        case DART_IIM42652_UNEXPECTED_DEVICE_ID: return ErrorCode::kUnexpectedDeviceId;
        case DART_IIM42652_CLOCK_ERROR: return ErrorCode::kClock;
        case DART_IIM42652_READY_TIMEOUT: return ErrorCode::kReadyTimeout;
        case DART_IIM42652_NO_DATA: return ErrorCode::kNoData;
        case DART_IIM42652_INVALID_ARGUMENT: return ErrorCode::kInvalidConfiguration;
        case DART_IIM42652_CONFIGURATION_MISMATCH: return ErrorCode::kInvalidConfiguration;
        case DART_IIM42652_BUS_ERROR:
        case DART_IIM42652_OK: return ErrorCode::kBusTransfer;
        }
        return ErrorCode::kBusTransfer;
    }
};

Iim42652::~Iim42652() { deinitialize(); }

std::expected<void, ErrorCode> Iim42652::initialize() noexcept {
    const auto status = dart_iim42652_init_with_diagnostic(&config_, &sensor_, &diagnostic_);
    if (status != DART_IIM42652_OK)
        return std::unexpected(convert_error(status));
    return {};
}

std::expected<Sample, ErrorCode> Iim42652::read_sample() noexcept {
    dart_iim42652_sample_t c_sample{};
    const auto status = dart_iim42652_read_sample(sensor_, &c_sample);
    if (status != DART_IIM42652_OK)
        return std::unexpected(convert_error(status));

    Sample sample{};
    sample.raw_counts.accelerometer = {
        c_sample.accelerometer[0], c_sample.accelerometer[1], c_sample.accelerometer[2]};
    sample.raw_counts.gyroscope = {
        c_sample.gyroscope[0], c_sample.gyroscope[1], c_sample.gyroscope[2]};
    sample.acceleration_m_s2 = {
        c_sample.acceleration_m_s2[0], c_sample.acceleration_m_s2[1],
        c_sample.acceleration_m_s2[2]};
    sample.angular_velocity_rad_s = {
        c_sample.angular_velocity_rad_s[0], c_sample.angular_velocity_rad_s[1],
        c_sample.angular_velocity_rad_s[2]};
    sample.temperature_c = c_sample.temperature_c;
    sample.timestamp_ns = c_sample.timestamp_ns;
    sample.flags = c_sample.flags;
    return sample;
}

void Iim42652::deinitialize() noexcept { dart_iim42652_deinit(&sensor_); }

} // namespace dart::imu::k230
