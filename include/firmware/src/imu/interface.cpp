module;
#include <cstdint>
#include <expected>

export module dart.imu.interface;

export import dart.imu.sample;

export namespace dart::imu {

enum class ErrorCode : std::uint8_t {
    kInvalidConfiguration,
    kAlreadyInitialized,
    kNotInitialized,
    kNoMemory,
    kBusTransfer,
    kUnexpectedDeviceId,
    kClock,
    kReadyTimeout,
    /* -32768 sense data: that sensor has not produced a sample since reset. */
    kNoData,
};

class IImu {
public:
    virtual ~IImu() = default;
    virtual std::expected<Sample, ErrorCode> read_sample() noexcept = 0;
};

} // namespace dart::imu
