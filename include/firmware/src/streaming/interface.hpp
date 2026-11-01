#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <span>

namespace dart::streaming {
enum class Code : std::uint8_t {
    kNone,
    kConfig,
    kMemory,
    kThread,
    kRadio,
    kConnect,
    kUpload,
    kJpeg,
    kBackend
};
struct Error {
    Code code = Code::kNone;
    int native = 0;
};

struct Config {
    char ssid[33]{};
    char password[65]{};
    char host[16]{}; // Literal IPv4 keeps DNS out of the bounded shutdown path.
    char token[129]{};
    std::uint32_t port = 8080;
    std::uint32_t width = 320;
    std::uint32_t height = 240;
    std::uint32_t fps = 15;
    std::uint32_t quality = 50;
    std::uint32_t connect_timeout_ms = 2000;
    std::uint32_t io_timeout_ms = 2000;
};

bool valid_config(const Config& config) noexcept;

std::expected<Config, Error> load_config(const char* path) noexcept;

class ITransport {
public:
    virtual ~ITransport() = default;
    virtual std::expected<void, Error>
        open(const Config&, const std::atomic_bool& stop) noexcept = 0;
    // Success means a complete HTTP receipt acknowledgment, before video encoding.
    virtual std::expected<void, Error> upload(
        std::span<const std::uint8_t>, std::uint64_t sequence, std::uint64_t received_us,
        const std::atomic_bool& stop) noexcept = 0;
    virtual void close() noexcept = 0;
};
using TransportFactory = std::unique_ptr<ITransport> (*)() noexcept;

} // namespace dart::streaming
