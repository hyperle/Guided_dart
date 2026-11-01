module;
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <expected>
#include <memory>
#include <span>
#include <string_view>

export module dart.streaming.interface;

export namespace dart::streaming {

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

bool valid_config(const Config& config) noexcept {
    unsigned a, b, c, d;
    char extra;
    const auto password_size = std::strlen(config.password);
    if (std::sscanf(config.host, "%u.%u.%u.%u%c", &a, &b, &c, &d, &extra) != 4 || a == 0 || a >= 224
        || b > 255 || c > 255 || d > 255 || config.port == 0 || config.port > 65535
        || config.width == 0 || config.width > 1920 || config.height == 0 || config.height > 1080
        || config.width % 2 != 0 || config.height % 2 != 0 || config.fps == 0 || config.fps > 120
        || config.quality == 0 || config.quality > 100 || config.connect_timeout_ms == 0
        || config.connect_timeout_ms > 30000 || config.io_timeout_ms == 0
        || config.io_timeout_ms > 30000 || config.token[0] == '\0'
        || (password_size != 0 && (password_size < 8 || password_size > 64)))
        return false;
    if (password_size == 64 && std::strspn(config.password, "0123456789abcdefABCDEF") != 64)
        return false;
    for (const char value : std::string_view{config.token})
        if (value <= ' ' || value >= 127)
            return false;
    return true;
}

std::expected<Config, Error> load_config(const char* path) noexcept {
    if (path == nullptr || path[0] == '\0')
        return std::unexpected(Error{Code::kConfig, EINVAL});
    FILE* file = std::fopen(path, "r");
    if (file == nullptr)
        return std::unexpected(Error{Code::kConfig, errno});
    Config config;
    std::uint32_t seen = 0;
    bool valid = true;
    char line[512];
    constexpr std::string_view keys[] = {
        "ssid",    "password",           "host",         "token", "port", "width", "height", "fps",
        "quality", "connect_timeout_ms", "io_timeout_ms"};
    while (valid && std::fgets(line, sizeof(line), file) != nullptr) {
        std::string_view text{line};
        if (text.size() == sizeof(line) - 1 && text.back() != '\n') {
            valid = false;
            break;
        }
        while (!text.empty() && (text.back() == '\r' || text.back() == '\n'))
            text.remove_suffix(1);
        if (text.empty() || text.front() == '#')
            continue;
        const auto equal = text.find('=');
        const auto key = text.substr(0, equal);
        const auto value = equal == text.npos ? std::string_view{} : text.substr(equal + 1);
        unsigned index = 0;
        while (index < 11 && keys[index] != key)
            ++index;
        if (equal == text.npos || index == 11 || (seen & (1U << index)) != 0) {
            valid = false;
            break;
        }
        seen |= 1U << index;
        if (index < 4) {
            char* targets[] = {config.ssid, config.password, config.host, config.token};
            const std::size_t capacities[] = {
                sizeof(config.ssid), sizeof(config.password), sizeof(config.host),
                sizeof(config.token)};
            if (value.size() >= capacities[index] || value.find('\0') != value.npos) {
                valid = false;
                break;
            }
            std::memcpy(targets[index], value.data(), value.size());
            targets[index][value.size()] = '\0';
        } else {
            if (value.empty() || value.find_first_not_of("0123456789") != value.npos) {
                valid = false;
                break;
            }
            char number[32]{};
            if (value.size() >= sizeof(number)) {
                valid = false;
                break;
            }
            std::memcpy(number, value.data(), value.size());
            errno = 0;
            const auto parsed = std::strtoul(number, nullptr, 10);
            if (errno != 0 || parsed > UINT32_MAX) {
                valid = false;
                break;
            }
            std::uint32_t* targets[] = {&config.port,         &config.width,
                                        &config.height,       &config.fps,
                                        &config.quality,      &config.connect_timeout_ms,
                                        &config.io_timeout_ms};
            *targets[index - 4] = static_cast<std::uint32_t>(parsed);
        }
    }
    valid = valid && !std::ferror(file) && valid_config(config);
    std::fclose(file);
    if (!valid)
        return std::unexpected(Error{Code::kConfig, EINVAL});
    return config;
}

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
