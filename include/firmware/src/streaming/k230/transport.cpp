#include "streaming/k230/transport.hpp"

#include <atomic>
#include <cstdint>
#include <expected>
#include <span>

#include "streaming/http.h"
#include "streaming/k230/radio.h"

#include "streaming/interface.hpp"

namespace dart::streaming::k230 {

std::expected<void, Error>
    Transport::open(const Config& config, const std::atomic_bool& stop) noexcept {
    config_ = config;
    int result = dart_wifi_ensure(
        config_.ssid, config_.password, &cancelled, const_cast<std::atomic_bool*>(&stop));
    if (result != 0)
        return std::unexpected(Error{Code::kRadio, result});
    result = dart_http_open(
        &client_, config_.host, static_cast<std::uint16_t>(config_.port),
        config_.connect_timeout_ms, config_.io_timeout_ms, &cancelled,
        const_cast<std::atomic_bool*>(&stop));
    if (result != 0)
        return std::unexpected(Error{Code::kConnect, result});
    return {};
}

std::expected<void, Error> Transport::upload(
    std::span<const std::uint8_t> jpeg, std::uint64_t sequence, std::uint64_t received_us,
    const std::atomic_bool&) noexcept {
    // The peer may request Connection: close after acknowledging a frame.
    if (client_.socket < 0) {
        const int result = dart_http_open(
            &client_, config_.host, static_cast<std::uint16_t>(config_.port),
            config_.connect_timeout_ms, config_.io_timeout_ms, client_.cancelled, client_.context);
        if (result != 0)
            return std::unexpected(Error{Code::kConnect, result});
    }
    const int result =
        dart_http_upload(&client_, config_.token, jpeg.data(), jpeg.size(), sequence, received_us);
    if (result != 0)
        return std::unexpected(Error{Code::kUpload, result});
    return {};
}

int Transport::cancelled(void* context) noexcept {
    return static_cast<std::atomic_bool*>(context)->load() ? 1 : 0;
}

} // namespace dart::streaming::k230
