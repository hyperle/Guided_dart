#pragma once

#include <atomic>
#include <cstdint>
#include <expected>
#include <span>

#include "streaming/http.h"
#include "streaming/k230/radio.h"

#include "streaming/interface.hpp"

namespace dart::streaming::k230 {
class Transport final : public ITransport {
public:
    ~Transport() override { close(); }
    std::expected<void, Error>
        open(const Config& config, const std::atomic_bool& stop) noexcept override;
    std::expected<void, Error> upload(
        std::span<const std::uint8_t> jpeg, std::uint64_t sequence, std::uint64_t received_us,
        const std::atomic_bool&) noexcept override;
    void close() noexcept override { dart_http_close(&client_); }

private:
    static int cancelled(void* context) noexcept;
    Config config_{};
    dart_http_client_t client_{-1, 0, nullptr, nullptr, {}, 0};
};

} // namespace dart::streaming::k230
