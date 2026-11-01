#pragma once

#include <chrono>
#include <cstdint>
#include <signal.h>

namespace dart::platform {
/* Process lifetime is independent of individual device availability. */
class RunControl final {
public:
    explicit RunControl(std::uint32_t seconds) noexcept;
    ~RunControl();
    RunControl(const RunControl&) = delete;
    RunControl& operator=(const RunControl&) = delete;

    bool finished() const noexcept;
    const char* reason() const noexcept { return stopped_ != 0 ? "signal" : "deadline"; }

private:
    static void handle(int) noexcept { stopped_ = 1; }
    inline static volatile sig_atomic_t stopped_ = 0;
    bool bounded_;
    std::chrono::steady_clock::time_point deadline_;
    struct sigaction old_int_{}, old_term_{};
};

} // namespace dart::platform
