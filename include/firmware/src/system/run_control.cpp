#include "system/run_control.hpp"

#include <chrono>
#include <cstdint>
#include <signal.h>

namespace dart::platform {

RunControl::RunControl(std::uint32_t seconds) noexcept
    : bounded_(seconds != 0)
    , deadline_(std::chrono::steady_clock::now() + std::chrono::seconds{seconds}) {
    stopped_ = 0;
    struct sigaction action{};
    action.sa_handler = &RunControl::handle;
    (void)sigaction(SIGINT, &action, &old_int_);
    (void)sigaction(SIGTERM, &action, &old_term_);
}

RunControl::~RunControl() {
    (void)sigaction(SIGINT, &old_int_, nullptr);
    (void)sigaction(SIGTERM, &old_term_, nullptr);
}

bool RunControl::finished() const noexcept {
    return stopped_ != 0 || (bounded_ && std::chrono::steady_clock::now() >= deadline_);
}

} // namespace dart::platform
