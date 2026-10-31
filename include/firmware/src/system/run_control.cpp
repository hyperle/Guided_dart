module;
#include <chrono>
#include <cstdint>
#include <signal.h>

export module dart.system.run_control;



export namespace dart::platform {

/* Process lifetime is independent of individual device availability. */
class RunControl final {
public:
    explicit RunControl(std::uint32_t seconds) noexcept
        : bounded_(seconds != 0)
        , deadline_(std::chrono::steady_clock::now() + std::chrono::seconds{seconds}) {
        stopped_ = 0;
        struct sigaction action{};
        action.sa_handler = &RunControl::handle;
        (void)sigaction(SIGINT, &action, &old_int_);
        (void)sigaction(SIGTERM, &action, &old_term_);
    }
    ~RunControl() {
        (void)sigaction(SIGINT, &old_int_, nullptr);
        (void)sigaction(SIGTERM, &old_term_, nullptr);
    }
    RunControl(const RunControl&) = delete;
    RunControl& operator=(const RunControl&) = delete;

    bool finished() const noexcept {
        return stopped_ != 0 || (bounded_ && std::chrono::steady_clock::now() >= deadline_);
    }
    const char* reason() const noexcept { return stopped_ != 0 ? "signal" : "deadline"; }

private:
    static void handle(int) noexcept { stopped_ = 1; }
    inline static volatile sig_atomic_t stopped_ = 0;
    bool bounded_;
    std::chrono::steady_clock::time_point deadline_;
    struct sigaction old_int_{}, old_term_{};
};

} // namespace dart::platform
