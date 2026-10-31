#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <limits>
#include "drv_fpioa.h"
#include "drv_pwm.h"

import dart.pwm.k230.output;

import dart.pwm.interface;

namespace dart::tests {

struct PwmHal {
    inline static std::array<std::uint32_t, 72> pins{};
    inline static std::array<std::uint32_t, 6> pulses{};
    inline static unsigned enabled = 0, routed = 0, frequencies = 0, cleanup = 0;
    inline static unsigned deinitializations = 0, writes = 0;
    inline static const char* failure = "none";
    inline static bool failed = false, initialized = false, operational = false;
    inline static constexpr std::array<int, 4> channels{1, 2, 3, 5};
    inline static constexpr std::array<int, 4> pin_numbers{61, 7, 8, 59};

    static bool fail(const char* operation) {
        if (std::strcmp(failure, operation) != 0 || failed)
            return false;
        failed = true;
        cleanup = 0;
        return true;
    }
    static unsigned bit(int channel) {
        assert(channel == 1 || channel == 2 || channel == 3 || channel == 5);
        return 1u << channel;
    }
    static unsigned index(int pin) {
        for (unsigned i = 0; i < pin_numbers.size(); ++i)
            if (pin == pin_numbers[i]) return i;
        assert(false);
        return 0;
    }
    static void reset(const char* scenario) {
        pins.fill(1u << 9); // Preserve voltage bank selection.
        pulses.fill(0);
        enabled = routed = frequencies = cleanup = deinitializations = writes = 0;
        failure = scenario;
        failed = initialized = operational = false;
    }
};

extern "C" int drv_pwm_init() {
    if (PwmHal::fail("init")) return -7;
    PwmHal::initialized = true;
    return 0;
}
extern "C" int drv_pwm_deinit() {
    assert(PwmHal::initialized);
    ++PwmHal::deinitializations;
    PwmHal::initialized = false;
    return PwmHal::fail("deinit") ? -7 : 0;
}
extern "C" int drv_pwm_disable(int channel) {
    const auto bit = PwmHal::bit(channel);
    PwmHal::cleanup |= bit;
    PwmHal::enabled &= ~bit;
    if (channel == 2 && PwmHal::operational && PwmHal::fail("disable")) return -7;
    return 0;
}
extern "C" int drv_fpioa_get_pin_cfg(int pin, std::uint32_t* value) {
    PwmHal::index(pin);
    if (pin == 7 && PwmHal::fail("pin_read")) return -7;
    *value = PwmHal::pins[pin];
    return 0;
}
extern "C" int drv_fpioa_set_pin_cfg(int pin, std::uint32_t value) {
    PwmHal::index(pin);
    assert(value & (1u << 9));
    PwmHal::pins[pin] = value;
    return 0;
}
extern "C" int drv_fpioa_set_pin_func(int pin, fpioa_func_t function) {
    assert(!PwmHal::enabled && !PwmHal::frequencies);
    const auto index = PwmHal::index(pin);
    assert(function == PWM0 + PwmHal::channels[index]);
    if (pin == 7 && PwmHal::fail("route")) return -7;
    PwmHal::pins[pin] |= 1u << 7; // Output enable; the adapter must also set IE.
    PwmHal::routed |= 1u << index;
    return 0;
}
extern "C" int drv_fpioa_get_pin_func(int pin, fpioa_func_t* function) {
    const auto index = PwmHal::index(pin);
    *function = static_cast<fpioa_func_t>(PWM0 + PwmHal::channels[index]);
    if (pin == 7 && PwmHal::fail("pin_mismatch")) *function = PWM0;
    return 0;
}
extern "C" int drv_pwm_set_freq(int channel, std::uint32_t hz) {
    assert(PwmHal::routed == 15 && !PwmHal::enabled && hz == 333);
    assert(channel == (PwmHal::frequencies == 0 ? 1 : 3));
    if (channel == 3 && PwmHal::fail("frequency")) return -7;
    ++PwmHal::frequencies;
    return 0;
}
extern "C" int drv_pwm_set_duty_ns(int channel, std::uint32_t ns) {
    PwmHal::bit(channel);
    assert(PwmHal::frequencies == 2 && ns > 0 && ns <= pwm::kPeriodNs);
    if (channel == 2 && PwmHal::fail(PwmHal::operational ? "write" : "initial")) return -7;
    ++PwmHal::writes;
    PwmHal::pulses[channel] = ns;
    return 0;
}
extern "C" int drv_pwm_enable(int channel) {
    for (const auto output : PwmHal::channels)
        assert(PwmHal::pulses[output] != 0);
    if (channel == 3 && PwmHal::fail("enable")) return -7;
    PwmHal::enabled |= PwmHal::bit(channel);
    return 0;
}
extern "C" int drv_pwm_get_freq(int channel, std::uint32_t* hz) {
    assert(PwmHal::enabled & PwmHal::bit(channel));
    *hz = 333;
    if (channel == 2 && PwmHal::fail("readback")) *hz = 332;
    return 0;
}
extern "C" int drv_pwm_get_duty_ns(int channel, std::uint32_t* ns) {
    assert(PwmHal::enabled & PwmHal::bit(channel));
    *ns = PwmHal::pulses[channel];
    return 0;
}

class PwmOutputTest final {
public:
    static void run() {
        const pwm::Pulses initial{1510.25F, 1520.5F, 1530.75F, 1540};
        for (const auto* scenario : {"none", "init", "pin_read", "route", "pin_mismatch",
                                    "frequency", "initial", "enable", "readback",
                                    "write", "disable", "deinit"}) {
            PwmHal::reset(scenario);
            {
                pwm::k230::Output output;
                const auto started = output.start(initial);
                if (std::strcmp(scenario, "none") == 0 || std::strcmp(scenario, "write") == 0
                    || std::strcmp(scenario, "disable") == 0 || std::strcmp(scenario, "deinit") == 0) {
                    assert(started && PwmHal::enabled == 46);
                    assert(PwmHal::pulses[1] == 1510250 && PwmHal::pulses[2] == 1520500);
                    PwmHal::operational = true;
                    const auto written = output.write({1420, 1500, 1600, 1620});
                    assert(written.has_value() == (std::strcmp(scenario, "write") != 0));
                    const auto stopped = output.stop();
                    assert(stopped.has_value() == (std::strcmp(scenario, "disable") != 0
                                                  && std::strcmp(scenario, "deinit") != 0));
                } else {
                    assert(!started);
                }
                assert(output.stop()); // No duplicate HAL cleanup on repeated stop/destruction.
            }
            assert(PwmHal::enabled == 0 && !PwmHal::initialized);
            if (std::strcmp(scenario, "init") != 0) {
                assert(PwmHal::deinitializations == 1);
                if (std::strcmp(scenario, "disable") != 0 && std::strcmp(scenario, "deinit") != 0)
                    assert(PwmHal::cleanup == 46); // All four outputs attempted after failure.
            }
        }
        PwmHal::reset("none");
        pwm::k230::Output output;
        assert(!output.start({0, 1520, 1520, 1520}));
        assert(!PwmHal::initialized && PwmHal::writes == 0);
        assert(output.start(initial));
        const auto writes = PwmHal::writes;
        assert(!output.write({1500, 1500, 1500, std::numeric_limits<float>::infinity()}));
        assert(PwmHal::writes == writes && PwmHal::enabled == 0);
    }
};

} // namespace dart::tests

int main() { dart::tests::PwmOutputTest::run(); }
