#include <cstddef>
#include <array>
#include <cassert>
#include <limits>

#include "src/control/servo_mapping.hpp"

#include "tests/odr_consumer.hpp"

namespace dart::tests {

class ServoMappingTest final {
public:
    static void run() {
        assert(OdrConsumer::check());
        control::ServoMapping::Config config{};
        config.channels = {{
            {1510.0F, 1.0F, 0.5F},
            {1520.0F, -2.0F, 1.0F},
            {1530.0F, 3.0F, -1.5F},
            {1540.0F, -4.0F, -2.0F},
        }};
        config.max_delta_us = 120.0F;
        const control::ServoMapping mapping{config};
        assert(mapping.valid_config());
        const auto initial = mapping.pulses();
        assert(initial);
        assert((initial->pulse_us == Values{1510.0F, 1520.0F, 1530.0F, 1540.0F}));
        assert(initial->delta_us == Values{});
        assert(initial->saturated == Flags{});

        const detection::Point preset{320.0F, 240.0F};
        detection::Target armor{};
        armor.valid = true;
        armor.center = {340.0F, 220.0F};
        detection::Target light{};
        light.valid = true;
        light.center = {330.0F, 250.0F};
        const auto horizontal = mapping.horizontal_increment(armor, light, preset);
        const auto vertical = mapping.vertical_increment(armor, light, preset);
        assert(horizontal && vertical);
        assert(horizontal->source == detection::TargetKind::kArmor);
        assert(vertical->source == detection::TargetKind::kArmor);
        assert((horizontal->delta_us == Values{20.0F, -40.0F, 60.0F, -80.0F}));
        assert((vertical->delta_us == Values{-10.0F, -20.0F, 30.0F, 40.0F}));
        const auto output = mapping.pulses(horizontal->delta_us, vertical->delta_us);
        assert(output);
        assert((output->pulse_us == Values{1520.0F, 1460.0F, 1620.0F, 1500.0F}));
        assert(output->saturated == Flags{});
        // Identical observations produce offsets, without integrating every call.
        assert(
            mapping.horizontal_increment(armor, light, preset)->delta_us == horizontal->delta_us);

        armor.center = {300.0F, 260.0F};
        assert((mapping.horizontal_increment(armor, light, preset)->delta_us
                == Values{-20.0F, 40.0F, -60.0F, 80.0F}));
        assert((mapping.vertical_increment(armor, light, preset)->delta_us
                == Values{10.0F, 20.0F, -30.0F, -40.0F}));
        armor.center = preset;
        assert(mapping.horizontal_increment(armor, light, preset)->delta_us == Values{});
        assert(mapping.vertical_increment(armor, light, preset)->delta_us == Values{});

        armor.valid = false;
        const auto fallback = mapping.horizontal_increment(armor, light, preset);
        assert(fallback && fallback->source == detection::TargetKind::kLight);
        assert((fallback->delta_us == Values{10.0F, -20.0F, 30.0F, -40.0F}));
        assert((mapping.vertical_increment(armor, light, preset)->delta_us
                == Values{5.0F, 10.0F, -15.0F, -20.0F}));
        light.valid = false;
        const auto missing = mapping.horizontal_increment(armor, light, preset);
        assert(missing && missing->source == detection::TargetKind::kNone);
        assert(missing->delta_us == Values{});
        assert(
            mapping.vertical_increment(armor, light, preset)->source == detection::TargetKind::kNone);
        const auto neutral = mapping.pulses(
            missing->delta_us, mapping.vertical_increment(armor, light, preset)->delta_us);
        assert(neutral && neutral->pulse_us == initial->pulse_us);

        light.valid = true;
        light.center = {}; // The image origin is a valid coordinate.
        assert((mapping.horizontal_increment(armor, light, {})->delta_us == Values{}));
        assert(
            mapping.horizontal_increment(armor, light, {})->source == detection::TargetKind::kLight);

        // Sum before clipping: a large axis contribution can be cancelled by the other.
        const auto combined = mapping.pulses(
            Values{200.0F, -200.0F, 0.25F, 100.0F}, Values{-150.0F, 0.0F, 0.25F, 100.0F});
        assert(combined);
        assert((combined->pulse_us == Values{1560.0F, 1400.0F, 1530.5F, 1660.0F}));
        assert((combined->delta_us == Values{50.0F, -120.0F, 0.5F, 120.0F}));
        assert((combined->saturated == Flags{false, true, false, true}));
        const auto boundaries = mapping.pulses(Values{-120.0F, 120.0F, -120.0F, 120.0F});
        assert(boundaries && boundaries->saturated == Flags{});

        const float nan = std::numeric_limits<float>::quiet_NaN();
        const float infinity = std::numeric_limits<float>::infinity();
        light.center = {nan, 0.0F};
        const auto bad_coordinate = mapping.horizontal_increment(armor, light, preset);
        assert(!bad_coordinate && bad_coordinate.error() == Error::kInvalidCoordinate);
        light.center = {0.0F, infinity};
        assert(!mapping.vertical_increment(armor, light, preset));
        assert(!mapping.horizontal_increment({}, {}, {nan, 0.0F}));
        assert(!mapping.vertical_increment({}, {}, {0.0F, infinity}));
        assert(!mapping.pulses(Values{nan, 0.0F, 0.0F, 0.0F}));
        const auto bad_increment = mapping.pulses({}, Values{0.0F, 0.0F, infinity, 0.0F});
        assert(!bad_increment && bad_increment.error() == Error::kInvalidIncrement);

        armor.valid = true;
        armor.center = preset;
        // An unused, invalid light coordinate must not poison a valid armor observation.
        assert(mapping.horizontal_increment(armor, light, preset));
        armor.center = {std::numeric_limits<float>::max(), 0.0F};
        const auto overflow = mapping.horizontal_increment(armor, {}, preset);
        assert(!overflow && overflow.error() == Error::kInvalidIncrement);
        // Finite inputs can be safely clipped even if their sum exceeds float range.
        const float large = std::numeric_limits<float>::max();
        const auto large_output = mapping.pulses(Values{large, large, large, large});
        assert((large_output && large_output->saturated == Flags{true, true, true, true}));

        for (std::size_t i = 0; i < control::ServoMapping::kChannelCount; ++i) {
            for (const auto bad : std::array<control::ServoMapping::Channel, 8>{{
                     {nan, 1.0F, 1.0F},
                     {infinity, 1.0F, 1.0F},
                     {0.0F, 1.0F, 1.0F},
                     {-1.0F, 1.0F, 1.0F},
                     {119.0F, 1.0F, 1.0F},
                     {120.0F, 1.0F, 1.0F},
                     {1520.0F, infinity, 1.0F},
                     {1520.0F, 1.0F, nan},
                 }}) {
                auto bad_config = config;
                bad_config.channels[i] = bad;
                const control::ServoMapping invalid{bad_config};
                assert(!invalid.valid_config());
                const auto result = invalid.horizontal_increment({}, {}, preset);
                assert(!result && result.error() == Error::kInvalidConfig);
                assert(!invalid.vertical_increment({}, {}, preset));
                assert(!invalid.pulses());
            }
        }

        for (const float limit : {nan, infinity, -infinity, -1.0F}) {
            auto invalid = config;
            invalid.max_delta_us = limit;
            assert(!control::ServoMapping{invalid}.valid_config());
            assert(!control::ServoMapping{invalid}.pulses());
        }
        auto frozen = config;
        frozen.max_delta_us = 0;
        const auto held = control::ServoMapping{frozen}.pulses(Values{100, -100, 0, 1});
        assert(held && held->pulse_us == initial->pulse_us && held->delta_us == Values{});
        assert((held->saturated == Flags{true, true, false, true}));
        auto fractional = config;
        fractional.max_delta_us = 120.5F;
        const auto rounded = control::ServoMapping{fractional}.pulses(Values{200, -200, 120.5F, -120.5F});
        assert((rounded && rounded->delta_us == Values{120.5F, -120.5F, 120.5F, -120.5F}));
        assert((rounded->saturated == Flags{true, true, false, false}));

        // The deployed limit is relative to each independent zero, after adding
        // both axes. Large opposing axis contributions must cancel first.
        control::ServoMapping::Config deployed{};
        deployed.channels = {{{1445, 10, 10}, {1490, -10, 10},
                              {1150, -10, -10}, {1475, 10, -10}}};
        assert(deployed.max_delta_us == 500);
        const control::ServoMapping limited{deployed};
        armor.valid = true;
        armor.center = {420, 140};
        const auto x = limited.horizontal_increment(armor, {}, preset);
        const auto y = limited.vertical_increment(armor, {}, preset);
        const auto diagonal = limited.pulses(x->delta_us, y->delta_us);
        assert((diagonal && diagonal->pulse_us == Values{1445, 990, 1150, 1975}));
        assert((diagonal->delta_us == Values{0, -500, 0, 500}));
        assert((diagonal->saturated == Flags{false, true, false, true}));
        const auto positive = limited.pulses(Values{1000, -1000, -1000, 1000});
        const auto negative = limited.pulses(Values{-1000, 1000, 1000, -1000});
        assert((positive && positive->delta_us == Values{500, -500, -500, 500}));
        assert((negative && negative->delta_us == Values{-500, 500, 500, -500}));
        deployed.channels[0].initial_pulse_us = 1800;
        const auto shifted = control::ServoMapping{deployed}.pulses(Values{1000, 0, 0, 0});
        assert(shifted && shifted->pulse_us[0] == 2300 && shifted->delta_us[0] == 500);

        // Image axes stay independent when the vertical gain signs change.
        control::ServoMapping::Config axes{};
        axes.channels = {{{1445, 1, 1}, {1490, -1, 1},
                          {1150, -1, -1}, {1475, 1, -1}}};
        axes.max_delta_us = 150;
        const control::ServoMapping image_axes{axes};
        armor.center = {340, 240};
        const auto right = image_axes.horizontal_increment(armor, {}, preset);
        assert((right && right->delta_us == Values{20, -20, -20, 20}));
        assert(image_axes.vertical_increment(armor, {}, preset)->delta_us == Values{});
        armor.center = {320, 260};
        assert(image_axes.horizontal_increment(armor, {}, preset)->delta_us == Values{});
        assert((image_axes.vertical_increment(armor, {}, preset)->delta_us
                == Values{20, 20, -20, -20}));
        armor.center = {320, 220};
        assert((image_axes.vertical_increment(armor, {}, preset)->delta_us
                == Values{-20, -20, 20, 20}));
        // A custom aim uses target.y - aim.y, rather than a hardcoded 240.
        armor.center = {100, 100};
        assert((image_axes.vertical_increment(armor, {}, {100, 80})->delta_us
                == Values{20, 20, -20, -20}));
        for (auto& channel : axes.channels)
            channel.vertical_us_per_pixel = -channel.vertical_us_per_pixel;
        const control::ServoMapping reversed_vertical{axes};
        armor.center = {340, 240};
        assert(reversed_vertical.horizontal_increment(armor, {}, preset)->delta_us
               == right->delta_us);
        assert(reversed_vertical.vertical_increment(armor, {}, preset)->delta_us == Values{});
        armor.center = {320, 260};
        assert((reversed_vertical.vertical_increment(armor, {}, preset)->delta_us
                == Values{-20, -20, 20, 20}));

        const control::ServoMapping defaults;
        armor.center = {340.0F, 220.0F};
        assert(defaults.horizontal_increment(armor, {}, preset)->delta_us == Values{});
        assert(defaults.vertical_increment(armor, {}, preset)->delta_us == Values{});
    }

private:
    using Values = control::ServoMapping::PwmValues;
    using Flags = std::array<bool, control::ServoMapping::kChannelCount>;
    using Error = control::ServoMapping::Error;
};

} // namespace dart::tests

int main() { dart::tests::ServoMappingTest::run(); }
