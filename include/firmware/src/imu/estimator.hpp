#pragma once

#include <cstdint>

#include "imu/sample.hpp"

namespace dart::imu {
class Estimator final {
public:
    struct Config {
        std::uint64_t max_integration_interval_ns = 100'000'000ULL;
        float max_acceleration_m_s2 = 160.0F;
        float max_angular_velocity_rad_s = 35.0F;
        float max_acceleration_step_m_s2 = 80.0F;
        float max_angular_velocity_step_rad_s = 25.0F;
        float seed_gravity_tolerance_m_s2 = 2.0F;
        float seed_max_angular_velocity_rad_s = 0.15F;
    };

    enum class QualityEvent : std::uint32_t {
        kNone = 0U,
        kSampleFlagged = 1U << 0U,
        kNonFinite = 1U << 1U,
        kAccelerationThreshold = 1U << 2U,
        kAngularRateThreshold = 1U << 3U,
        kAccelerationStepThreshold = 1U << 4U,
        kAngularRateStepThreshold = 1U << 5U,
        kTimestampNonMonotonic = 1U << 6U,
        kTimestampGap = 1U << 7U,
        kReferenceNotSeeded = 1U << 8U,
        kSeedNotStationary = 1U << 9U,
        kInvalidReference = 1U << 10U,
    };

    struct Estimate {
        Sample sample{};
        Quaternionf attitude{};
        Vector3f attitude_rpy_rad{};
        Vector3f angular_velocity_rad_s{};   /* Selected reference-frame axes. */
        Vector3f linear_acceleration_m_s2{}; /* Gravity compensated, reference-frame axes. */
        Vector3f velocity_m_s{};
        Vector3f cumulative_rotation_rad{};  /* Unwrapped integral, reference-frame axes. */
        std::uint64_t generation = 0U;
        std::uint32_t quality_events = 0U;
        bool sample_accepted = false;
        bool integrated = false;
        bool has_reference_frame = false;
    };

    struct SeedResult {
        bool accepted = false;
        std::uint32_t quality_events = 0U;
        std::uint64_t generation = 0U;
    };

    Estimator() noexcept;
    explicit Estimator(const Config& config) noexcept;

    Estimate update(const Sample& sample) noexcept;
    SeedResult
        seed(const Sample& stationary_sample, const Quaternionf& sensor_to_reference) noexcept;
    void reset() noexcept;
    Estimate current() const noexcept;
    std::uint64_t generation() const noexcept { return generation_; }

private:
    std::uint32_t assess_sample(const Sample& sample, bool check_steps) const noexcept;
    Estimate make_estimate(
        const Sample& sample, std::uint32_t events, bool accepted, bool integrated) const noexcept;

    Config config_{};
    Sample previous_sample_{};
    Sample latest_sample_{};
    Quaternionf attitude_{};
    Vector3f gravity_reference_m_s2_{};
    Vector3f velocity_m_s_{};
    Vector3f cumulative_rotation_rad_{};
    Vector3f previous_linear_acceleration_m_s2_{};
    std::uint64_t generation_ = 0U;
    bool has_sample_ = false;
    bool has_reference_frame_ = false;

private:
    static constexpr float kGravityMPerS2 = 9.80665F;

    static constexpr float kHalf = 0.5F;

    static Vector3f add(Vector3f a, Vector3f b) noexcept;

    static Vector3f subtract(Vector3f a, Vector3f b) noexcept;

    static Vector3f scale(Vector3f value, float factor) noexcept;

    static Vector3f cross(Vector3f a, Vector3f b) noexcept;

    static float norm(Vector3f value) noexcept;

    static bool finite(Vector3f value) noexcept;

    static float dot(Quaternionf a, Quaternionf b) noexcept;

    static Quaternionf multiply(Quaternionf a, Quaternionf b) noexcept;

    static bool normalize(Quaternionf& q) noexcept;

    static Vector3f rotate(Quaternionf q, Vector3f value) noexcept;

    static Quaternionf rotation_delta(Vector3f omega_rad_s, float seconds) noexcept;

    static Vector3f to_rpy(Quaternionf q) noexcept;

    static std::uint32_t event(QualityEvent value) noexcept;

    static bool finite_sample(const Sample& sample) noexcept;
};

} // namespace dart::imu
