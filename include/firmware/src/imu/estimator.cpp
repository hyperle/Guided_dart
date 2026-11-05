#include "imu/estimator.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

#include "imu/sample.hpp"

namespace dart::imu {

Vector3f Estimator::add(Vector3f a, Vector3f b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vector3f Estimator::subtract(Vector3f a, Vector3f b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vector3f Estimator::scale(Vector3f value, float factor) noexcept {
    return {value.x * factor, value.y * factor, value.z * factor};
}

Vector3f Estimator::cross(Vector3f a, Vector3f b) noexcept {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

float Estimator::norm(Vector3f value) noexcept {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

bool Estimator::finite(Vector3f value) noexcept {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

float Estimator::dot(Quaternionf a, Quaternionf b) noexcept {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

Quaternionf Estimator::multiply(Quaternionf a, Quaternionf b) noexcept {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

bool Estimator::normalize(Quaternionf& q) noexcept {
    const float norm_squared = dot(q, q);
    if (!std::isfinite(norm_squared) || norm_squared < 1.0e-12F)
        return false;
    const float inverse_norm = 1.0F / std::sqrt(norm_squared);
    q.w *= inverse_norm;
    q.x *= inverse_norm;
    q.y *= inverse_norm;
    q.z *= inverse_norm;
    return true;
}

Vector3f Estimator::rotate(Quaternionf q, Vector3f value) noexcept {
    const Vector3f imaginary{q.x, q.y, q.z};
    const Vector3f twice_cross = scale(cross(imaginary, value), 2.0F);
    return add(value, add(scale(twice_cross, q.w), cross(imaginary, twice_cross)));
}

Quaternionf Estimator::rotation_delta(Vector3f omega_rad_s, float seconds) noexcept {
    const Vector3f half_angle = scale(omega_rad_s, kHalf * seconds);
    const float angle = norm(half_angle);
    if (angle < 1.0e-6F) {
        Quaternionf result{1.0F, half_angle.x, half_angle.y, half_angle.z};
        (void)normalize(result);
        return result;
    }
    const float vector_scale = std::sin(angle) / angle;
    return {
        std::cos(angle), half_angle.x * vector_scale, half_angle.y * vector_scale,
        half_angle.z * vector_scale};
}

Vector3f Estimator::to_rpy(Quaternionf q) noexcept {
    constexpr float kHalfPi = 1.57079632679489661923F;
    const float sin_roll_cos_pitch = 2.0F * (q.w * q.x + q.y * q.z);
    const float cos_roll_cos_pitch = 1.0F - 2.0F * (q.x * q.x + q.y * q.y);
    const float sin_pitch = 2.0F * (q.w * q.y - q.z * q.x);
    const float bounded_pitch = sin_pitch < -1.0F ? -1.0F : sin_pitch > 1.0F ? 1.0F : sin_pitch;
    const float sin_yaw_cos_pitch = 2.0F * (q.w * q.z + q.x * q.y);
    const float cos_yaw_cos_pitch = 1.0F - 2.0F * (q.y * q.y + q.z * q.z);
    const float pitch = std::abs(bounded_pitch) >= 1.0F ? std::copysign(kHalfPi, bounded_pitch)
                                                        : std::asin(bounded_pitch);
    return {
        std::atan2(sin_roll_cos_pitch, cos_roll_cos_pitch), pitch,
        std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)};
}

std::uint32_t Estimator::event(QualityEvent value) noexcept {
    return static_cast<std::uint32_t>(value);
}

bool Estimator::finite_sample(const Sample& sample) noexcept {
    return finite(sample.acceleration_m_s2) && finite(sample.angular_velocity_rad_s)
        && std::isfinite(sample.temperature_c);
}

Estimator::Estimator() noexcept
    : Estimator(Config{}) {}

Estimator::Estimator(const Config& config) noexcept
    : config_(config) {}

std::uint32_t Estimator::assess_sample(const Sample& sample, bool check_steps) const noexcept {
    std::uint32_t events = 0U;
    if (sample.flags != 0U)
        events |= event(QualityEvent::kSampleFlagged);
    if (!finite_sample(sample))
        events |= event(QualityEvent::kNonFinite);

    if (finite(sample.acceleration_m_s2)
        && norm(sample.acceleration_m_s2) > config_.max_acceleration_m_s2) {
        events |= event(QualityEvent::kAccelerationThreshold);
    }
    if (finite(sample.angular_velocity_rad_s)
        && norm(sample.angular_velocity_rad_s) > config_.max_angular_velocity_rad_s) {
        events |= event(QualityEvent::kAngularRateThreshold);
    }

    if (check_steps && has_sample_ && finite_sample(sample)) {
        if (norm(subtract(sample.acceleration_m_s2, previous_sample_.acceleration_m_s2))
            > config_.max_acceleration_step_m_s2) {
            events |= event(QualityEvent::kAccelerationStepThreshold);
        }
        if (norm(subtract(sample.angular_velocity_rad_s, previous_sample_.angular_velocity_rad_s))
            > config_.max_angular_velocity_step_rad_s) {
            events |= event(QualityEvent::kAngularRateStepThreshold);
        }
    }
    return events;
}

Estimator::Estimate Estimator::make_estimate(
    const Sample& sample, std::uint32_t events, bool accepted, bool integrated) const noexcept {
    Estimate result{};
    result.sample = sample;
    result.attitude = attitude_;
    result.attitude_rpy_rad = to_rpy(attitude_);
    result.angular_velocity_rad_s = has_reference_frame_
                                      ? rotate(attitude_, sample.angular_velocity_rad_s)
                                      : sample.angular_velocity_rad_s;
    result.linear_acceleration_m_s2 =
        has_reference_frame_
            ? subtract(rotate(attitude_, sample.acceleration_m_s2), gravity_reference_m_s2_)
            : Vector3f{};
    result.velocity_m_s = velocity_m_s_;
    result.cumulative_rotation_rad = cumulative_rotation_rad_;
    result.generation = generation_;
    result.quality_events = events;
    result.sample_accepted = accepted;
    result.integrated = integrated;
    result.has_reference_frame = has_reference_frame_;
    return result;
}

Estimator::Estimate Estimator::update(const Sample& sample) noexcept {
    std::uint32_t events = assess_sample(sample, has_sample_);
    const std::uint32_t rejection_mask =
        event(QualityEvent::kSampleFlagged) | event(QualityEvent::kNonFinite)
        | event(QualityEvent::kAccelerationThreshold) | event(QualityEvent::kAngularRateThreshold)
        | event(QualityEvent::kAccelerationStepThreshold)
        | event(QualityEvent::kAngularRateStepThreshold);
    if ((events & rejection_mask) != 0U) {
        return make_estimate(sample, events, false, false);
    }

    if (has_sample_ && sample.timestamp_ns <= previous_sample_.timestamp_ns) {
        events |= event(QualityEvent::kTimestampNonMonotonic);
        return make_estimate(sample, events, false, false);
    }

    if (!has_reference_frame_) {
        previous_sample_ = sample;
        latest_sample_ = sample;
        has_sample_ = true;
        events |= event(QualityEvent::kReferenceNotSeeded);
        return make_estimate(sample, events, true, false);
    }

    bool integrated = false;
    if (has_sample_) {
        const std::uint64_t dt_ns = sample.timestamp_ns - previous_sample_.timestamp_ns;
        if (dt_ns > config_.max_integration_interval_ns) {
            events |= event(QualityEvent::kTimestampGap);
            previous_linear_acceleration_m_s2_ =
                subtract(rotate(attitude_, sample.acceleration_m_s2), gravity_reference_m_s2_);
        } else {
            const float dt = static_cast<float>(dt_ns) * 1.0e-9F;
            const Vector3f omega_mid_body = scale(
                add(previous_sample_.angular_velocity_rad_s, sample.angular_velocity_rad_s), kHalf);
            const Quaternionf midpoint =
                multiply(attitude_, rotation_delta(omega_mid_body, dt * kHalf));
            const Vector3f omega_mid_reference = rotate(midpoint, omega_mid_body);
            cumulative_rotation_rad_ =
                add(cumulative_rotation_rad_, scale(omega_mid_reference, dt));

            attitude_ = multiply(attitude_, rotation_delta(omega_mid_body, dt));
            (void)normalize(attitude_);

            const Vector3f linear_acceleration =
                subtract(rotate(attitude_, sample.acceleration_m_s2), gravity_reference_m_s2_);
            const Vector3f mean_acceleration =
                scale(add(previous_linear_acceleration_m_s2_, linear_acceleration), kHalf);
            velocity_m_s_ = add(velocity_m_s_, scale(mean_acceleration, dt));
            previous_linear_acceleration_m_s2_ = linear_acceleration;
            integrated = true;
        }
    }

    /* Rebase after a timestamp gap without integrating across the missing interval. */
    previous_sample_ = sample;
    latest_sample_ = sample;
    has_sample_ = true;
    return make_estimate(sample, events, true, integrated);
}

Estimator::SeedResult
    Estimator::seed(const Sample& sample, const Quaternionf& sensor_to_reference) noexcept {
    SeedResult result{};
    result.generation = generation_;
    result.quality_events = assess_sample(sample, false);

    Quaternionf normalized_reference = sensor_to_reference;
    const float quaternion_norm_squared = dot(normalized_reference, normalized_reference);
    if (!std::isfinite(quaternion_norm_squared) || quaternion_norm_squared < 1.0e-12F) {
        result.quality_events |= event(QualityEvent::kInvalidReference);
        return result;
    }
    (void)normalize(normalized_reference);

    const float acceleration_norm = norm(sample.acceleration_m_s2);
    if (result.quality_events != 0U
        || std::abs(acceleration_norm - kGravityMPerS2) > config_.seed_gravity_tolerance_m_s2
        || norm(sample.angular_velocity_rad_s) > config_.seed_max_angular_velocity_rad_s) {
        result.quality_events |= event(QualityEvent::kSeedNotStationary);
        return result;
    }

    /* Seeding with the current sample just returned by update() is valid. */
    if (has_sample_ && sample.timestamp_ns < previous_sample_.timestamp_ns) {
        result.quality_events |= event(QualityEvent::kTimestampNonMonotonic);
        return result;
    }

    attitude_ = normalized_reference;
    gravity_reference_m_s2_ = rotate(attitude_, sample.acceleration_m_s2);
    velocity_m_s_ = {};
    cumulative_rotation_rad_ = {};
    previous_linear_acceleration_m_s2_ = {};
    previous_sample_ = sample;
    latest_sample_ = sample;
    has_sample_ = true;
    has_reference_frame_ = true;
    if (generation_ != std::numeric_limits<std::uint64_t>::max())
        ++generation_;

    result.accepted = true;
    result.generation = generation_;
    result.quality_events = 0U;
    return result;
}

void Estimator::reset() noexcept {
    attitude_ = {};
    gravity_reference_m_s2_ = {};
    velocity_m_s_ = {};
    cumulative_rotation_rad_ = {};
    previous_linear_acceleration_m_s2_ = {};
    previous_sample_ = {};
    latest_sample_ = {};
    has_sample_ = false;
    has_reference_frame_ = false;
    if (generation_ != std::numeric_limits<std::uint64_t>::max())
        ++generation_;
}

Estimator::Estimate Estimator::current() const noexcept {
    const std::uint32_t events =
        has_reference_frame_ ? 0U : event(QualityEvent::kReferenceNotSeeded);
    return make_estimate(latest_sample_, events, has_sample_, false);
}

} // namespace dart::imu
