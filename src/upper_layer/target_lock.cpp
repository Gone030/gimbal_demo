#include "gimbal_mani/arm_upper/target_lock.hpp"

#include <algorithm>
#include <cmath>

namespace gimbal_mani::arm_upper
{

TargetLock::TargetLock(const TargetLockParams &params)
    : params_(params)
{
}

void TargetLock::setParams(const TargetLockParams &params)
{
    params_ = params;
}

void TargetLock::reset()
{
    buffer_.clear();
}

void TargetLock::pushSample(const TargetSample &sample)
{
    buffer_.push_back(sample);
    while (static_cast<int>(buffer_.size()) > params_.min_samples)
    {
        buffer_.pop_front();
    }
}

size_t TargetLock::sampleCount() const
{
    return buffer_.size();
}

bool TargetLock::empty() const
{
    return buffer_.empty();
}

std::optional<LockedTarget> TargetLock::tryLock(const TargetVector &fallback_axis) const
{
    if (static_cast<int>(buffer_.size()) < params_.min_samples)
    {
        return std::nullopt;
    }

    const auto mean = meanLockedTarget(fallback_axis);
    const double max_dev = maxLockedTargetDeviation(mean);
    if (max_dev > params_.pos_tol_m)
    {
        return std::nullopt;
    }
    const double max_bearing_dev = maxLockedBearingDeviation(mean);
    if (max_bearing_dev > params_.bearing_tol_rad)
    {
        return std::nullopt;
    }

    LockedTarget locked;
    locked.object_point = {mean.x, mean.y, mean.z};
    locked.approach_axis = normalizedVector(
        {mean.approach_x, mean.approach_y, mean.approach_z},
        fallback_axis);
    locked.object_yaw = mean.object_yaw;
    locked.max_target_deviation_m = max_dev;
    locked.max_bearing_deviation_rad = max_bearing_dev;
    return locked;
}

double TargetLock::currentMaxTargetDeviation(const TargetVector &fallback_axis) const
{
    if (buffer_.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return maxLockedTargetDeviation(meanLockedTarget(fallback_axis));
}

double TargetLock::currentMaxBearingDeviation(const TargetVector &fallback_axis) const
{
    if (buffer_.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return maxLockedBearingDeviation(meanLockedTarget(fallback_axis));
}

TargetSample TargetLock::meanLockedTarget(const TargetVector &fallback_axis) const
{
    TargetSample m;
    if (buffer_.empty())
    {
        return m;
    }

    double yaw_c = 0.0;
    double yaw_s = 0.0;
    double approach_x = 0.0;
    double approach_y = 0.0;
    double approach_z = 0.0;
    for (const auto &p : buffer_)
    {
        m.x += p.x;
        m.y += p.y;
        m.z += p.z;
        approach_x += p.approach_x;
        approach_y += p.approach_y;
        approach_z += p.approach_z;
        yaw_c += std::cos(p.object_yaw);
        yaw_s += std::sin(p.object_yaw);
    }

    const double n = static_cast<double>(buffer_.size());
    m.x /= n;
    m.y /= n;
    m.z /= n;
    const auto approach = normalizedVector({approach_x / n, approach_y / n, approach_z / n},
                                           fallback_axis);
    m.approach_x = approach.x;
    m.approach_y = approach.y;
    m.approach_z = approach.z;
    m.bearing_yaw = meanBearingYaw();
    m.bearing_pitch = meanBearingPitch();
    m.object_yaw = std::atan2(yaw_s, yaw_c);
    return m;
}

double TargetLock::maxLockedTargetDeviation(const TargetSample &mean) const
{
    double max_d = 0.0;
    for (const auto &p : buffer_)
    {
        const double dx = p.x - mean.x;
        const double dy = p.y - mean.y;
        const double dz = p.z - mean.z;
        const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
        max_d = std::max(max_d, d);
    }
    return max_d;
}

double TargetLock::maxLockedBearingDeviation(const TargetSample &mean) const
{
    double max_d = 0.0;
    for (const auto &p : buffer_)
    {
        const double dyaw = wrapPi(p.bearing_yaw - mean.bearing_yaw);
        const double dpitch = p.bearing_pitch - mean.bearing_pitch;
        max_d = std::max(max_d, std::hypot(dyaw, dpitch));
    }
    return max_d;
}

double TargetLock::meanBearingYaw() const
{
    double c = 0.0;
    double s = 0.0;
    for (const auto &p : buffer_)
    {
        c += std::cos(p.bearing_yaw);
        s += std::sin(p.bearing_yaw);
    }
    return std::atan2(s, c);
}

double TargetLock::meanBearingPitch() const
{
    if (buffer_.empty())
    {
        return 0.0;
    }

    double sum = 0.0;
    for (const auto &p : buffer_)
    {
        sum += p.bearing_pitch;
    }
    return sum / static_cast<double>(buffer_.size());
}

TargetVector TargetLock::normalizedVector(const TargetVector &v, const TargetVector &fallback)
{
    const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (n <= 1e-9 || !std::isfinite(n))
    {
        return fallback;
    }

    return {v.x / n, v.y / n, v.z / n};
}

double TargetLock::wrapPi(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

}  // namespace gimbal_mani::arm_upper
