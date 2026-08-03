#pragma once

#include <cstddef>
#include <deque>
#include <limits>
#include <optional>

#include "gimbal_mani/arm_upper/arm_params.hpp"
#include "gimbal_mani/arm_upper/arm_types.hpp"

namespace gimbal_mani::arm_upper
{

struct LockedTarget
{
    TargetPoint object_point;
    TargetVector approach_axis;
    double object_yaw{0.0};
    double max_target_deviation_m{std::numeric_limits<double>::quiet_NaN()};
    double max_bearing_deviation_rad{std::numeric_limits<double>::quiet_NaN()};
};

class TargetLock
{
public:
    TargetLock() = default;
    explicit TargetLock(const TargetLockParams &params);

    void setParams(const TargetLockParams &params);
    void reset();
    void pushSample(const TargetSample &sample);
    size_t sampleCount() const;
    bool empty() const;

    std::optional<LockedTarget> tryLock(const TargetVector &fallback_axis) const;
    double currentMaxTargetDeviation(const TargetVector &fallback_axis) const;
    double currentMaxBearingDeviation(const TargetVector &fallback_axis) const;

private:
    TargetSample meanLockedTarget(const TargetVector &fallback_axis) const;
    double maxLockedTargetDeviation(const TargetSample &mean) const;
    double maxLockedBearingDeviation(const TargetSample &mean) const;
    double meanBearingYaw() const;
    double meanBearingPitch() const;

    static TargetVector normalizedVector(const TargetVector &v, const TargetVector &fallback);
    static double wrapPi(double angle);

    TargetLockParams params_;
    std::deque<TargetSample> buffer_;
};

}  // namespace gimbal_mani::arm_upper
