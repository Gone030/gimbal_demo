#pragma once

#include <optional>

#include "gimbal_mani/arm_upper/arm_params.hpp"
#include "gimbal_mani/arm_upper/arm_types.hpp"

namespace gimbal_mani::arm_upper
{

class GraspPlanner
{
public:
    GraspPlanner() = default;
    explicit GraspPlanner(const GraspParams &params);

    void setParams(const GraspParams &params);
    std::optional<TargetPoint> computePreGraspPoint(
        const TargetPoint &object_point,
        const TargetVector &approach_axis) const;
    TargetPoint makeLiftGoal(const TargetPoint &current_point, double lift_distance_m) const;

    static double pointDistance(const TargetPoint &a, const TargetPoint &b);

private:
    static TargetVector normalizedVector(const TargetVector &v, const TargetVector &fallback);

    GraspParams params_;
};

}  // namespace gimbal_mani::arm_upper
