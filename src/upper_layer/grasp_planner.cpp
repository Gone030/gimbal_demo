#include "gimbal_mani/arm_upper/grasp_planner.hpp"

#include <algorithm>
#include <cmath>

namespace gimbal_mani::arm_upper
{

GraspPlanner::GraspPlanner(const GraspParams &params)
    : params_(params)
{
}

void GraspPlanner::setParams(const GraspParams &params)
{
    params_ = params;
}

std::optional<TargetPoint> GraspPlanner::computePreGraspPoint(
    const TargetPoint &object_point,
    const TargetVector &approach_axis) const
{
    const auto axis = normalizedVector(approach_axis, {0.0, 0.0, 0.0});
    const double n = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (n <= 1e-6 || !std::isfinite(n))
    {
        return std::nullopt;
    }

    TargetVector sensor_to_grasp_perp{0.0, 0.0, 0.0};

    TargetPoint out;
    out.x = object_point.x + sensor_to_grasp_perp.x - params_.pre_grasp_distance_m * axis.x;
    out.y = object_point.y + sensor_to_grasp_perp.y - params_.pre_grasp_distance_m * axis.y;
    out.z = object_point.z + sensor_to_grasp_perp.z - params_.pre_grasp_distance_m * axis.z +
            params_.pre_grasp_lift_z_m;

    return out;
}

TargetPoint GraspPlanner::makeLiftGoal(const TargetPoint &current_point, double lift_distance_m) const
{
    return {
        current_point.x,
        current_point.y,
        current_point.z + std::max(0.0, lift_distance_m)};
}

double GraspPlanner::pointDistance(const TargetPoint &a, const TargetPoint &b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

TargetVector GraspPlanner::normalizedVector(const TargetVector &v, const TargetVector &fallback)
{
    const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (n <= 1e-9 || !std::isfinite(n))
    {
        return fallback;
    }

    return {v.x / n, v.y / n, v.z / n};
}

}  // namespace gimbal_mani::arm_upper
