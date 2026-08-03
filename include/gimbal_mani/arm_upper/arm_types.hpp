#pragma once

#include <array>
#include <string>
#include <vector>

namespace gimbal_mani::arm_upper
{

enum class AutoPhase
{
    IDLE,
    TARGET_ACQUIRE,
    TRACK_PRE_GRASP,
    PRE_GRASP_HOLD,
    GRIPPER_OPEN,
    TOF_APPROACH,
    GRIPPER_CLOSE,
    LIFT,
    GRASP_HOLD,
};

struct Vec3
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

using TargetPoint = Vec3;
using TargetVector = Vec3;

struct TargetSample
{
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double approach_x{1.0};
    double approach_y{0.0};
    double approach_z{0.0};
    double bearing_yaw{0.0};
    double bearing_pitch{0.0};
    double object_yaw{0.0};
};

struct ArmGoal
{
    std::array<double, 6> positions{};
};

struct JointState6
{
    std::array<double, 6> positions{};
    bool valid{false};
};

using JointVector = std::vector<double>;

}  // namespace gimbal_mani::arm_upper
