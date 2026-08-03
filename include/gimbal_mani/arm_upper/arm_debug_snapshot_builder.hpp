#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>

#include "gimbal_mani/arm_upper/arm_debug.hpp"
#include "gimbal_mani/arm_upper/tof_approach.hpp"

namespace gimbal_mani::arm_upper
{

struct DebugSnapshotInput
{
    builtin_interfaces::msg::Time stamp;
    double now_sec{0.0};
    std::string base_frame;
    std::string end_effector_frame;

    bool auto_enabled{false};
    AutoPhase phase{AutoPhase::IDLE};
    bool target_valid{false};
    bool has_joint_state{false};
    bool has_locked_object_point{false};

    TargetPoint locked_object_point;
    TargetPoint pre_grasp_point;
    TargetPoint commanded_goal_point;
    TargetPoint frozen_pre_grasp_point;
    TargetPoint tracked_pre_grasp_candidate;

    bool has_commanded_goal_point{false};
    bool has_current_arm_point{false};
    TargetPoint current_arm_point;
    double freeze_cartesian_tol_m{0.0};
    bool hold_frozen{false};
    bool has_tracked_pre_grasp_candidate{false};
    double hold_target_exit_tol_m{0.0};
    bool grasp_start_requested{false};

    bool has_tof_range{false};
    double tof_range_m{0.0};
    double tof_age_sec{0.0};
    double desired_grasp_range_m{0.0};
    double gripper_inside_range_m{0.0};
    double gripper_inner_extra_approach_m{0.0};
    double tof_approach_step_m{0.0};
    bool tof_target_inside_gripper{false};
    TofDebugState tof_debug;

    bool has_frozen_pre_grasp_joints{false};
    std::vector<double> frozen_pre_grasp_joint_positions;

    std::vector<double> current_joint_positions;
    std::vector<double> target_joint_positions;
    double joint_error_sum{0.0};
    double joint_reach_tol_rad{0.0};
    bool joint_goal_reached{false};

    double hold_start_time_sec{-1.0};
    double hold_time_sec{0.0};

    int32_t target_lock_samples{0};
    int32_t target_lock_min_samples{0};
    double target_lock_max_deviation_m{0.0};
    double target_lock_pos_tol_m{0.0};
    double target_lock_max_bearing_deviation_rad{0.0};

    std::string last_limit_rejection;
};

class ArmDebugSnapshotBuilder
{
public:
    ArmDebugSnapshot build(const DebugSnapshotInput &input) const;

private:
    static double cartesianErrorToGoal(const DebugSnapshotInput &input);
    static double pointDistance(const TargetPoint &a, const TargetPoint &b);
    static double wrapPi(double angle);
    static double shoulderPitchDeltaFromHold(const DebugSnapshotInput &input);
    static double elbowDeltaFromHold(const DebugSnapshotInput &input);
    static double shoulderToElbowDeltaRatio(const DebugSnapshotInput &input);
    static double holdElapsedSec(const DebugSnapshotInput &input);
    static double holdTargetDrift(const DebugSnapshotInput &input);
};

}  // namespace gimbal_mani::arm_upper
