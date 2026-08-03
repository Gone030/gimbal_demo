#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>

#include "gimbal_mani/arm_upper/arm_types.hpp"
#include "gimbal_mani/msg/arm_auto_debug.hpp"

namespace gimbal_mani::arm_upper
{

struct ArmDebugSnapshot
{
    builtin_interfaces::msg::Time stamp;
    std::string base_frame;
    std::string end_effector_frame;

    bool auto_enabled{false};
    AutoPhase phase{AutoPhase::IDLE};
    bool target_valid{false};
    bool has_joint_state{false};
    bool has_locked_object_point{false};

    TargetPoint object_point_base;
    TargetPoint pre_grasp_hold_point_base;
    TargetPoint commanded_goal_point_base;
    TargetPoint frozen_pre_grasp_point_base;
    TargetPoint tracked_pre_grasp_candidate_base;

    bool has_current_arm_point{false};
    TargetPoint current_arm_point_base;
    double cartesian_error_to_goal_m{0.0};
    double freeze_cartesian_tol_m{0.0};
    bool cartesian_goal_reached{false};
    bool freeze_ready{false};
    bool hold_frozen{false};
    double hold_target_drift_m{0.0};
    double hold_target_exit_tol_m{0.0};
    bool grasp_start_requested{false};

    bool has_tof_range{false};
    double tof_range_m{0.0};
    double tof_age_sec{0.0};
    double desired_grasp_range_m{0.0};
    double gripper_inside_range_m{0.0};
    double gripper_inner_extra_approach_m{0.0};
    double tof_approach_step_m{0.0};
    double tof_forward_distance_m{0.0};
    bool tof_target_inside_gripper{false};
    bool has_tof_approach_point{false};
    TargetPoint tof_approach_point_base;

    bool has_frozen_pre_grasp_joints{false};
    double shoulder_pitch_delta_from_hold_rad{0.0};
    double elbow_delta_from_hold_rad{0.0};
    double shoulder_to_elbow_delta_ratio{0.0};

    std::vector<double> current_joint_positions;
    std::vector<double> target_joint_positions;
    double joint_error_sum{0.0};
    double joint_reach_tol_rad{0.0};
    bool joint_goal_reached{false};

    double hold_elapsed_sec{0.0};
    double hold_time_sec{0.0};

    int32_t target_lock_samples{0};
    int32_t target_lock_min_samples{0};
    double target_lock_max_deviation_m{0.0};
    double target_lock_pos_tol_m{0.0};
    double target_lock_max_bearing_deviation_rad{0.0};

    int tof_active_probe_index{-1};
    int tof_probe_failed_count{0};
    bool tof_approach_blocked{false};
    std::string last_limit_rejection;
};

class ArmDebug
{
public:
    static const char *phaseName(AutoPhase phase);
    static gimbal_mani::msg::ArmAutoDebug buildDebugMessage(const ArmDebugSnapshot &snapshot);
    static std_msgs::msg::String buildStatusMessage(const ArmDebugSnapshot &snapshot);

private:
    static geometry_msgs::msg::Point toPointMsg(const TargetPoint &p);
};

}  // namespace gimbal_mani::arm_upper
