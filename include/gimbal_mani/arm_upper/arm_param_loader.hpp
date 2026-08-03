#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "gimbal_mani/arm_upper/arm_params.hpp"

namespace gimbal_mani::arm_upper
{

struct ArmParamValues
{
    std::string arm_base_frame{"Base"};
    std::string end_effector_frame{"gripper_grasp_frame"};
    std::string target_sensor_frame{"tof_link"};
    double min_effective_range{0.0};

    double L1{0.116};
    double L2{0.135};
    bool saturate_reach{true};
    double shoulder_rot_x{0.0};
    double shoulder_rot_y{-0.0452};
    double shoulder_rot_z{0.0165};
    double shoulder_pitch_offset_y{0.1025};
    double shoulder_pitch_offset_z{0.0306};
    double wrist_pitch_level_bias{-1.5};

    double pre_grasp_distance_m{0.05};
    double pre_grasp_lift_z_m{0.0};
    double pre_grasp_lateral_offset_m{0.023};

    int target_lock_min_samples{8};
    double target_lock_pos_tol_m{0.01};
    double hold_time_sec{0.4};
    double joint_reach_tol_rad{0.25};
    double hold_exit_tol_rad{0.35};
    double target_filter_alpha{0.05};
    double freeze_cartesian_tol_m{0.03};
    double hold_target_exit_tol_m{0.02};
    double target_lock_bearing_tol_rad{0.03};

    double desired_grasp_range_m{0.03};
    double gripper_inside_range_m{0.070};
    double gripper_inner_extra_approach_m{0.02};
    double tof_approach_step_m{0.01};
    double min_forward_approach_m{0.0};
    double max_forward_approach_m{0.085};
    double tof_timeout_sec{0.5};

    double gripper_open_pos{0.8};
    double gripper_close_pos{0.0};
    double gripper_action_time_sec{0.8};
    double lift_distance_m{0.04};
    double lift_command_time_sec{1.5};
    double lift_cartesian_tol_m{0.015};
    double approach_reach_tol_rad{0.25};

    int numerical_ik_max_iterations{30};
    double numerical_ik_position_tol_m{0.005};
    double numerical_ik_axis_tol_rad{0.05236};
    double numerical_ik_accept_position_m{0.015};
    double numerical_ik_accept_axis_rad{0.52360};
    double numerical_ik_fd_step_rad{0.001};
    double numerical_ik_max_step_rad{0.08727};
    double numerical_ik_damping{0.0001};
    double numerical_ik_axis_weight_m{0.10};
    double numerical_ik_position_cost_weight{10.0};
    double numerical_ik_axis_cost_weight{0.2};

    double tof_range_increase_tol_m{0.005};
    double tof_progress_reverse_tol_m{0.005};
    double tof_cartesian_error_stop_m{0.06};
    double tof_joint_probe_step_rad{0.035};
    double tof_joint_probe_shoulder_ratio{0.3};
    double tof_joint_probe_wrist_ratio{1.0};
    double tof_joint_probe_command_time_sec{0.25};
    double tof_joint_probe_reach_tol_rad{0.02};
    double tof_success_range_delta_m{0.002};
    double tof_fail_range_increase_m{0.002};
    int tof_max_failed_probe_count{6};
    double tof_ik_elbow_only_penalty{8.0};
    double tof_ik_shoulder_bonus{1.0};
    double tof_ik_shoulder_balance_ratio{0.8};
    double wrist_to_grasp_local_x_m{-0.010};
    double wrist_to_grasp_forward_offset_m{0.1151};
    double wrist_to_grasp_local_z_m{-0.030};

    std::vector<std::string> joint_names{
        "Shoulder_Rotation",
        "Shoulder_Pitch",
        "Elbow",
        "Wrist_Pitch",
        "Wrist_Roll",
        "Gripper"};

    ArmUpperParams params;
};

class ArmParamLoader
{
public:
    static ArmParamValues declareAndLoad(rclcpp::Node &node);
    static ArmUpperParams toArmUpperParams(const ArmParamValues &values);
};

}  // namespace gimbal_mani::arm_upper
