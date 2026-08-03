#include "gimbal_mani/arm_upper/arm_param_loader.hpp"

namespace gimbal_mani::arm_upper
{

ArmParamValues ArmParamLoader::declareAndLoad(rclcpp::Node &node)
{
    ArmParamValues values;

    values.pre_grasp_distance_m =
        node.declare_parameter<double>("pre_grasp_distance_m", values.pre_grasp_distance_m);
    values.pre_grasp_lift_z_m =
        node.declare_parameter<double>("pre_grasp_lift_z_m", values.pre_grasp_lift_z_m);
    values.pre_grasp_lateral_offset_m =
        node.declare_parameter<double>("pre_grasp_lateral_offset_m", values.pre_grasp_lateral_offset_m);
    values.target_lock_bearing_tol_rad =
        node.declare_parameter<double>("target_lock_bearing_tol_rad", values.target_lock_bearing_tol_rad);
    values.max_forward_approach_m =
        node.declare_parameter<double>("max_forward_approach_m", values.max_forward_approach_m);
    values.gripper_open_pos =
        node.declare_parameter<double>("gripper_open_pos", values.gripper_open_pos);
    values.gripper_close_pos =
        node.declare_parameter<double>("gripper_close_pos", values.gripper_close_pos);
    values.gripper_action_time_sec =
        node.declare_parameter<double>("gripper_action_time_sec", values.gripper_action_time_sec);
    values.lift_distance_m =
        node.declare_parameter<double>("lift_distance_m", values.lift_distance_m);
    values.lift_command_time_sec =
        node.declare_parameter<double>("lift_command_time_sec", values.lift_command_time_sec);
    values.lift_cartesian_tol_m =
        node.declare_parameter<double>("lift_cartesian_tol_m", values.lift_cartesian_tol_m);
    values.numerical_ik_max_iterations =
        node.declare_parameter<int>("numerical_ik_max_iterations", values.numerical_ik_max_iterations);
    values.numerical_ik_position_tol_m =
        node.declare_parameter<double>("numerical_ik_position_tol_m", values.numerical_ik_position_tol_m);
    values.numerical_ik_axis_tol_rad =
        node.declare_parameter<double>("numerical_ik_axis_tol_rad", values.numerical_ik_axis_tol_rad);
    values.numerical_ik_accept_position_m =
        node.declare_parameter<double>("numerical_ik_accept_position_m", values.numerical_ik_accept_position_m);
    values.numerical_ik_accept_axis_rad =
        node.declare_parameter<double>("numerical_ik_accept_axis_rad", values.numerical_ik_accept_axis_rad);
    values.numerical_ik_max_step_rad =
        node.declare_parameter<double>("numerical_ik_max_step_rad", values.numerical_ik_max_step_rad);
    values.numerical_ik_damping =
        node.declare_parameter<double>("numerical_ik_damping", values.numerical_ik_damping);
    values.numerical_ik_axis_weight_m =
        node.declare_parameter<double>("numerical_ik_axis_weight_m", values.numerical_ik_axis_weight_m);
    values.tof_joint_probe_step_rad =
        node.declare_parameter<double>("tof_joint_probe_step_rad", values.tof_joint_probe_step_rad);
    values.tof_joint_probe_shoulder_ratio =
        node.declare_parameter<double>("tof_joint_probe_shoulder_ratio", values.tof_joint_probe_shoulder_ratio);
    values.tof_joint_probe_wrist_ratio =
        node.declare_parameter<double>("tof_joint_probe_wrist_ratio", values.tof_joint_probe_wrist_ratio);
    values.tof_joint_probe_command_time_sec =
        node.declare_parameter<double>("tof_joint_probe_command_time_sec", values.tof_joint_probe_command_time_sec);
    values.tof_joint_probe_reach_tol_rad =
        node.declare_parameter<double>("tof_joint_probe_reach_tol_rad", values.tof_joint_probe_reach_tol_rad);
    values.tof_success_range_delta_m =
        node.declare_parameter<double>("tof_success_range_delta_m", values.tof_success_range_delta_m);
    values.tof_fail_range_increase_m =
        node.declare_parameter<double>("tof_fail_range_increase_m", values.tof_fail_range_increase_m);
    values.tof_max_failed_probe_count =
        node.declare_parameter<int>("tof_max_failed_probe_count", values.tof_max_failed_probe_count);

    values.params = toArmUpperParams(values);
    return values;
}

ArmUpperParams ArmParamLoader::toArmUpperParams(const ArmParamValues &values)
{
    ArmUpperParams params;

    params.kinematics.L1 = values.L1;
    params.kinematics.L2 = values.L2;
    params.kinematics.saturate_reach = values.saturate_reach;
    params.kinematics.shoulder_rot_x = values.shoulder_rot_x;
    params.kinematics.shoulder_rot_y = values.shoulder_rot_y;
    params.kinematics.shoulder_rot_z = values.shoulder_rot_z;
    params.kinematics.shoulder_pitch_offset_y = values.shoulder_pitch_offset_y;
    params.kinematics.shoulder_pitch_offset_z = values.shoulder_pitch_offset_z;
    params.kinematics.wrist_pitch_level_bias = values.wrist_pitch_level_bias;
    params.kinematics.numerical_ik_max_iterations = values.numerical_ik_max_iterations;
    params.kinematics.numerical_ik_position_tol_m = values.numerical_ik_position_tol_m;
    params.kinematics.numerical_ik_axis_tol_rad = values.numerical_ik_axis_tol_rad;
    params.kinematics.numerical_ik_accept_position_m = values.numerical_ik_accept_position_m;
    params.kinematics.numerical_ik_accept_axis_rad = values.numerical_ik_accept_axis_rad;
    params.kinematics.numerical_ik_fd_step_rad = values.numerical_ik_fd_step_rad;
    params.kinematics.numerical_ik_max_step_rad = values.numerical_ik_max_step_rad;
    params.kinematics.numerical_ik_damping = values.numerical_ik_damping;
    params.kinematics.numerical_ik_axis_weight_m = values.numerical_ik_axis_weight_m;
    params.kinematics.numerical_ik_position_cost_weight = values.numerical_ik_position_cost_weight;
    params.kinematics.numerical_ik_axis_cost_weight = values.numerical_ik_axis_cost_weight;
    params.kinematics.tof_ik_elbow_only_penalty = values.tof_ik_elbow_only_penalty;
    params.kinematics.tof_ik_shoulder_bonus = values.tof_ik_shoulder_bonus;
    params.kinematics.tof_ik_shoulder_balance_ratio = values.tof_ik_shoulder_balance_ratio;
    params.kinematics.wrist_to_grasp_local_x_m = values.wrist_to_grasp_local_x_m;
    params.kinematics.wrist_to_grasp_forward_offset_m = values.wrist_to_grasp_forward_offset_m;
    params.kinematics.wrist_to_grasp_local_z_m = values.wrist_to_grasp_local_z_m;
    params.kinematics.joint_names = values.joint_names;

    params.target_lock.min_samples = values.target_lock_min_samples;
    params.target_lock.pos_tol_m = values.target_lock_pos_tol_m;
    params.target_lock.bearing_tol_rad = values.target_lock_bearing_tol_rad;
    params.target_lock.filter_alpha = values.target_filter_alpha;
    params.target_lock.hold_time_sec = values.hold_time_sec;
    params.target_lock.joint_reach_tol_rad = values.joint_reach_tol_rad;
    params.target_lock.hold_exit_tol_rad = values.hold_exit_tol_rad;
    params.target_lock.freeze_cartesian_tol_m = values.freeze_cartesian_tol_m;
    params.target_lock.hold_target_exit_tol_m = values.hold_target_exit_tol_m;

    params.grasp.pre_grasp_distance_m = values.pre_grasp_distance_m;
    params.grasp.pre_grasp_lift_z_m = values.pre_grasp_lift_z_m;
    params.grasp.pre_grasp_lateral_offset_m = values.pre_grasp_lateral_offset_m;
    params.grasp.gripper_open_pos = values.gripper_open_pos;
    params.grasp.gripper_close_pos = values.gripper_close_pos;
    params.grasp.gripper_action_time_sec = values.gripper_action_time_sec;
    params.grasp.lift_distance_m = values.lift_distance_m;
    params.grasp.lift_command_time_sec = values.lift_command_time_sec;
    params.grasp.lift_cartesian_tol_m = values.lift_cartesian_tol_m;

    params.tof.desired_grasp_range_m = values.desired_grasp_range_m;
    params.tof.gripper_inside_range_m = values.gripper_inside_range_m;
    params.tof.gripper_inner_extra_approach_m = values.gripper_inner_extra_approach_m;
    params.tof.approach_step_m = values.tof_approach_step_m;
    params.tof.min_forward_approach_m = values.min_forward_approach_m;
    params.tof.max_forward_approach_m = values.max_forward_approach_m;
    params.tof.timeout_sec = values.tof_timeout_sec;
    params.tof.approach_reach_tol_rad = values.approach_reach_tol_rad;
    params.tof.range_increase_tol_m = values.tof_range_increase_tol_m;
    params.tof.progress_reverse_tol_m = values.tof_progress_reverse_tol_m;
    params.tof.cartesian_error_stop_m = values.tof_cartesian_error_stop_m;
    params.tof.joint_probe_step_rad = values.tof_joint_probe_step_rad;
    params.tof.joint_probe_shoulder_ratio = values.tof_joint_probe_shoulder_ratio;
    params.tof.joint_probe_wrist_ratio = values.tof_joint_probe_wrist_ratio;
    params.tof.joint_probe_command_time_sec = values.tof_joint_probe_command_time_sec;
    params.tof.joint_probe_reach_tol_rad = values.tof_joint_probe_reach_tol_rad;
    params.tof.success_range_delta_m = values.tof_success_range_delta_m;
    params.tof.fail_range_increase_m = values.tof_fail_range_increase_m;
    params.tof.max_failed_probe_count = values.tof_max_failed_probe_count;

    return params;
}

}  // namespace gimbal_mani::arm_upper
