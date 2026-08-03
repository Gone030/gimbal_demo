#pragma once

#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>
#include <limits>
#include <cstdint>
#include <memory>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "gimbal_mani/arm_upper/arm_kinematics.hpp"
#include "gimbal_mani/arm_upper/arm_auto_controller.hpp"
#include "gimbal_mani/arm_upper/arm_debug.hpp"
#include "gimbal_mani/arm_upper/arm_debug_snapshot_builder.hpp"
#include "gimbal_mani/arm_upper/arm_param_loader.hpp"
#include "gimbal_mani/arm_upper/arm_params.hpp"
#include "gimbal_mani/arm_upper/arm_tf_lookup.hpp"
#include "gimbal_mani/arm_upper/arm_types.hpp"
#include "gimbal_mani/arm_upper/grasp_planner.hpp"
#include "gimbal_mani/arm_upper/target_lock.hpp"
#include "gimbal_mani/arm_upper/target_projector.hpp"
#include "gimbal_mani/arm_upper/tof_approach.hpp"
#include "gimbal_mani/arm_upper/trajectory_builder.hpp"
#include "gimbal_mani/msg/arm_auto_debug.hpp"
#include "gimbal_mani/msg/target_bearing_range.hpp"


/*
Shoulder_Rotation:  90 deg(left) ~ -90 deg(right)
Shoulder_Pitch:    -90 deg ~ 90 deg
Elbow:              90 deg ~ -90 deg
Wrist_Pitch:       -180 deg ~ 0 deg
Wrist_Roll:          0 deg 고정 운용
Gripper:             0 closed ~ 1.5 open
*/

namespace gimbal_mani::arm_upper
{

class ArmUpperLayerNode : public rclcpp::Node
{
public:
    ArmUpperLayerNode();

private:
    void on_timer();

    void on_target(const gimbal_mani::msg::TargetBearingRange &msg);

    void on_home(const std_msgs::msg::Empty &);

    void on_auto_enable(const std_msgs::msg::Bool &msg);

    void on_grasp_start(const std_msgs::msg::Empty &);

    void on_go_up(const std_msgs::msg::Empty &);

    void on_tof_scan(const sensor_msgs::msg::LaserScan &msg);

    void on_joint_state(const sensor_msgs::msg::JointState &msg);

    void publish_current_goal();

    bool goal_within_joint_limits(const std::vector<double> &goal, bool warn);

    static double display_joint_value(size_t index, double value);

    static const char *joint_display_unit(size_t index);

    void reset_auto_sequence();

    bool plan_locked_points();

    bool update_pre_grasp_point();

    TargetPoint compute_pre_grasp_point(const TargetPoint &object_point);

    bool solve_goal_from_point(const TargetPoint &p, double wrist_roll_ref);

    double arm_goal_error_sum() const;

    bool arm_goal_reached(double tol) const;

    static double point_distance(const TargetPoint &a, const TargetPoint &b);

    bool cartesian_goal_reached(double tol) const;

    bool freeze_ready() const;

    bool use_tof_approach_ik_bias() const;

    static double wrap_pi(double angle);

    bool approach_axis(TargetVector &axis) const;

    void freeze_pre_grasp_point();

    double tof_age_sec() const;

    bool tof_range_valid() const;

    bool init_tof_joint_approach();

    bool run_tof_joint_probe_step();

    bool hold_last_safe_tof_joint_goal();

    double current_forward_progress() const;

    void stop_tof_approach_and_hold();

    bool tof_target_inside_gripper() const;

    bool lookup_current_arm_point(geometry_msgs::msg::Point &out) const;

    double cartesian_error_to_goal(const geometry_msgs::msg::Point &current) const;

    double current_target_lock_max_deviation() const;

    double current_target_lock_max_bearing_deviation() const;

    ArmDebugSnapshot make_debug_snapshot();

    void publish_auto_debug();

    void publish_auto_status();

    rcl_interfaces::msg::SetParametersResult on_parameters_changed(const std::vector<rclcpp::Parameter> &params);

    ArmParamValues current_param_values() const;

    void apply_param_values(const ArmParamValues &values);

    void sync_params_from_members();

private:
    double L1_{0.116}, L2_{0.135};
    bool saturate_reach_{true};
    bool target_valid_{false};
    bool auto_enabled_{false};
    double command_time_from_start_sec_{0.1};
    std::string arm_base_frame_{"Base"};
    std::string end_effector_frame_{"gripper_grasp_frame"};
    std::string target_sensor_frame_{"tof_link"};
    double min_effective_range_{0.0};

    double shoulder_rot_x_{0.0};
    double shoulder_rot_y_{-0.0452};
    double shoulder_rot_z_{0.0165};

    double shoulder_pitch_offset_y_{0.1025};
    double shoulder_pitch_offset_z_{0.0306};

    double wrist_pitch_level_bias_{-1.5};

    double pre_grasp_distance_m_{0.05};
    double pre_grasp_lift_z_m_{0.0};
    double pre_grasp_lateral_offset_m_{0.023};

    int target_lock_min_samples_{8};
    double target_lock_pos_tol_m_{0.01};
    double hold_time_sec_{0.4};
    double joint_reach_tol_rad_{0.25};
    double hold_exit_tol_rad_{0.35};
    double target_filter_alpha_{0.05};
    double freeze_cartesian_tol_m_{0.03};
    double hold_target_exit_tol_m_{0.02};
    double target_lock_bearing_tol_rad_{0.03};
    double desired_grasp_range_m_{0.03};
    double gripper_inside_range_m_{0.070};
    double gripper_inner_extra_approach_m_{0.02};
    double tof_approach_step_m_{0.01};
    double min_forward_approach_m_{0.0};
    double max_forward_approach_m_{0.085};
    double tof_timeout_sec_{0.5};
    double gripper_open_pos_{0.8};
    double gripper_close_pos_{0.0};
    double gripper_action_time_sec_{0.8};
    double lift_distance_m_{0.04};
    double lift_command_time_sec_{1.5};
    double lift_cartesian_tol_m_{0.015};
    double approach_reach_tol_rad_{0.25};
    int numerical_ik_max_iterations_{30};
    double numerical_ik_position_tol_m_{0.005};
    double numerical_ik_axis_tol_rad_{0.05236};
    double numerical_ik_accept_position_m_{0.015};
    double numerical_ik_accept_axis_rad_{0.52360};
    double numerical_ik_fd_step_rad_{0.001};
    double numerical_ik_max_step_rad_{0.08727};
    double numerical_ik_damping_{0.0001};
    double numerical_ik_axis_weight_m_{0.10};
    double numerical_ik_position_cost_weight_{10.0};
    double numerical_ik_axis_cost_weight_{0.2};
    double tof_range_increase_tol_m_{0.005};
    double tof_progress_reverse_tol_m_{0.005};
    double tof_cartesian_error_stop_m_{0.06};
    double tof_joint_probe_step_rad_{0.035};
    double tof_joint_probe_shoulder_ratio_{0.3};
    double tof_joint_probe_wrist_ratio_{1.0};
    double tof_joint_probe_command_time_sec_{0.25};
    double tof_joint_probe_reach_tol_rad_{0.02};
    double tof_success_range_delta_m_{0.002};
    double tof_fail_range_increase_m_{0.002};
    int tof_max_failed_probe_count_{6};
    double tof_ik_elbow_only_penalty_{8.0};
    double tof_ik_shoulder_bonus_{1.0};
    double tof_ik_shoulder_balance_ratio_{0.8};
    double wrist_to_grasp_local_x_m_{-0.010};
    double wrist_to_grasp_forward_offset_m_{0.1151};
    double wrist_to_grasp_local_z_m_{-0.030};

    AutoPhase auto_phase_{AutoPhase::IDLE};
    TargetPoint locked_object_point_{};
    TargetVector locked_approach_axis_{};
    TargetPoint pre_grasp_point_{};
    TargetPoint commanded_goal_point_{};
    TargetPoint frozen_pre_grasp_point_{};
    TargetPoint tracked_pre_grasp_candidate_{};
    TargetPoint lift_goal_point_{};
    std::vector<double> frozen_pre_grasp_joint_positions_;
    bool has_locked_object_point_{false};
    bool has_commanded_goal_point_{false};
    bool has_frozen_pre_grasp_point_{false};
    bool has_tracked_pre_grasp_candidate_{false};
    bool has_frozen_pre_grasp_joints_{false};
    bool has_lift_goal_point_{false};
    bool has_frozen_approach_axis_{false};
    TargetVector frozen_approach_axis_{};
    double locked_object_yaw_{0.0};
    double hold_start_time_sec_{-1.0};
    double phase_start_time_sec_{-1.0};
    bool hold_reached_logged_{false};
    bool grasp_start_requested_{false};
    bool go_up_requested_{false};
    bool has_tof_range_{false};
    double latest_tof_range_m_{std::numeric_limits<double>::quiet_NaN()};
    double last_tof_time_sec_{-1.0};
    TargetVector last_valid_approach_axis_{};

    std::vector<std::string> joint_name_;
    std::vector<double> goal_;
    std::vector<double> home_goal_;
    std::vector<double> q_meas_{std::vector<double>(6, 0.0)};
    std::string last_limit_rejection_;
    bool has_joint_state_{false};
    ArmUpperParams params_;
    ArmKinematics kinematics_;
    TargetProjector target_projector_;
    TargetLock target_lock_;
    GraspPlanner grasp_planner_;
    TrajectoryBuilder trajectory_builder_;
    TofApproach tof_approach_;
    ArmAutoController auto_controller_;
    ArmTfLookup arm_tf_lookup_;
    ArmDebugSnapshotBuilder debug_snapshot_builder_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_traj_;
    rclcpp::Publisher<gimbal_mani::msg::ArmAutoDebug>::SharedPtr pub_auto_debug_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_auto_status_;
    rclcpp::Subscription<gimbal_mani::msg::TargetBearingRange>::SharedPtr sub_target_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_home_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_auto_enable_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_grasp_start_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_go_up_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_tof_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace gimbal_mani::arm_upper
