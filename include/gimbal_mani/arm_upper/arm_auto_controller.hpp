#pragma once

#include <functional>
#include <vector>

#include "gimbal_mani/arm_upper/arm_types.hpp"

namespace gimbal_mani::arm_upper
{

struct AutoControllerContext
{
    double now_sec{0.0};
    bool auto_enabled{false};

    AutoPhase *phase{nullptr};
    bool *target_valid{nullptr};
    std::vector<double> *goal{nullptr};
    double *command_time_from_start_sec{nullptr};

    bool *grasp_start_requested{nullptr};
    bool *go_up_requested{nullptr};

    TargetPoint *pre_grasp_point{nullptr};
    TargetPoint *frozen_pre_grasp_point{nullptr};
    TargetPoint *tracked_pre_grasp_candidate{nullptr};
    TargetPoint *lift_goal_point{nullptr};

    bool *has_frozen_pre_grasp_point{nullptr};
    bool *has_tracked_pre_grasp_candidate{nullptr};
    bool *has_lift_goal_point{nullptr};

    double locked_object_yaw{0.0};
    double *hold_start_time_sec{nullptr};
    double *phase_start_time_sec{nullptr};
    bool *hold_reached_logged{nullptr};

    double hold_target_exit_tol_m{0.0};
    double hold_exit_tol_rad{0.0};
    double hold_time_sec{0.0};
    double gripper_open_pos{0.0};
    double gripper_close_pos{0.0};
    double gripper_action_time_sec{0.0};
    double lift_command_time_sec{0.0};
    double lift_cartesian_tol_m{0.0};
    double joint_reach_tol_rad{0.0};
    double lift_distance_m{0.0};

    std::function<bool()> plan_locked_points;
    std::function<bool(const TargetPoint &, double)> solve_goal_from_point;
    std::function<bool()> freeze_ready;
    std::function<void()> freeze_pre_grasp_point;
    std::function<bool(double)> arm_goal_reached;
    std::function<bool(double)> cartesian_goal_reached;
    std::function<bool()> init_tof_joint_approach;
    std::function<bool()> tof_target_inside_gripper;
    std::function<bool()> run_tof_joint_probe_step;
    std::function<void()> stop_tof_approach_and_hold;
    std::function<bool()> hold_last_safe_tof_joint_goal;
    std::function<bool()> tof_approach_blocked;
    std::function<void()> set_last_safe_goal;
    std::function<bool(TargetPoint &)> lookup_current_arm_point;
    std::function<TargetPoint(const TargetPoint &, double)> make_lift_goal;

    std::function<void()> log_pre_grasp_hold_reached;
    std::function<void()> warn_grasp_start_tof_blocked;
    std::function<void()> warn_lift_ik_unavailable;
    std::function<void()> warn_waiting_for_lift_tf;
    std::function<void(double)> log_lift_reached;
};

class ArmAutoController
{
public:
    void tick(AutoControllerContext &ctx) const;

private:
    static double elapsedInPhase(const AutoControllerContext &ctx);
    static double pointDistance(const TargetPoint &a, const TargetPoint &b);
};

}  // namespace gimbal_mani::arm_upper
