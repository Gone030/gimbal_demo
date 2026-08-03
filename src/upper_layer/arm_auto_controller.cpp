#include "gimbal_mani/arm_upper/arm_auto_controller.hpp"

#include <cmath>

namespace gimbal_mani::arm_upper
{

void ArmAutoController::tick(AutoControllerContext &ctx) const
{
    if (!ctx.auto_enabled || ctx.phase == nullptr)
    {
        return;
    }

    switch (*ctx.phase)
    {
    case AutoPhase::IDLE:
        break;

    case AutoPhase::TARGET_ACQUIRE:
    {
        if (ctx.plan_locked_points())
        {
            if (!ctx.solve_goal_from_point(*ctx.pre_grasp_point, ctx.locked_object_yaw))
            {
                *ctx.target_valid = false;
                break;
            }

            *ctx.command_time_from_start_sec = 1.0;
            *ctx.phase = AutoPhase::TRACK_PRE_GRASP;
        }
        break;
    }

    case AutoPhase::TRACK_PRE_GRASP:
    {
        if (!ctx.solve_goal_from_point(*ctx.pre_grasp_point, ctx.locked_object_yaw))
        {
            *ctx.target_valid = false;
            break;
        }

        *ctx.command_time_from_start_sec = 1.0;
        if (ctx.freeze_ready())
        {
            ctx.freeze_pre_grasp_point();
            *ctx.phase = AutoPhase::PRE_GRASP_HOLD;
            *ctx.hold_start_time_sec = ctx.now_sec;
            *ctx.hold_reached_logged = false;
        }
        break;
    }

    case AutoPhase::PRE_GRASP_HOLD:
    {
        if (!*ctx.has_frozen_pre_grasp_point)
        {
            ctx.freeze_pre_grasp_point();
        }

        if (*ctx.has_tracked_pre_grasp_candidate)
        {
            const double drift = pointDistance(*ctx.tracked_pre_grasp_candidate, *ctx.frozen_pre_grasp_point);
            if (drift > ctx.hold_target_exit_tol_m)
            {
                *ctx.pre_grasp_point = *ctx.tracked_pre_grasp_candidate;
                *ctx.has_frozen_pre_grasp_point = false;
                *ctx.phase = AutoPhase::TRACK_PRE_GRASP;
                *ctx.hold_start_time_sec = -1.0;
                *ctx.hold_reached_logged = false;
                break;
            }
        }

        if (!ctx.solve_goal_from_point(*ctx.frozen_pre_grasp_point, ctx.locked_object_yaw))
        {
            *ctx.target_valid = false;
            break;
        }

        *ctx.command_time_from_start_sec = 1.0;
        if (!ctx.arm_goal_reached(ctx.hold_exit_tol_rad))
        {
            *ctx.phase = AutoPhase::TRACK_PRE_GRASP;
            *ctx.hold_start_time_sec = -1.0;
            *ctx.hold_reached_logged = false;
            break;
        }

        const double held = ctx.now_sec - *ctx.hold_start_time_sec;
        if (held >= ctx.hold_time_sec && !*ctx.hold_reached_logged)
        {
            ctx.log_pre_grasp_hold_reached();
            *ctx.hold_reached_logged = true;
        }
        if (*ctx.hold_reached_logged && *ctx.grasp_start_requested)
        {
            *ctx.grasp_start_requested = false;
            if (ctx.tof_approach_blocked())
            {
                ctx.warn_grasp_start_tof_blocked();
                break;
            }
            *ctx.phase = AutoPhase::GRIPPER_OPEN;
            *ctx.phase_start_time_sec = ctx.now_sec;
        }
        break;
    }

    case AutoPhase::GRIPPER_OPEN:
    {
        if (!ctx.solve_goal_from_point(*ctx.frozen_pre_grasp_point, ctx.locked_object_yaw))
        {
            *ctx.target_valid = false;
            break;
        }

        (*ctx.goal)[5] = ctx.gripper_open_pos;
        *ctx.command_time_from_start_sec = ctx.gripper_action_time_sec;
        if (elapsedInPhase(ctx) >= ctx.gripper_action_time_sec &&
            ctx.init_tof_joint_approach())
        {
            *ctx.phase = AutoPhase::TOF_APPROACH;
            *ctx.phase_start_time_sec = ctx.now_sec;
        }
        break;
    }

    case AutoPhase::TOF_APPROACH:
    {
        if (ctx.tof_target_inside_gripper())
        {
            *ctx.phase = AutoPhase::GRIPPER_CLOSE;
            *ctx.phase_start_time_sec = ctx.now_sec;
            break;
        }

        if (!ctx.run_tof_joint_probe_step())
        {
            ctx.stop_tof_approach_and_hold();
        }
        break;
    }

    case AutoPhase::GRIPPER_CLOSE:
    {
        if (!ctx.hold_last_safe_tof_joint_goal())
        {
            *ctx.target_valid = false;
            break;
        }

        (*ctx.goal)[5] = ctx.gripper_close_pos;
        *ctx.command_time_from_start_sec = ctx.gripper_action_time_sec;
        if (elapsedInPhase(ctx) >= ctx.gripper_action_time_sec)
        {
            *ctx.phase = AutoPhase::GRASP_HOLD;
        }
        break;
    }

    case AutoPhase::LIFT:
    {
        if (!*ctx.has_lift_goal_point ||
            !ctx.solve_goal_from_point(*ctx.lift_goal_point, ctx.locked_object_yaw))
        {
            *ctx.target_valid = ctx.hold_last_safe_tof_joint_goal();
            ctx.warn_lift_ik_unavailable();
            break;
        }

        (*ctx.goal)[5] = ctx.gripper_close_pos;
        *ctx.command_time_from_start_sec = ctx.lift_command_time_sec;

        if (ctx.arm_goal_reached(ctx.joint_reach_tol_rad) &&
            ctx.cartesian_goal_reached(ctx.lift_cartesian_tol_m))
        {
            (*ctx.goal)[5] = ctx.gripper_close_pos;
            ctx.set_last_safe_goal();
            *ctx.phase = AutoPhase::GRASP_HOLD;
            ctx.log_lift_reached(ctx.lift_distance_m);
        }
        break;
    }

    case AutoPhase::GRASP_HOLD:
    {
        if (!ctx.hold_last_safe_tof_joint_goal())
        {
            *ctx.target_valid = false;
            break;
        }

        (*ctx.goal)[5] = ctx.gripper_close_pos;
        *ctx.command_time_from_start_sec = 1.0;

        if (*ctx.go_up_requested)
        {
            TargetPoint current;
            if (!ctx.lookup_current_arm_point(current))
            {
                ctx.warn_waiting_for_lift_tf();
                break;
            }

            *ctx.go_up_requested = false;
            *ctx.lift_goal_point = ctx.make_lift_goal(current, ctx.lift_distance_m);
            *ctx.has_lift_goal_point = true;
            *ctx.phase = AutoPhase::LIFT;
            *ctx.phase_start_time_sec = ctx.now_sec;
        }
        break;
    }
    }
}

double ArmAutoController::elapsedInPhase(const AutoControllerContext &ctx)
{
    if (ctx.phase_start_time_sec == nullptr || *ctx.phase_start_time_sec < 0.0)
    {
        return 0.0;
    }
    return ctx.now_sec - *ctx.phase_start_time_sec;
}

double ArmAutoController::pointDistance(const TargetPoint &a, const TargetPoint &b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace gimbal_mani::arm_upper
