#include "gimbal_mani/arm_upper/arm_debug_snapshot_builder.hpp"

#include <cmath>
#include <limits>

namespace gimbal_mani::arm_upper
{

ArmDebugSnapshot ArmDebugSnapshotBuilder::build(const DebugSnapshotInput &input) const
{
    ArmDebugSnapshot snapshot;
    snapshot.stamp = input.stamp;
    snapshot.base_frame = input.base_frame;
    snapshot.end_effector_frame = input.end_effector_frame;

    snapshot.auto_enabled = input.auto_enabled;
    snapshot.phase = input.phase;
    snapshot.target_valid = input.target_valid;
    snapshot.has_joint_state = input.has_joint_state;
    snapshot.has_locked_object_point = input.has_locked_object_point;

    snapshot.object_point_base = input.locked_object_point;
    snapshot.pre_grasp_hold_point_base = input.pre_grasp_point;
    snapshot.commanded_goal_point_base = input.commanded_goal_point;
    snapshot.frozen_pre_grasp_point_base = input.frozen_pre_grasp_point;
    snapshot.tracked_pre_grasp_candidate_base = input.tracked_pre_grasp_candidate;

    snapshot.has_current_arm_point = input.has_current_arm_point;
    snapshot.current_arm_point_base = input.current_arm_point;
    snapshot.cartesian_error_to_goal_m = cartesianErrorToGoal(input);
    snapshot.freeze_cartesian_tol_m = input.freeze_cartesian_tol_m;
    snapshot.cartesian_goal_reached = snapshot.has_current_arm_point &&
                                      std::isfinite(snapshot.cartesian_error_to_goal_m) &&
                                      snapshot.cartesian_error_to_goal_m <= input.freeze_cartesian_tol_m;
    snapshot.freeze_ready = input.has_locked_object_point &&
                            input.joint_goal_reached &&
                            snapshot.cartesian_goal_reached;
    snapshot.hold_frozen = input.hold_frozen;
    snapshot.hold_target_drift_m = holdTargetDrift(input);
    snapshot.hold_target_exit_tol_m = input.hold_target_exit_tol_m;
    snapshot.grasp_start_requested = input.grasp_start_requested;

    snapshot.has_tof_range = input.has_tof_range;
    snapshot.tof_range_m = input.tof_range_m;
    snapshot.tof_age_sec = input.tof_age_sec;
    snapshot.desired_grasp_range_m = input.desired_grasp_range_m;
    snapshot.gripper_inside_range_m = input.gripper_inside_range_m;
    snapshot.gripper_inner_extra_approach_m = input.gripper_inner_extra_approach_m;
    snapshot.tof_approach_step_m = input.tof_approach_step_m;
    snapshot.tof_forward_distance_m = input.tof_debug.forward_distance_m;
    snapshot.tof_target_inside_gripper = input.tof_target_inside_gripper;
    snapshot.has_tof_approach_point = input.tof_debug.has_approach_point;
    snapshot.tof_approach_point_base = input.tof_debug.approach_point;

    snapshot.has_frozen_pre_grasp_joints = input.has_frozen_pre_grasp_joints;
    snapshot.shoulder_pitch_delta_from_hold_rad = shoulderPitchDeltaFromHold(input);
    snapshot.elbow_delta_from_hold_rad = elbowDeltaFromHold(input);
    snapshot.shoulder_to_elbow_delta_ratio = shoulderToElbowDeltaRatio(input);

    snapshot.current_joint_positions = input.current_joint_positions;
    snapshot.target_joint_positions = input.target_joint_positions;
    snapshot.joint_error_sum = input.joint_error_sum;
    snapshot.joint_reach_tol_rad = input.joint_reach_tol_rad;
    snapshot.joint_goal_reached = input.joint_goal_reached;

    snapshot.hold_elapsed_sec = holdElapsedSec(input);
    snapshot.hold_time_sec = input.hold_time_sec;

    snapshot.target_lock_samples = input.target_lock_samples;
    snapshot.target_lock_min_samples = input.target_lock_min_samples;
    snapshot.target_lock_max_deviation_m = input.target_lock_max_deviation_m;
    snapshot.target_lock_pos_tol_m = input.target_lock_pos_tol_m;
    snapshot.target_lock_max_bearing_deviation_rad = input.target_lock_max_bearing_deviation_rad;

    snapshot.tof_active_probe_index = input.tof_debug.active_probe_index;
    snapshot.tof_probe_failed_count = input.tof_debug.probe_failed_count;
    snapshot.tof_approach_blocked = input.tof_debug.approach_blocked;
    snapshot.last_limit_rejection = input.last_limit_rejection;
    return snapshot;
}

double ArmDebugSnapshotBuilder::cartesianErrorToGoal(const DebugSnapshotInput &input)
{
    if (!input.has_commanded_goal_point || !input.has_current_arm_point)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double dx = input.commanded_goal_point.x - input.current_arm_point.x;
    const double dy = input.commanded_goal_point.y - input.current_arm_point.y;
    const double dz = input.commanded_goal_point.z - input.current_arm_point.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double ArmDebugSnapshotBuilder::pointDistance(const TargetPoint &a, const TargetPoint &b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double ArmDebugSnapshotBuilder::wrapPi(double angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double ArmDebugSnapshotBuilder::shoulderPitchDeltaFromHold(const DebugSnapshotInput &input)
{
    if (!input.has_frozen_pre_grasp_joints || input.frozen_pre_grasp_joint_positions.size() <= 1 ||
        input.target_joint_positions.size() <= 1)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return wrapPi(input.target_joint_positions[1] - input.frozen_pre_grasp_joint_positions[1]);
}

double ArmDebugSnapshotBuilder::elbowDeltaFromHold(const DebugSnapshotInput &input)
{
    if (!input.has_frozen_pre_grasp_joints || input.frozen_pre_grasp_joint_positions.size() <= 2 ||
        input.target_joint_positions.size() <= 2)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return wrapPi(input.target_joint_positions[2] - input.frozen_pre_grasp_joint_positions[2]);
}

double ArmDebugSnapshotBuilder::shoulderToElbowDeltaRatio(const DebugSnapshotInput &input)
{
    const double d_sh = shoulderPitchDeltaFromHold(input);
    const double d_el = elbowDeltaFromHold(input);
    if (!std::isfinite(d_sh) || !std::isfinite(d_el))
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double abs_el = std::fabs(d_el);
    if (abs_el <= 1e-6)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return std::fabs(d_sh) / abs_el;
}

double ArmDebugSnapshotBuilder::holdElapsedSec(const DebugSnapshotInput &input)
{
    if (input.phase != AutoPhase::PRE_GRASP_HOLD || input.hold_start_time_sec < 0.0)
    {
        return 0.0;
    }
    return input.now_sec - input.hold_start_time_sec;
}

double ArmDebugSnapshotBuilder::holdTargetDrift(const DebugSnapshotInput &input)
{
    if (!input.hold_frozen || !input.has_tracked_pre_grasp_candidate)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return pointDistance(input.tracked_pre_grasp_candidate, input.frozen_pre_grasp_point);
}

}  // namespace gimbal_mani::arm_upper
