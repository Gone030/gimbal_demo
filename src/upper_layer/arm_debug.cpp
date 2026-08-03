#include "gimbal_mani/arm_upper/arm_debug.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace gimbal_mani::arm_upper
{

const char *ArmDebug::phaseName(AutoPhase phase)
{
    switch (phase)
    {
    case AutoPhase::IDLE:
        return "IDLE";
    case AutoPhase::TARGET_ACQUIRE:
        return "TARGET_ACQUIRE";
    case AutoPhase::TRACK_PRE_GRASP:
        return "TRACK_PRE_GRASP";
    case AutoPhase::PRE_GRASP_HOLD:
        return "PRE_GRASP_HOLD";
    case AutoPhase::GRIPPER_OPEN:
        return "GRIPPER_OPEN";
    case AutoPhase::TOF_APPROACH:
        return "TOF_APPROACH";
    case AutoPhase::GRIPPER_CLOSE:
        return "GRIPPER_CLOSE";
    case AutoPhase::LIFT:
        return "LIFT";
    case AutoPhase::GRASP_HOLD:
        return "GRASP_HOLD";
    }
    return "UNKNOWN";
}

gimbal_mani::msg::ArmAutoDebug ArmDebug::buildDebugMessage(const ArmDebugSnapshot &snapshot)
{
    gimbal_mani::msg::ArmAutoDebug msg;
    msg.header.stamp = snapshot.stamp;
    msg.header.frame_id = snapshot.base_frame;

    msg.auto_enabled = snapshot.auto_enabled;
    msg.phase = phaseName(snapshot.phase);

    msg.target_valid = snapshot.target_valid;
    msg.has_joint_state = snapshot.has_joint_state;
    msg.has_locked_object_point = snapshot.has_locked_object_point;

    msg.base_frame = snapshot.base_frame;
    msg.end_effector_frame = snapshot.end_effector_frame;

    msg.object_point_base = toPointMsg(snapshot.object_point_base);
    msg.pre_grasp_hold_point_base = toPointMsg(snapshot.pre_grasp_hold_point_base);
    msg.commanded_goal_point_base = toPointMsg(snapshot.commanded_goal_point_base);
    msg.frozen_pre_grasp_point_base = toPointMsg(snapshot.frozen_pre_grasp_point_base);
    msg.tracked_pre_grasp_candidate_base = toPointMsg(snapshot.tracked_pre_grasp_candidate_base);

    msg.has_current_arm_point = snapshot.has_current_arm_point;
    msg.current_arm_point_base = toPointMsg(snapshot.current_arm_point_base);
    msg.cartesian_error_to_goal_m = snapshot.cartesian_error_to_goal_m;
    msg.freeze_cartesian_tol_m = snapshot.freeze_cartesian_tol_m;
    msg.cartesian_goal_reached = snapshot.cartesian_goal_reached;
    msg.freeze_ready = snapshot.freeze_ready;
    msg.hold_frozen = snapshot.hold_frozen;
    msg.hold_target_drift_m = snapshot.hold_target_drift_m;
    msg.hold_target_exit_tol_m = snapshot.hold_target_exit_tol_m;
    msg.grasp_start_requested = snapshot.grasp_start_requested;
    msg.has_tof_range = snapshot.has_tof_range;
    msg.tof_range_m = snapshot.tof_range_m;
    msg.tof_age_sec = snapshot.tof_age_sec;
    msg.desired_grasp_range_m = snapshot.desired_grasp_range_m;
    msg.gripper_inside_range_m = snapshot.gripper_inside_range_m;
    msg.gripper_inner_extra_approach_m = snapshot.gripper_inner_extra_approach_m;
    msg.tof_approach_step_m = snapshot.tof_approach_step_m;
    msg.tof_forward_distance_m = snapshot.tof_forward_distance_m;
    msg.tof_target_inside_gripper = snapshot.tof_target_inside_gripper;
    msg.has_tof_approach_point = snapshot.has_tof_approach_point;
    msg.tof_approach_point_base = toPointMsg(snapshot.tof_approach_point_base);
    msg.has_frozen_pre_grasp_joints = snapshot.has_frozen_pre_grasp_joints;
    msg.shoulder_pitch_delta_from_hold_rad = snapshot.shoulder_pitch_delta_from_hold_rad;
    msg.elbow_delta_from_hold_rad = snapshot.elbow_delta_from_hold_rad;
    msg.shoulder_to_elbow_delta_ratio = snapshot.shoulder_to_elbow_delta_ratio;

    msg.current_joint_positions = snapshot.current_joint_positions;
    msg.target_joint_positions = snapshot.target_joint_positions;
    msg.joint_error_sum = snapshot.joint_error_sum;
    msg.joint_reach_tol_rad = snapshot.joint_reach_tol_rad;
    msg.joint_goal_reached = snapshot.joint_goal_reached;

    msg.hold_elapsed_sec = snapshot.hold_elapsed_sec;
    msg.hold_time_sec = snapshot.hold_time_sec;

    msg.target_lock_samples = snapshot.target_lock_samples;
    msg.target_lock_min_samples = snapshot.target_lock_min_samples;
    msg.target_lock_max_deviation_m = snapshot.target_lock_max_deviation_m;
    msg.target_lock_pos_tol_m = snapshot.target_lock_pos_tol_m;

    return msg;
}

std_msgs::msg::String ArmDebug::buildStatusMessage(const ArmDebugSnapshot &snapshot)
{
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3)
       << "phase=" << phaseName(snapshot.phase)
       << " tof=" << snapshot.tof_range_m
       << " in=" << (snapshot.tof_target_inside_gripper ? 1 : 0)
       << " fwd=" << snapshot.tof_forward_distance_m
       << " probe=" << snapshot.tof_active_probe_index
       << " fails=" << snapshot.tof_probe_failed_count
       << " block=" << (snapshot.tof_approach_blocked ? 1 : 0)
       << " bear=" << snapshot.target_lock_max_bearing_deviation_rad
       << " limit=" << (snapshot.last_limit_rejection.empty() ? "ok" : snapshot.last_limit_rejection)
       << " sh=" << snapshot.shoulder_pitch_delta_from_hold_rad
       << " el=" << snapshot.elbow_delta_from_hold_rad
       << " r=" << snapshot.shoulder_to_elbow_delta_ratio;

    std_msgs::msg::String msg;
    msg.data = ss.str();
    return msg;
}

geometry_msgs::msg::Point ArmDebug::toPointMsg(const TargetPoint &p)
{
    geometry_msgs::msg::Point out;
    out.x = p.x;
    out.y = p.y;
    out.z = p.z;
    return out;
}

}  // namespace gimbal_mani::arm_upper
