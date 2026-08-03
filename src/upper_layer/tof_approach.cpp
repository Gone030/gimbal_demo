#include "gimbal_mani/arm_upper/tof_approach.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gimbal_mani::arm_upper
{

TofApproach::TofApproach() = default;

TofApproach::TofApproach(const TofApproachParams &params)
    : params_(params)
{
}

void TofApproach::setParams(const TofApproachParams &params)
{
    params_ = params;
}

void TofApproach::setLogger(rclcpp::Logger logger)
{
    logger_ = logger;
}

void TofApproach::reset()
{
    forward_distance_m_ = 0.0;
    last_approach_range_m_ = std::numeric_limits<double>::quiet_NaN();
    last_forward_progress_m_ = std::numeric_limits<double>::quiet_NaN();
    approach_faulted_ = false;
    approach_blocked_ = false;
    reverse_retry_used_ = false;
    approach_axis_sign_ = 1.0;
    last_safe_joint_positions_.clear();
    pending_probe_ = false;
    active_probe_index_ = -1;
    preferred_probe_index_ = 0;
    probe_failed_mask_ = 0;
    probe_failed_count_ = 0;
    probe_start_range_m_ = std::numeric_limits<double>::quiet_NaN();
    probe_start_progress_m_ = std::numeric_limits<double>::quiet_NaN();
    probe_start_time_sec_ = -1.0;
    approach_point_ = {};
    has_approach_point_ = false;
}

bool TofApproach::init(
    const TofInitRequest &request,
    const std::function<bool(const std::vector<double> &)> &within_joint_limits)
{
    if (!request.has_frozen_pre_grasp_joints ||
        request.frozen_pre_grasp_joint_positions.size() != request.joint_count ||
        !request.range_valid)
    {
        return false;
    }

    last_safe_joint_positions_ = request.goal;
    if (last_safe_joint_positions_.size() != request.joint_count)
    {
        last_safe_joint_positions_ = request.frozen_pre_grasp_joint_positions;
    }
    if (last_safe_joint_positions_.size() != request.joint_count)
    {
        return false;
    }
    last_safe_joint_positions_[4] = 0.0;
    last_safe_joint_positions_[5] = request.gripper_open_pos;
    if (!within_joint_limits(last_safe_joint_positions_))
    {
        return false;
    }
    pending_probe_ = false;
    probe_failed_mask_ = 0;
    probe_failed_count_ = 0;
    preferred_probe_index_ = 0;
    approach_faulted_ = false;
    approach_blocked_ = false;
    forward_distance_m_ = 0.0;
    last_approach_range_m_ = request.latest_range_m;
    last_forward_progress_m_ = request.current_forward_progress_m;
    return true;
}

TofStepOutput TofApproach::step(
    const TofStepRequest &request,
    const std::function<bool(const std::vector<double> &)> &within_joint_limits,
    const std::function<double(size_t, double)> &display_joint_value)
{
    if (approach_blocked_)
    {
        return {TofApproachResult::BLOCKED};
    }

    if (pending_probe_)
    {
        return evaluateProbe(request);
    }

    if (last_safe_joint_positions_.size() == request.joint_count &&
        !request.probe_goal_reached)
    {
        TofStepOutput out;
        out.goal = last_safe_joint_positions_;
        out.goal[4] = 0.0;
        out.goal[5] = request.gripper_open_pos;
        out.has_goal = true;
        out.target_valid = within_joint_limits(out.goal);
        out.result = out.target_valid ? TofApproachResult::RUNNING : TofApproachResult::FAILED;
        return out;
    }

    return issueProbe(request, within_joint_limits, display_joint_value);
}

bool TofApproach::holdLastSafeGoal(
    size_t joint_count,
    double gripper_close_pos,
    std::vector<double> &goal_out) const
{
    if (last_safe_joint_positions_.size() != joint_count)
    {
        return false;
    }

    goal_out = last_safe_joint_positions_;
    goal_out[4] = 0.0;
    goal_out[5] = gripper_close_pos;
    return true;
}

void TofApproach::setLastSafeGoal(const std::vector<double> &goal)
{
    last_safe_joint_positions_ = goal;
}

bool TofApproach::targetInsideGripper(bool range_valid, double latest_range_m) const
{
    return range_valid &&
           latest_range_m <= params_.gripper_inside_range_m;
}

void TofApproach::stopAndHold(const TargetPoint &frozen_pre_grasp_point)
{
    forward_distance_m_ = 0.0;
    approach_point_ = frozen_pre_grasp_point;
    has_approach_point_ = false;
}

bool TofApproach::isBlocked() const
{
    return approach_blocked_;
}

TofDebugState TofApproach::debugState() const
{
    TofDebugState state;
    state.forward_distance_m = forward_distance_m_;
    state.has_approach_point = has_approach_point_;
    state.approach_point = approach_point_;
    state.active_probe_index = active_probe_index_;
    state.probe_failed_count = probe_failed_count_;
    state.approach_blocked = approach_blocked_;
    return state;
}

double TofApproach::currentForwardProgress(
    bool has_frozen_pre_grasp_point,
    bool has_frozen_approach_axis,
    const TargetPoint &current,
    const TargetPoint &frozen_pre_grasp_point,
    const TargetVector &frozen_approach_axis) const
{
    if (!has_frozen_pre_grasp_point || !has_frozen_approach_axis)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    TargetVector axis = signedApproachAxis(frozen_approach_axis);
    return (current.x - frozen_pre_grasp_point.x) * axis.x +
           (current.y - frozen_pre_grasp_point.y) * axis.y +
           (current.z - frozen_pre_grasp_point.z) * axis.z;
}

std::vector<TofApproach::ProbeCandidate> TofApproach::probeCandidates() const
{
    const double sh = params_.joint_probe_shoulder_ratio;
    const double wr = params_.joint_probe_wrist_ratio;
    return {
        {0.0, 1.0, -wr},
        {0.0, 1.0, -0.5 * wr},
        {sh, 1.0, -wr},
        {-sh, 1.0, -wr},
        {0.0, 0.5, -wr},
        {0.0, 0.0, -wr}};
}

TofStepOutput TofApproach::issueProbe(
    const TofStepRequest &request,
    const std::function<bool(const std::vector<double> &)> &within_joint_limits,
    const std::function<double(size_t, double)> &display_joint_value)
{
    if (last_safe_joint_positions_.size() != request.joint_count ||
        !request.range_valid)
    {
        approach_blocked_ = true;
        return {TofApproachResult::BLOCKED};
    }

    const auto probes = probeCandidates();
    const size_t n = probes.size();
    if (n == 0)
    {
        approach_blocked_ = true;
        return {TofApproachResult::BLOCKED};
    }

    const double step = std::max(0.0, params_.joint_probe_step_rad);
    for (size_t k = 0; k < n; ++k)
    {
        const size_t idx = (static_cast<size_t>(preferred_probe_index_) + k) % n;
        if ((probe_failed_mask_ & (1u << idx)) != 0)
        {
            continue;
        }

        auto candidate = last_safe_joint_positions_;
        candidate[1] += probes[idx].shoulder_scale * step;
        candidate[2] += probes[idx].elbow_scale * step;
        candidate[3] += probes[idx].wrist_scale * step;
        candidate[4] = 0.0;
        candidate[5] = request.gripper_open_pos;

        if (!within_joint_limits(candidate))
        {
            probe_failed_mask_ |= (1u << idx);
            ++probe_failed_count_;
            continue;
        }

        pending_probe_ = true;
        active_probe_index_ = static_cast<int>(idx);
        probe_start_time_sec_ = request.now_sec;
        probe_start_range_m_ = request.latest_range_m;
        probe_start_progress_m_ = request.current_forward_progress_m;
        RCLCPP_INFO(
            logger_,
            "TOF joint probe idx=%zu sh=%.1f el=%.1f wrist=%.1f start_range=%.4f",
            idx,
            display_joint_value(1, candidate[1]),
            display_joint_value(2, candidate[2]),
            display_joint_value(3, candidate[3]),
            probe_start_range_m_);

        TofStepOutput out;
        out.goal = candidate;
        out.has_goal = true;
        out.target_valid = true;
        out.has_command_time = true;
        out.command_time_sec = params_.joint_probe_command_time_sec;
        return out;
    }

    approach_blocked_ = true;
    return {TofApproachResult::BLOCKED};
}

TofStepOutput TofApproach::evaluateProbe(const TofStepRequest &request)
{
    TofStepOutput out;
    out.goal = request.goal;
    if (out.goal.size() > 5)
    {
        out.goal[5] = request.gripper_open_pos;
        out.has_goal = true;
        out.target_valid = true;
    }

    const double elapsed = request.now_sec - probe_start_time_sec_;
    if (elapsed < params_.joint_probe_command_time_sec)
    {
        return out;
    }

    if (!request.probe_goal_reached)
    {
        return out;
    }

    if (!request.range_valid)
    {
        return out;
    }

    const double range_delta = probe_start_range_m_ - request.latest_range_m;
    const double progress = request.current_forward_progress_m;
    const double progress_delta =
        std::isfinite(progress) && std::isfinite(probe_start_progress_m_)
            ? progress - probe_start_progress_m_
            : std::numeric_limits<double>::quiet_NaN();
    const bool range_increased = range_delta <= -params_.fail_range_increase_m;

    if (request.target_inside_gripper || range_delta >= params_.success_range_delta_m)
    {
        last_safe_joint_positions_ = out.goal;
        pending_probe_ = false;
        preferred_probe_index_ = std::max(0, active_probe_index_);
        probe_failed_mask_ = 0;
        probe_failed_count_ = 0;
        forward_distance_m_ = clamp(
            forward_distance_m_ + std::max(0.0, params_.approach_step_m),
            params_.min_forward_approach_m,
            params_.max_forward_approach_m);
        last_approach_range_m_ = request.latest_range_m;
        last_forward_progress_m_ = progress;
        RCLCPP_INFO(
            logger_,
            "TOF joint probe accepted idx=%d range_delta=%.4f progress_delta=%.4f",
            active_probe_index_, range_delta, progress_delta);
        return out;
    }

    if (active_probe_index_ >= 0)
    {
        probe_failed_mask_ |= (1u << static_cast<size_t>(active_probe_index_));
    }
    ++probe_failed_count_;
    out.goal = last_safe_joint_positions_;
    if (out.goal.size() > 5)
    {
        out.goal[4] = 0.0;
        out.goal[5] = request.gripper_open_pos;
    }
    out.has_goal = true;
    out.target_valid = true;
    pending_probe_ = false;
    RCLCPP_WARN(
        logger_,
        "TOF joint probe rejected idx=%d range_delta=%.4f progress_delta=%.4f increased=%d failed=%d",
        active_probe_index_, range_delta, progress_delta,
        range_increased ? 1 : 0, probe_failed_count_);

    if (probe_failed_count_ >= params_.max_failed_probe_count)
    {
        approach_blocked_ = true;
        out.result = TofApproachResult::BLOCKED;
        return out;
    }

    return out;
}

TargetVector TofApproach::signedApproachAxis(const TargetVector &frozen_approach_axis) const
{
    return {
        approach_axis_sign_ * frozen_approach_axis.x,
        approach_axis_sign_ * frozen_approach_axis.y,
        approach_axis_sign_ * frozen_approach_axis.z};
}

double TofApproach::clamp(double value, double lo, double hi)
{
    return std::max(lo, std::min(hi, value));
}

}  // namespace gimbal_mani::arm_upper
