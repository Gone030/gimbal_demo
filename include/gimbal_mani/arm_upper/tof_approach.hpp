#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "gimbal_mani/arm_upper/arm_params.hpp"
#include "gimbal_mani/arm_upper/arm_types.hpp"

namespace gimbal_mani::arm_upper
{

enum class TofApproachResult
{
    RUNNING,
    TARGET_INSIDE,
    BLOCKED,
    FAILED,
};

struct TofInitRequest
{
    std::vector<double> goal;
    std::vector<double> frozen_pre_grasp_joint_positions;
    bool has_frozen_pre_grasp_joints{false};
    bool range_valid{false};
    double latest_range_m{0.0};
    double current_forward_progress_m{0.0};
    size_t joint_count{0};
    double gripper_open_pos{0.0};
};

struct TofStepRequest
{
    std::vector<double> goal;
    bool range_valid{false};
    double latest_range_m{0.0};
    double current_forward_progress_m{0.0};
    bool target_inside_gripper{false};
    bool probe_goal_reached{false};
    double now_sec{0.0};
    size_t joint_count{0};
    double gripper_open_pos{0.0};
};

struct TofStepOutput
{
    TofApproachResult result{TofApproachResult::RUNNING};
    std::vector<double> goal;
    bool has_goal{false};
    bool target_valid{false};
    bool has_command_time{false};
    double command_time_sec{0.0};
};

struct TofDebugState
{
    double forward_distance_m{0.0};
    bool has_approach_point{false};
    TargetPoint approach_point;
    int active_probe_index{-1};
    int probe_failed_count{0};
    bool approach_blocked{false};
};

class TofApproach
{
public:
    TofApproach();
    explicit TofApproach(const TofApproachParams &params);

    void setParams(const TofApproachParams &params);
    void setLogger(rclcpp::Logger logger);
    void reset();

    bool init(
        const TofInitRequest &request,
        const std::function<bool(const std::vector<double> &)> &within_joint_limits);

    TofStepOutput step(
        const TofStepRequest &request,
        const std::function<bool(const std::vector<double> &)> &within_joint_limits,
        const std::function<double(size_t, double)> &display_joint_value);

    bool holdLastSafeGoal(
        size_t joint_count,
        double gripper_close_pos,
        std::vector<double> &goal_out) const;
    void setLastSafeGoal(const std::vector<double> &goal);

    bool targetInsideGripper(bool range_valid, double latest_range_m) const;
    void stopAndHold(const TargetPoint &frozen_pre_grasp_point);
    bool isBlocked() const;
    TofDebugState debugState() const;

    double currentForwardProgress(
        bool has_frozen_pre_grasp_point,
        bool has_frozen_approach_axis,
        const TargetPoint &current,
        const TargetPoint &frozen_pre_grasp_point,
        const TargetVector &frozen_approach_axis) const;

private:
    struct ProbeCandidate
    {
        double shoulder_scale{0.0};
        double elbow_scale{0.0};
        double wrist_scale{0.0};
    };

    std::vector<ProbeCandidate> probeCandidates() const;
    TofStepOutput issueProbe(
        const TofStepRequest &request,
        const std::function<bool(const std::vector<double> &)> &within_joint_limits,
        const std::function<double(size_t, double)> &display_joint_value);
    TofStepOutput evaluateProbe(const TofStepRequest &request);
    TargetVector signedApproachAxis(const TargetVector &frozen_approach_axis) const;

    static double clamp(double value, double lo, double hi);

    TofApproachParams params_;
    rclcpp::Logger logger_{rclcpp::get_logger("tof_approach")};

    std::vector<double> last_safe_joint_positions_;
    bool pending_probe_{false};
    int active_probe_index_{-1};
    int preferred_probe_index_{0};
    uint32_t probe_failed_mask_{0};
    int probe_failed_count_{0};
    double probe_start_range_m_{0.0};
    double probe_start_progress_m_{0.0};
    double probe_start_time_sec_{-1.0};

    double forward_distance_m_{0.0};
    double last_approach_range_m_{0.0};
    double last_forward_progress_m_{0.0};
    bool approach_faulted_{false};
    bool approach_blocked_{false};
    bool reverse_retry_used_{false};
    double approach_axis_sign_{1.0};
    TargetPoint approach_point_;
    bool has_approach_point_{false};
};

}  // namespace gimbal_mani::arm_upper
