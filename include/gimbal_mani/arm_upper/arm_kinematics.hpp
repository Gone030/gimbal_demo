#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "gimbal_mani/arm_upper/arm_params.hpp"
#include "gimbal_mani/arm_upper/arm_types.hpp"

namespace gimbal_mani::arm_upper
{

struct LimitCheckResult
{
    bool ok{false};
    std::string rejection;
    size_t joint_index{0};
    double value{0.0};
    double min{0.0};
    double max{0.0};
};

struct IkRequest
{
    TargetPoint target_point;
    TargetVector target_axis;
    std::vector<double> current_goal;
    std::vector<double> measured_joints;
    double wrist_roll_ref{0.0};
    bool use_tof_approach_ik_bias{false};
    bool has_frozen_pre_grasp_joints{false};
    std::vector<double> frozen_pre_grasp_joint_positions;
    bool has_frozen_pre_grasp_point{false};
    TargetPoint frozen_pre_grasp_point;
    bool has_locked_object_point{false};
    TargetPoint locked_object_point;
    std::string phase_name{"UNKNOWN"};
};

struct IkResult
{
    std::vector<double> goal;
    double position_error_m{0.0};
    double score{0.0};
    std::string selected_candidate{"none"};
    std::string limit_rejection;
};

struct GraspFkResult
{
    TargetPoint position;
    TargetVector approach_axis;
};

class ArmKinematics
{
public:
    ArmKinematics();
    explicit ArmKinematics(const KinematicsParams &params);

    void setParams(const KinematicsParams &params);
    void setLogger(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

    std::optional<IkResult> solveGoalFromPoint(const IkRequest &request);

    LimitCheckResult checkJointLimits(const std::vector<double> &goal) const;
    bool withinJointLimits(const std::vector<double> &goal) const;
    double goalErrorSum(const std::vector<double> &goal, const std::vector<double> &measured) const;
    bool goalReached(
        const std::vector<double> &goal,
        const std::vector<double> &measured,
        double tol_rad) const;
    GraspFkResult forwardGripperPoint(const std::vector<double> &q) const;
    TargetPoint wristToGraspPoint(const TargetPoint &wrist_point, const TargetVector &approach_axis) const;
    TargetPoint graspToWristTarget(const TargetPoint &grasp_point, const TargetVector &approach_axis) const;

    static double pointDistance(const TargetPoint &a, const TargetPoint &b);
    static TargetVector normalizedVector(const TargetVector &v, const TargetVector &fallback);
    static TargetVector crossVector(const TargetVector &a, const TargetVector &b);
    static double wrapPi(double angle);
    static double clamp(double value, double lo, double hi);
    static double nearestEquivalentAngle(double target, double reference);
    static double displayJointValue(size_t index, double value);
    static const char *jointDisplayUnit(size_t index);

private:
    struct IkCandidateDebug;

    bool refineGraspCandidate(
        const TargetPoint &target_position,
        const TargetVector &target_axis,
        std::vector<double> &candidate,
        double &position_error,
        double &axis_error,
        double &yaw_error,
        double &pitch_error,
        int &iterations,
        std::string &last_limit_rejection) const;
    TargetPoint fkPointFromSolution(double psi_raw, double shoulder_joint, double elbow_joint) const;
    bool wristToGraspOffsetBase(const TargetVector &approach_axis, TargetVector &offset) const;
    void snapSeedToJointLimits(std::vector<double> &seed) const;
    void logIkCandidates(const std::vector<IkCandidateDebug> &candidates, const std::string &selected, const std::string &phase_name) const;

    static double ikElbowToJointElbow(double ik_elbow);
    static double jointElbowToIkElbow(double joint_elbow);
    static double ikShoulderToJointShoulder(double ik_shoulder, double joint_elbow);
    static double jointShoulderToIkShoulder(double joint_shoulder, double joint_elbow);
    static double vectorAngle(const TargetVector &a, const TargetVector &b);
    static double vectorYaw(const TargetVector &v);
    static double vectorPitch(const TargetVector &v);
    static double vectorYawError(const TargetVector &actual, const TargetVector &target);
    static double vectorPitchError(const TargetVector &actual, const TargetVector &target);
    static bool solveLinear4x4(
        std::array<std::array<double, 4>, 4> a,
        std::array<double, 4> b,
        std::array<double, 4> &x);

    KinematicsParams params_;
    rclcpp::Logger logger_{rclcpp::get_logger("arm_kinematics")};
    rclcpp::Clock::SharedPtr clock_;
};

}  // namespace gimbal_mani::arm_upper
