#include "gimbal_mani/arm_upper/arm_kinematics.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include "gimbal_mani/ik_2link_2d.hpp"

namespace gimbal_mani::arm_upper
{

struct ArmKinematics::IkCandidateDebug
{
    std::string name{"unknown"};
    bool available{false};
    bool valid{false};
    double rotation{0.0};
    double shoulder{0.0};
    double elbow{0.0};
    double wrist{0.0};
    double position_error{std::numeric_limits<double>::quiet_NaN()};
    double axis_error{std::numeric_limits<double>::quiet_NaN()};
    double yaw_error{std::numeric_limits<double>::quiet_NaN()};
    double pitch_error{std::numeric_limits<double>::quiet_NaN()};
    int iterations{0};
    std::string rejection{"none"};
    double cost{std::numeric_limits<double>::quiet_NaN()};
};

ArmKinematics::ArmKinematics() = default;

ArmKinematics::ArmKinematics(const KinematicsParams &params)
    : params_(params)
{
}

void ArmKinematics::setParams(const KinematicsParams &params)
{
    params_ = params;
}

void ArmKinematics::setLogger(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
{
    logger_ = logger;
    clock_ = std::move(clock);
}

std::optional<IkResult> ArmKinematics::solveGoalFromPoint(const IkRequest &request)
{
    if (request.current_goal.size() != params_.joint_names.size() ||
        request.measured_joints.size() != params_.joint_names.size())
    {
        return std::nullopt;
    }

    const TargetPoint wrist_target = graspToWristTarget(request.target_point, request.target_axis);
    const double x = wrist_target.x;
    const double y = wrist_target.y;
    const double z = wrist_target.z;

    const double x_sr = x - params_.shoulder_rot_x;
    const double y_sr = y - params_.shoulder_rot_y;
    const double z_sr = z - params_.shoulder_rot_z;

    const double psi_raw = std::atan2(x_sr, -y_sr);
    const double psi = nearestEquivalentAngle(psi_raw, request.measured_joints[0]);

    const double c = std::cos(psi_raw);
    const double s = std::sin(psi_raw);

    const double y_plane = z_sr;
    const double z_plane = s * x_sr - c * y_sr;

    double rho = z_plane - params_.shoulder_pitch_offset_y;
    double z2 = y_plane - params_.shoulder_pitch_offset_z;

    if (params_.saturate_reach)
    {
        const double rr = std::sqrt((rho * rho) + (z2 * z2));
        if (rr >= 1e-9)
        {
            const double r_max = (params_.L1 + params_.L2);
            const double r_min = std::fabs(params_.L1 - params_.L2);
            double rr_sat = rr;
            if (rr > r_max)
            {
                rr_sat = r_max;
            }
            else if (rr < r_min)
            {
                rr_sat = r_min;
            }

            if (rr_sat != rr)
            {
                const double scale = rr_sat / rr;
                rho *= scale;
                z2 *= scale;
            }
        }
    }

    double th1_up = 0.0, th2_up = 0.0;
    double th1_dn = 0.0, th2_dn = 0.0;
    const bool ok_up = ik_2link_2d(rho, z2, params_.L1, params_.L2, +1, th1_up, th2_up);
    const bool ok_dn = ik_2link_2d(rho, z2, params_.L1, params_.L2, -1, th1_dn, th2_dn);

    if (!ok_up && !ok_dn)
    {
        if (clock_)
        {
            RCLCPP_WARN_THROTTLE(
                logger_, *clock_, 2000,
                "IK unreachable: rho=%.4f z=%.4f (L1=%.3f L2=%.3f)",
                rho, z2, params_.L1, params_.L2);
        }
        return std::nullopt;
    }

    const double prev_shoulder_pitch = request.measured_joints[1];
    const double prev_elbow = request.measured_joints[2];

    const double elbow_up = nearestEquivalentAngle(ikElbowToJointElbow(th2_up), prev_elbow);
    const double shoulder_pitch_up =
        nearestEquivalentAngle(ikShoulderToJointShoulder(th1_up, elbow_up), prev_shoulder_pitch);
    const double elbow_dn = nearestEquivalentAngle(ikElbowToJointElbow(th2_dn), prev_elbow);
    const double shoulder_pitch_dn =
        nearestEquivalentAngle(ikShoulderToJointShoulder(th1_dn, elbow_dn), prev_shoulder_pitch);

    auto cost = [&](double a1, double a2)
    {
        const double w_sh = 0.7;
        const double w_el = 1.1;
        double value = w_sh * std::fabs(a1 - prev_shoulder_pitch) +
                       w_el * std::fabs(a2 - prev_elbow);

        if (request.use_tof_approach_ik_bias && request.has_frozen_pre_grasp_joints &&
            request.frozen_pre_grasp_joint_positions.size() > 2)
        {
            const double shoulder_delta =
                std::fabs(wrapPi(a1 - request.frozen_pre_grasp_joint_positions[1]));
            const double elbow_delta =
                std::fabs(wrapPi(a2 - request.frozen_pre_grasp_joint_positions[2]));
            const double elbow_only =
                std::max(0.0, elbow_delta - params_.tof_ik_shoulder_balance_ratio * shoulder_delta);

            value += params_.tof_ik_elbow_only_penalty * elbow_only;
            value -= params_.tof_ik_shoulder_bonus * shoulder_delta;
        }

        if (request.use_tof_approach_ik_bias && request.has_frozen_pre_grasp_point &&
            request.has_locked_object_point)
        {
            const auto candidate_point =
                wristToGraspPoint(fkPointFromSolution(psi_raw, a1, a2), request.target_axis);
            const double axis_x = request.locked_object_point.x - request.frozen_pre_grasp_point.x;
            const double axis_y = request.locked_object_point.y - request.frozen_pre_grasp_point.y;
            const double axis_z = request.locked_object_point.z - request.frozen_pre_grasp_point.z;
            const double axis_n = std::sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
            if (axis_n > 1e-6)
            {
                const double progress =
                    ((candidate_point.x - request.frozen_pre_grasp_point.x) * axis_x +
                     (candidate_point.y - request.frozen_pre_grasp_point.y) * axis_y +
                     (candidate_point.z - request.frozen_pre_grasp_point.z) * axis_z) /
                    axis_n;
                const double height_error = std::fabs(candidate_point.z - request.target_point.z);

                value -= 3.0 * progress;
                value += 4.0 * height_error;
            }
        }

        return value;
    };

    std::vector<double> best_goal;
    double best_cost = std::numeric_limits<double>::infinity();
    bool has_valid_candidate = false;
    std::string selected_candidate{"none"};
    std::string last_limit_rejection;
    double best_position_error = std::numeric_limits<double>::quiet_NaN();

    std::vector<IkCandidateDebug> candidate_debug;

    auto try_candidate = [&](const char *name, bool ok, std::vector<double> candidate)
    {
        candidate_debug.emplace_back();
        auto &debug = candidate_debug.back();
        debug.name = name;
        if (!ok)
        {
            return;
        }

        debug.available = true;
        candidate[4] = 0.0;
        debug.rotation = candidate[0];
        debug.shoulder = candidate[1];
        debug.elbow = candidate[2];
        debug.wrist = candidate[3];

        auto limit = checkJointLimits(candidate);
        if (!limit.ok)
        {
            last_limit_rejection = limit.rejection;
            debug.rejection = last_limit_rejection;
            return;
        }

        if (!refineGraspCandidate(
                request.target_point, request.target_axis, candidate,
                debug.position_error, debug.axis_error,
                debug.yaw_error, debug.pitch_error, debug.iterations,
                last_limit_rejection))
        {
            debug.rotation = candidate[0];
            debug.shoulder = candidate[1];
            debug.elbow = candidate[2];
            debug.wrist = candidate[3];
            debug.rejection = last_limit_rejection.empty()
                                  ? "pose-tolerance"
                                  : last_limit_rejection;
            return;
        }

        debug.rotation = candidate[0];
        debug.shoulder = candidate[1];
        debug.elbow = candidate[2];
        debug.wrist = candidate[3];
        const double c =
            cost(candidate[1], candidate[2]) +
            params_.numerical_ik_position_cost_weight * debug.position_error +
            params_.numerical_ik_axis_cost_weight * debug.axis_error;
        debug.valid = true;
        debug.rejection = "none";
        debug.cost = c;
        if (!has_valid_candidate || c < best_cost)
        {
            best_goal = candidate;
            best_cost = c;
            best_position_error = debug.position_error;
            has_valid_candidate = true;
            selected_candidate = name;
        }
    };

    auto seed_from_analytic = [&](double shoulder_pitch, double elbow)
    {
        auto seed = request.current_goal;
        seed[0] = psi;
        seed[1] = shoulder_pitch;
        seed[2] = elbow;
        const double wrist_pitch_raw =
            -(seed[1] + seed[2]) + params_.wrist_pitch_level_bias;
        seed[3] = nearestEquivalentAngle(wrist_pitch_raw, request.measured_joints[3]);
        (void)request.wrist_roll_ref;
        seed[4] = 0.0;
        return seed;
    };

    const auto up_seed = seed_from_analytic(shoulder_pitch_up, elbow_up);
    try_candidate("up", ok_up, up_seed);
    try_candidate(
        "dn", ok_dn, seed_from_analytic(shoulder_pitch_dn, elbow_dn));

    auto current_seed = request.measured_joints;
    if (current_seed.size() == params_.joint_names.size())
    {
        snapSeedToJointLimits(current_seed);
        current_seed[4] = 0.0;
        try_candidate("current", true, current_seed);
    }

    logIkCandidates(candidate_debug, selected_candidate, request.phase_name);

    if (!has_valid_candidate)
    {
        auto rejected = request.current_goal;
        rejected[0] = psi;
        rejected[1] = ok_up ? shoulder_pitch_up : shoulder_pitch_dn;
        rejected[2] = ok_up ? elbow_up : elbow_dn;
        rejected[3] = nearestEquivalentAngle(
            -(rejected[1] + rejected[2]) + params_.wrist_pitch_level_bias,
            request.measured_joints[3]);
        rejected[4] = 0.0;
        const auto limit = checkJointLimits(rejected);
        last_limit_rejection = limit.rejection;
        return std::nullopt;
    }

    IkResult result;
    result.goal = best_goal;
    result.position_error_m = best_position_error;
    result.score = best_cost;
    result.selected_candidate = selected_candidate;
    result.limit_rejection = checkJointLimits(best_goal).rejection;
    return result;
}

LimitCheckResult ArmKinematics::checkJointLimits(const std::vector<double> &goal) const
{
    LimitCheckResult result;
    if (goal.size() != params_.joint_names.size())
    {
        result.rejection = "goal-size";
        return result;
    }

    constexpr double eps = 1e-6;
    for (size_t i = 0; i < goal.size(); ++i)
    {
        const auto &limit = params_.joint_limits[i];
        if (goal[i] < limit.min - eps || goal[i] > limit.max + eps)
        {
            std::ostringstream ss;
            ss << params_.joint_names[i] << ":"
               << std::fixed << std::setprecision(1) << displayJointValue(i, goal[i]);
            result.rejection = ss.str();
            result.joint_index = i;
            result.value = goal[i];
            result.min = limit.min;
            result.max = limit.max;
            return result;
        }
    }

    result.ok = true;
    return result;
}

bool ArmKinematics::withinJointLimits(const std::vector<double> &goal) const
{
    return checkJointLimits(goal).ok;
}

double ArmKinematics::goalErrorSum(const std::vector<double> &goal, const std::vector<double> &measured) const
{
    if (goal.size() < 4 || measured.size() < 4)
    {
        return std::numeric_limits<double>::infinity();
    }

    return std::fabs(wrapPi(goal[0] - measured[0])) +
           std::fabs(wrapPi(goal[1] - measured[1])) +
           std::fabs(wrapPi(goal[2] - measured[2])) +
           0.5 * std::fabs(wrapPi(goal[3] - measured[3]));
}

bool ArmKinematics::goalReached(
    const std::vector<double> &goal,
    const std::vector<double> &measured,
    double tol_rad) const
{
    return goalErrorSum(goal, measured) <= tol_rad;
}

static tf2::Transform joint_origin(double x, double y, double z,
                                   double roll, double pitch, double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::Transform(q, tf2::Vector3(x, y, z));
}

static tf2::Transform axis_rotation(const tf2::Vector3 &axis, double angle)
{
    tf2::Quaternion q(axis, angle);
    return tf2::Transform(q, tf2::Vector3(0.0, 0.0, 0.0));
}

GraspFkResult ArmKinematics::forwardGripperPoint(const std::vector<double> &q) const
{
    tf2::Transform tf;
    tf.setIdentity();

    tf *= joint_origin(0.0, -0.0452, 0.0165, 1.5708, 0.0, 0.0);
    tf *= axis_rotation(tf2::Vector3(0.0, 1.0, 0.0), q[0]);

    tf *= joint_origin(0.0, 0.1025, 0.0306, 0.0, 0.0, 0.0);
    tf *= axis_rotation(tf2::Vector3(1.0, 0.0, 0.0), q[1]);

    tf *= joint_origin(0.0, 0.11257, 0.028, 0.0, 0.0, 0.0);
    tf *= axis_rotation(tf2::Vector3(1.0, 0.0, 0.0), q[2]);

    tf *= joint_origin(0.0, 0.0052, 0.1349, 0.0, 0.0, 0.0);
    tf *= axis_rotation(tf2::Vector3(1.0, 0.0, 0.0), q[3]);

    tf *= joint_origin(0.0, -0.0601, 0.0, 0.0, 0.0, 0.0);
    tf *= axis_rotation(tf2::Vector3(0.0, 1.0, 0.0), 0.0);
    tf *= joint_origin(-0.010, -0.055, -0.030, 0.0, 0.0, 0.0);

    const tf2::Vector3 position = tf.getOrigin();
    const tf2::Vector3 approach = tf.getBasis() * tf2::Vector3(0.0, -1.0, 0.0);

    GraspFkResult out;
    out.position = {position.x(), position.y(), position.z()};
    out.approach_axis = normalizedVector(
        {approach.x(), approach.y(), approach.z()},
        {0.0, -1.0, 0.0});
    return out;
}

TargetPoint ArmKinematics::wristToGraspPoint(const TargetPoint &wrist_point, const TargetVector &approach_axis) const
{
    TargetVector offset;
    if (!wristToGraspOffsetBase(approach_axis, offset))
    {
        return wrist_point;
    }

    TargetPoint out = wrist_point;
    out.x += offset.x;
    out.y += offset.y;
    out.z += offset.z;
    return out;
}

TargetPoint ArmKinematics::graspToWristTarget(const TargetPoint &grasp_point, const TargetVector &approach_axis) const
{
    TargetVector offset;
    if (!wristToGraspOffsetBase(approach_axis, offset))
    {
        return grasp_point;
    }

    TargetPoint out = grasp_point;
    out.x -= offset.x;
    out.y -= offset.y;
    out.z -= offset.z;
    return out;
}

double ArmKinematics::pointDistance(const TargetPoint &a, const TargetPoint &b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

TargetVector ArmKinematics::normalizedVector(const TargetVector &v, const TargetVector &fallback)
{
    const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (n <= 1e-9 || !std::isfinite(n))
    {
        return fallback;
    }

    return {v.x / n, v.y / n, v.z / n};
}

TargetVector ArmKinematics::crossVector(const TargetVector &a, const TargetVector &b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x};
}

double ArmKinematics::wrapPi(double angle)
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

double ArmKinematics::clamp(double value, double lo, double hi)
{
    return std::max(lo, std::min(hi, value));
}

double ArmKinematics::nearestEquivalentAngle(double target, double reference)
{
    return reference + wrapPi(target - reference);
}

double ArmKinematics::displayJointValue(size_t index, double value)
{
    if (index == 5)
    {
        return value;
    }
    return value * 180.0 / M_PI;
}

const char *ArmKinematics::jointDisplayUnit(size_t index)
{
    return index == 5 ? "raw" : "deg";
}

bool ArmKinematics::refineGraspCandidate(
    const TargetPoint &target_position,
    const TargetVector &target_axis,
    std::vector<double> &candidate,
    double &position_error,
    double &axis_error,
    double &yaw_error,
    double &pitch_error,
    int &iterations,
    std::string &last_limit_rejection) const
{
    constexpr size_t error_dim = 6;
    constexpr size_t joint_dim = 4;
    const double axis_weight = params_.numerical_ik_axis_weight_m;

    auto error_vector = [&](const std::vector<double> &q)
    {
        std::array<double, error_dim> e{};
        const auto fk = forwardGripperPoint(q);
        e[0] = target_position.x - fk.position.x;
        e[1] = target_position.y - fk.position.y;
        e[2] = target_position.z - fk.position.z;
        e[3] = axis_weight * (target_axis.x - fk.approach_axis.x);
        e[4] = axis_weight * (target_axis.y - fk.approach_axis.y);
        e[5] = axis_weight * (target_axis.z - fk.approach_axis.z);
        return e;
    };

    iterations = 0;
    for (; iterations < params_.numerical_ik_max_iterations; ++iterations)
    {
        const auto fk = forwardGripperPoint(candidate);
        position_error = pointDistance(fk.position, target_position);
        axis_error = vectorAngle(fk.approach_axis, target_axis);
        yaw_error = vectorYawError(fk.approach_axis, target_axis);
        pitch_error = vectorPitchError(fk.approach_axis, target_axis);
        if (position_error <= params_.numerical_ik_position_tol_m &&
            axis_error <= params_.numerical_ik_axis_tol_rad)
        {
            const auto limit = checkJointLimits(candidate);
            last_limit_rejection = limit.rejection;
            return limit.ok;
        }

        const auto error = error_vector(candidate);
        std::array<std::array<double, joint_dim>, error_dim> jacobian{};
        for (size_t joint = 0; joint < joint_dim; ++joint)
        {
            auto perturbed = candidate;
            perturbed[joint] += params_.numerical_ik_fd_step_rad;
            const auto perturbed_error = error_vector(perturbed);
            for (size_t row = 0; row < error_dim; ++row)
            {
                jacobian[row][joint] =
                    -(perturbed_error[row] - error[row]) / params_.numerical_ik_fd_step_rad;
            }
        }

        std::array<std::array<double, joint_dim>, joint_dim> normal{};
        std::array<double, joint_dim> rhs{};
        for (size_t row = 0; row < joint_dim; ++row)
        {
            for (size_t col = 0; col < joint_dim; ++col)
            {
                for (size_t k = 0; k < error_dim; ++k)
                {
                    normal[row][col] += jacobian[k][row] * jacobian[k][col];
                }
            }
            normal[row][row] += params_.numerical_ik_damping;
            for (size_t k = 0; k < error_dim; ++k)
            {
                rhs[row] += jacobian[k][row] * error[k];
            }
        }

        std::array<double, joint_dim> delta{};
        if (!solveLinear4x4(normal, rhs, delta))
        {
            break;
        }

        bool accepted = false;
        double current_score = 0.0;
        for (const double value : error)
        {
            current_score += value * value;
        }
        for (double scale : {1.0, 0.5, 0.25, 0.125})
        {
            auto next = candidate;
            for (size_t joint = 0; joint < joint_dim; ++joint)
            {
                const double limited_delta = clamp(
                    scale * delta[joint],
                    -params_.numerical_ik_max_step_rad,
                    params_.numerical_ik_max_step_rad);
                next[joint] = nearestEquivalentAngle(
                    next[joint] + limited_delta, candidate[joint]);
            }
            next[4] = 0.0;
            auto limit = checkJointLimits(next);
            last_limit_rejection = limit.rejection;
            if (!limit.ok)
            {
                continue;
            }

            const auto next_error = error_vector(next);
            double next_score = 0.0;
            for (const double value : next_error)
            {
                next_score += value * value;
            }
            if (next_score < current_score)
            {
                candidate = next;
                accepted = true;
                break;
            }
        }

        if (!accepted)
        {
            break;
        }
    }

    const auto fk = forwardGripperPoint(candidate);
    position_error = pointDistance(fk.position, target_position);
    axis_error = vectorAngle(fk.approach_axis, target_axis);
    yaw_error = vectorYawError(fk.approach_axis, target_axis);
    pitch_error = vectorPitchError(fk.approach_axis, target_axis);
    const auto limit = checkJointLimits(candidate);
    last_limit_rejection = limit.rejection;
    return limit.ok &&
           position_error <= params_.numerical_ik_accept_position_m &&
           axis_error <= params_.numerical_ik_accept_axis_rad;
}

TargetPoint ArmKinematics::fkPointFromSolution(double psi_raw, double shoulder_joint, double elbow_joint) const
{
    const double shoulder_pitch = jointShoulderToIkShoulder(shoulder_joint, elbow_joint);
    const double elbow_ik = jointElbowToIkElbow(elbow_joint);
    const double z_plane =
        params_.shoulder_pitch_offset_y +
        params_.L1 * std::cos(shoulder_pitch) +
        params_.L2 * std::cos(shoulder_pitch + elbow_ik);
    const double y_plane =
        params_.shoulder_pitch_offset_z +
        params_.L1 * std::sin(shoulder_pitch) +
        params_.L2 * std::sin(shoulder_pitch + elbow_ik);

    const double c = std::cos(psi_raw);
    const double s = std::sin(psi_raw);

    TargetPoint out;
    out.x = params_.shoulder_rot_x + s * z_plane;
    out.y = params_.shoulder_rot_y - c * z_plane;
    out.z = params_.shoulder_rot_z + y_plane;
    return out;
}

bool ArmKinematics::wristToGraspOffsetBase(const TargetVector &approach_axis, TargetVector &offset) const
{
    TargetVector forward = normalizedVector(approach_axis, {0.0, 0.0, 0.0});
    const double n = std::sqrt(forward.x * forward.x + forward.y * forward.y + forward.z * forward.z);
    if (n <= 1e-6 || !std::isfinite(n))
    {
        return false;
    }

    const TargetVector world_up{0.0, 0.0, 1.0};
    TargetVector side = normalizedVector(crossVector(world_up, forward), {1.0, 0.0, 0.0});
    if (std::fabs(side.x) < 1e-6 && std::fabs(side.y) < 1e-6 && std::fabs(side.z) < 1e-6)
    {
        side = {1.0, 0.0, 0.0};
    }

    TargetVector up = normalizedVector(crossVector(forward, side), world_up);

    offset.x = params_.wrist_to_grasp_local_x_m * side.x +
               params_.wrist_to_grasp_forward_offset_m * forward.x +
               params_.wrist_to_grasp_local_z_m * up.x;
    offset.y = params_.wrist_to_grasp_local_x_m * side.y +
               params_.wrist_to_grasp_forward_offset_m * forward.y +
               params_.wrist_to_grasp_local_z_m * up.y;
    offset.z = params_.wrist_to_grasp_local_x_m * side.z +
               params_.wrist_to_grasp_forward_offset_m * forward.z +
               params_.wrist_to_grasp_local_z_m * up.z;
    return true;
}

void ArmKinematics::snapSeedToJointLimits(std::vector<double> &seed) const
{
    constexpr double measurement_tolerance_rad = 1e-4;

    const size_t count = std::min(seed.size(), params_.joint_limits.size());
    for (size_t i = 0; i < count; ++i)
    {
        const auto &limit = params_.joint_limits[i];
        if (seed[i] < limit.min &&
            limit.min - seed[i] <= measurement_tolerance_rad)
        {
            seed[i] = limit.min;
        }
        else if (seed[i] > limit.max &&
                 seed[i] - limit.max <= measurement_tolerance_rad)
        {
            seed[i] = limit.max;
        }
    }
}

void ArmKinematics::logIkCandidates(
    const std::vector<IkCandidateDebug> &candidates,
    const std::string &selected,
    const std::string &phase_name) const
{
    if (!clock_)
    {
        return;
    }

    auto finite_cost = [](double value)
    {
        return std::isfinite(value) ? value : -1.0;
    };

    std::ostringstream ss;
    ss << "IK candidates phase=" << phase_name
       << " selected=" << selected;
    for (const auto &candidate : candidates)
    {
        ss << " | " << candidate.name
           << ": avail=" << (candidate.available ? 1 : 0)
           << " valid=" << (candidate.valid ? 1 : 0)
           << std::fixed << std::setprecision(1)
           << " rot=" << displayJointValue(0, candidate.rotation)
           << " sh=" << displayJointValue(1, candidate.shoulder)
           << " el=" << displayJointValue(2, candidate.elbow)
           << " wrist=" << displayJointValue(3, candidate.wrist)
           << std::setprecision(3)
           << " pos=" << candidate.position_error
           << std::setprecision(1)
           << " axis=" << candidate.axis_error * 180.0 / M_PI
           << " yaw=" << candidate.yaw_error * 180.0 / M_PI
           << " pitch=" << candidate.pitch_error * 180.0 / M_PI
           << " it=" << candidate.iterations
           << " reject=" << candidate.rejection
           << std::setprecision(3)
           << " cost=" << finite_cost(candidate.cost);
    }

    RCLCPP_INFO_THROTTLE(
        logger_, *clock_, 500, "%s", ss.str().c_str());
}

double ArmKinematics::ikElbowToJointElbow(double ik_elbow)
{
    return wrapPi(0.5 * M_PI - ik_elbow);
}

double ArmKinematics::jointElbowToIkElbow(double joint_elbow)
{
    return wrapPi(0.5 * M_PI - joint_elbow);
}

double ArmKinematics::ikShoulderToJointShoulder(double ik_shoulder, double joint_elbow)
{
    return wrapPi(ik_shoulder - joint_elbow);
}

double ArmKinematics::jointShoulderToIkShoulder(double joint_shoulder, double joint_elbow)
{
    return wrapPi(joint_shoulder + joint_elbow);
}

double ArmKinematics::vectorAngle(const TargetVector &a, const TargetVector &b)
{
    const double dot = clamp(a.x * b.x + a.y * b.y + a.z * b.z, -1.0, 1.0);
    return std::acos(dot);
}

double ArmKinematics::vectorYaw(const TargetVector &v)
{
    return std::atan2(v.x, -v.y);
}

double ArmKinematics::vectorPitch(const TargetVector &v)
{
    return std::atan2(v.z, std::hypot(v.x, v.y));
}

double ArmKinematics::vectorYawError(const TargetVector &actual, const TargetVector &target)
{
    return std::fabs(wrapPi(vectorYaw(actual) - vectorYaw(target)));
}

double ArmKinematics::vectorPitchError(const TargetVector &actual, const TargetVector &target)
{
    return std::fabs(vectorPitch(actual) - vectorPitch(target));
}

bool ArmKinematics::solveLinear4x4(
    std::array<std::array<double, 4>, 4> a,
    std::array<double, 4> b,
    std::array<double, 4> &x)
{
    for (size_t col = 0; col < 4; ++col)
    {
        size_t pivot = col;
        for (size_t row = col + 1; row < 4; ++row)
        {
            if (std::fabs(a[row][col]) > std::fabs(a[pivot][col]))
            {
                pivot = row;
            }
        }
        if (std::fabs(a[pivot][col]) < 1e-10)
        {
            return false;
        }
        if (pivot != col)
        {
            std::swap(a[pivot], a[col]);
            std::swap(b[pivot], b[col]);
        }

        const double divisor = a[col][col];
        for (size_t k = col; k < 4; ++k)
        {
            a[col][k] /= divisor;
        }
        b[col] /= divisor;

        for (size_t row = 0; row < 4; ++row)
        {
            if (row == col)
            {
                continue;
            }
            const double factor = a[row][col];
            for (size_t k = col; k < 4; ++k)
            {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }

    x = b;
    return true;
}

}  // namespace gimbal_mani::arm_upper
