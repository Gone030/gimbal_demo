#include "gimbal_mani/arm_upper/arm_upper_layer_node.hpp"

namespace gimbal_mani::arm_upper
{

ArmUpperLayerNode::ArmUpperLayerNode()
    : Node("arm_upper_layer_main"),
      target_projector_(this->get_logger(), this->get_clock())
{
    apply_param_values(ArmParamLoader::declareAndLoad(*this));
    goal_.assign(joint_name_.size(), 0.0);
    home_goal_ = {0.0, -0.5 * M_PI, 0.5 * M_PI, -0.5 * M_PI, 0.0, 0.0};
    kinematics_.setParams(params_.kinematics);
    kinematics_.setLogger(this->get_logger(), this->get_clock());
    target_lock_.setParams(params_.target_lock);
    grasp_planner_.setParams(params_.grasp);
    trajectory_builder_.setJointNames(joint_name_);
    tof_approach_.setParams(params_.tof);
    tof_approach_.setLogger(this->get_logger());

    pub_arm_traj_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_in/arm", 10);
    pub_auto_debug_ = this->create_publisher<gimbal_mani::msg::ArmAutoDebug>(
        "/arm/auto_debug", 10);
    pub_auto_status_ = this->create_publisher<std_msgs::msg::String>(
        "/arm/auto_status", 10);

    sub_target_ = this->create_subscription<gimbal_mani::msg::TargetBearingRange>(
        "/target", 10,
        std::bind(&ArmUpperLayerNode::on_target, this, std::placeholders::_1));

    sub_home_ = this->create_subscription<std_msgs::msg::Empty>(
        "/arm/home", 10,
        std::bind(&ArmUpperLayerNode::on_home, this, std::placeholders::_1));

    sub_auto_enable_ = this->create_subscription<std_msgs::msg::Bool>(
        "/arm/auto_enable", 10,
        std::bind(&ArmUpperLayerNode::on_auto_enable, this, std::placeholders::_1));

    sub_grasp_start_ = this->create_subscription<std_msgs::msg::Empty>(
        "/arm/grasp_start", 10,
        std::bind(&ArmUpperLayerNode::on_grasp_start, this, std::placeholders::_1));

    sub_go_up_ = this->create_subscription<std_msgs::msg::Empty>(
        "/arm/go_up", 10,
        std::bind(&ArmUpperLayerNode::on_go_up, this, std::placeholders::_1));

    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        std::bind(&ArmUpperLayerNode::on_joint_state, this, std::placeholders::_1));

    sub_tof_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/gimbal/tof_distance", rclcpp::SensorDataQoS(),
        std::bind(&ArmUpperLayerNode::on_tof_scan, this, std::placeholders::_1));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    param_cb_handle_ = this->add_on_set_parameters_callback(
        std::bind(&ArmUpperLayerNode::on_parameters_changed, this, std::placeholders::_1));

    const double publish_hz = 50.0;
    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz));
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&ArmUpperLayerNode::on_timer, this));
}

void ArmUpperLayerNode::on_timer()
{
    if (!auto_enabled_)
    {
        publish_current_goal();
        publish_auto_debug();
        publish_auto_status();
        return;
    }

    AutoControllerContext ctx;
    ctx.now_sec = this->now().seconds();
    ctx.auto_enabled = auto_enabled_;
    ctx.phase = &auto_phase_;
    ctx.target_valid = &target_valid_;
    ctx.goal = &goal_;
    ctx.command_time_from_start_sec = &command_time_from_start_sec_;
    ctx.grasp_start_requested = &grasp_start_requested_;
    ctx.go_up_requested = &go_up_requested_;
    ctx.pre_grasp_point = &pre_grasp_point_;
    ctx.frozen_pre_grasp_point = &frozen_pre_grasp_point_;
    ctx.tracked_pre_grasp_candidate = &tracked_pre_grasp_candidate_;
    ctx.lift_goal_point = &lift_goal_point_;
    ctx.has_frozen_pre_grasp_point = &has_frozen_pre_grasp_point_;
    ctx.has_tracked_pre_grasp_candidate = &has_tracked_pre_grasp_candidate_;
    ctx.has_lift_goal_point = &has_lift_goal_point_;
    ctx.locked_object_yaw = locked_object_yaw_;
    ctx.hold_start_time_sec = &hold_start_time_sec_;
    ctx.phase_start_time_sec = &phase_start_time_sec_;
    ctx.hold_reached_logged = &hold_reached_logged_;
    ctx.hold_target_exit_tol_m = hold_target_exit_tol_m_;
    ctx.hold_exit_tol_rad = hold_exit_tol_rad_;
    ctx.hold_time_sec = hold_time_sec_;
    ctx.gripper_open_pos = gripper_open_pos_;
    ctx.gripper_close_pos = gripper_close_pos_;
    ctx.gripper_action_time_sec = gripper_action_time_sec_;
    ctx.lift_command_time_sec = lift_command_time_sec_;
    ctx.lift_cartesian_tol_m = lift_cartesian_tol_m_;
    ctx.joint_reach_tol_rad = joint_reach_tol_rad_;
    ctx.lift_distance_m = lift_distance_m_;

    ctx.plan_locked_points = [this]() { return plan_locked_points(); };
    ctx.solve_goal_from_point = [this](const TargetPoint &point, double wrist_roll_ref) {
        return solve_goal_from_point(point, wrist_roll_ref);
    };
    ctx.freeze_ready = [this]() { return freeze_ready(); };
    ctx.freeze_pre_grasp_point = [this]() { freeze_pre_grasp_point(); };
    ctx.arm_goal_reached = [this](double tol) { return arm_goal_reached(tol); };
    ctx.cartesian_goal_reached = [this](double tol) { return cartesian_goal_reached(tol); };
    ctx.init_tof_joint_approach = [this]() { return init_tof_joint_approach(); };
    ctx.tof_target_inside_gripper = [this]() { return tof_target_inside_gripper(); };
    ctx.run_tof_joint_probe_step = [this]() { return run_tof_joint_probe_step(); };
    ctx.stop_tof_approach_and_hold = [this]() { stop_tof_approach_and_hold(); };
    ctx.hold_last_safe_tof_joint_goal = [this]() { return hold_last_safe_tof_joint_goal(); };
    ctx.tof_approach_blocked = [this]() { return tof_approach_.isBlocked(); };
    ctx.set_last_safe_goal = [this]() { tof_approach_.setLastSafeGoal(goal_); };
    ctx.lookup_current_arm_point = [this](TargetPoint &out) {
        geometry_msgs::msg::Point current;
        if (!lookup_current_arm_point(current))
        {
            return false;
        }
        out = {current.x, current.y, current.z};
        return true;
    };
    ctx.make_lift_goal = [this](const TargetPoint &current, double lift_distance) {
        return grasp_planner_.makeLiftGoal(current, lift_distance);
    };
    ctx.log_pre_grasp_hold_reached = [this]() {
        RCLCPP_INFO(this->get_logger(), "PRE_GRASP_HOLD reached.");
    };
    ctx.warn_grasp_start_tof_blocked = [this]() {
        RCLCPP_WARN(this->get_logger(),
                    "Ignoring grasp_start: TOF approach is blocked after repeated unsafe motion.");
    };
    ctx.warn_lift_ik_unavailable = [this]() {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Lift IK unavailable; holding the closed-gripper grasp pose.");
    };
    ctx.warn_waiting_for_lift_tf = [this]() {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Waiting for gripper TF before starting lift.");
    };
    ctx.log_lift_reached = [this](double lift_distance) {
        RCLCPP_INFO(
            this->get_logger(),
            "LIFT reached: dz=%.3f m.", lift_distance);
    };

    auto_controller_.tick(ctx);

    publish_current_goal();
    publish_auto_debug();
    publish_auto_status();
}

void ArmUpperLayerNode::on_target(const gimbal_mani::msg::TargetBearingRange &msg)
{
    if (!auto_enabled_)
    {
        return;
    }
    if (!has_joint_state_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for /joint_states before closed-loop IK update.");
        return;
    }

    if (!std::isfinite(msg.yaw) || !std::isfinite(msg.pitch) || !std::isfinite(msg.range) || msg.range <= 0.0)
    {
        return;
    }

    const auto projection = target_projector_.project(
        msg, *tf_buffer_, arm_base_frame_, min_effective_range_);
    if (!projection)
    {
        return;
    }
    target_sensor_frame_ = projection->sensor_frame;

    if (auto_phase_ == AutoPhase::TARGET_ACQUIRE)
    {
        target_lock_.pushSample(projection->sample);
        return;
    }
}

void ArmUpperLayerNode::on_home(const std_msgs::msg::Empty &)
{
    auto_enabled_ = false;
    auto_phase_ = AutoPhase::IDLE;
    reset_auto_sequence();

    goal_ = home_goal_;
    command_time_from_start_sec_ = 2.0;
    target_valid_ = true;
}

void ArmUpperLayerNode::on_auto_enable(const std_msgs::msg::Bool &msg)
{
    if (msg.data == auto_enabled_)
    {
        return;
    }

    auto_enabled_ = msg.data;

    if (auto_enabled_)
    {
        reset_auto_sequence();
        auto_phase_ = AutoPhase::TARGET_ACQUIRE;
        target_valid_ = false;
    }
    else
    {
        auto_phase_ = AutoPhase::IDLE;
        reset_auto_sequence();
        target_valid_ = false;
    }
}

void ArmUpperLayerNode::on_grasp_start(const std_msgs::msg::Empty &)
{
    if (auto_enabled_ &&
        auto_phase_ == AutoPhase::PRE_GRASP_HOLD &&
        has_frozen_pre_grasp_point_)
    {
        grasp_start_requested_ = true;
    }
}

void ArmUpperLayerNode::on_go_up(const std_msgs::msg::Empty &)
{
    if (auto_enabled_ && auto_phase_ == AutoPhase::GRASP_HOLD)
    {
        go_up_requested_ = true;
    }
}

void ArmUpperLayerNode::on_tof_scan(const sensor_msgs::msg::LaserScan &msg)
{
    if (msg.ranges.empty())
    {
        return;
    }

    const float r0 = msg.ranges[0];
    if (!std::isfinite(r0))
    {
        return;
    }
    if (std::isfinite(msg.range_min) && r0 < msg.range_min)
    {
        return;
    }
    if (std::isfinite(msg.range_max) && r0 > msg.range_max)
    {
        return;
    }

    latest_tof_range_m_ = static_cast<double>(r0);
    last_tof_time_sec_ = this->now().seconds();
    has_tof_range_ = true;
}

void ArmUpperLayerNode::on_joint_state(const sensor_msgs::msg::JointState &msg)
{
    if (msg.name.size() != msg.position.size())
    {
        return;
    }

    auto update = [&](const char *joint_name, size_t idx)
    {
        for (size_t i = 0; i < msg.name.size(); ++i)
        {
            if (msg.name[i] == joint_name)
            {
                q_meas_[idx] = msg.position[i];
                return true;
            }
        }
        return false;
    };

    const bool ok =
        update("Shoulder_Rotation", 0) &&
        update("Shoulder_Pitch", 1) &&
        update("Elbow", 2) &&
        update("Wrist_Pitch", 3) &&
        update("Wrist_Roll", 4) &&
        update("Gripper", 5);

    if (ok)
    {
        has_joint_state_ = true;
    }
}

void ArmUpperLayerNode::publish_current_goal()
{
    if (!target_valid_)
    {
        return;
    }
    if (!goal_within_joint_limits(goal_, true))
    {
        target_valid_ = false;
        return;
    }

    const auto traj = trajectory_builder_.build(
        goal_, this->now(), command_time_from_start_sec_);
    pub_arm_traj_->publish(traj);
}

bool ArmUpperLayerNode::goal_within_joint_limits(const std::vector<double> &goal, bool warn)
{
    const auto limit = kinematics_.checkJointLimits(goal);
    if (!limit.ok)
    {
        last_limit_rejection_ = limit.rejection;
        if (warn && goal.size() == joint_name_.size())
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "IK rejected: %s=%.3f outside [%.3f, %.3f] rad "
                "(display %.1f outside [%.1f, %.1f] %s), phase=%s, "
                "goal=(%.3f, %.3f, %.3f), object=(%.3f, %.3f, %.3f), "
                "pre=(%.3f, %.3f, %.3f)",
                joint_name_[limit.joint_index].c_str(), limit.value, limit.min, limit.max,
                display_joint_value(limit.joint_index, limit.value),
                display_joint_value(limit.joint_index, limit.min),
                display_joint_value(limit.joint_index, limit.max),
                joint_display_unit(limit.joint_index),
                ArmDebug::phaseName(auto_phase_),
                commanded_goal_point_.x, commanded_goal_point_.y, commanded_goal_point_.z,
                locked_object_point_.x, locked_object_point_.y, locked_object_point_.z,
                pre_grasp_point_.x, pre_grasp_point_.y, pre_grasp_point_.z);
        }
        return false;
    }

    last_limit_rejection_.clear();
    return true;
}

double ArmUpperLayerNode::display_joint_value(size_t index, double value)
{
    return ArmKinematics::displayJointValue(index, value);
}

const char *ArmUpperLayerNode::joint_display_unit(size_t index)
{
    return ArmKinematics::jointDisplayUnit(index);
}

void ArmUpperLayerNode::reset_auto_sequence()
{
    target_lock_.reset();

    locked_object_point_ = {};
    locked_approach_axis_ = {};

    pre_grasp_point_ = {};
    commanded_goal_point_ = {};
    frozen_pre_grasp_point_ = {};
    tracked_pre_grasp_candidate_ = {};
    lift_goal_point_ = {};

    has_locked_object_point_ = false;
    has_commanded_goal_point_ = false;
    has_frozen_pre_grasp_point_ = false;
    has_tracked_pre_grasp_candidate_ = false;
    has_frozen_pre_grasp_joints_ = false;
    has_frozen_approach_axis_ = false;
    has_lift_goal_point_ = false;

    grasp_start_requested_ = false;

    go_up_requested_ = false;

    locked_object_yaw_ = 0.0;

    tof_approach_.reset();

    last_valid_approach_axis_ = {};
    last_limit_rejection_.clear();
    phase_start_time_sec_ = -1.0;
    hold_start_time_sec_ = -1.0;
    hold_reached_logged_ = false;
    target_valid_ = false;
}

bool ArmUpperLayerNode::plan_locked_points()
{
    const auto locked = target_lock_.tryLock(last_valid_approach_axis_);
    if (!locked)
    {
        return false;
    }

    locked_object_point_ = locked->object_point;
    locked_approach_axis_ = locked->approach_axis;
    last_valid_approach_axis_ = locked_approach_axis_;
    locked_object_yaw_ = locked->object_yaw;
    has_locked_object_point_ = true;

    return update_pre_grasp_point();
}

bool ArmUpperLayerNode::update_pre_grasp_point()
{
    pre_grasp_point_ = compute_pre_grasp_point(locked_object_point_);
    tracked_pre_grasp_candidate_ = pre_grasp_point_;
    has_tracked_pre_grasp_candidate_ = true;
    return true;
}

TargetPoint ArmUpperLayerNode::compute_pre_grasp_point(const TargetPoint &object_point)
{
    TargetVector axis;
    if (!approach_axis(axis))
    {
        RCLCPP_WARN(this->get_logger(), "compute_pre_grasp_point: approach axis is invalid.");
        return {};
    }

    const auto pre_grasp = grasp_planner_.computePreGraspPoint(object_point, axis);
    if (!pre_grasp)
    {
        RCLCPP_WARN(this->get_logger(), "compute_pre_grasp_point: approach axis is invalid.");
        return {};
    }
    return *pre_grasp;
}

bool ArmUpperLayerNode::solve_goal_from_point(const TargetPoint &p, double wrist_roll_ref)
{
    commanded_goal_point_ = p;
    has_commanded_goal_point_ = true;

    TargetVector target_axis;
    if (!approach_axis(target_axis))
    {
        target_valid_ = false;
        return false;
    }

    gimbal_mani::arm_upper::IkRequest request;
    request.target_point = p;
    request.target_axis = target_axis;
    request.current_goal = goal_;
    request.measured_joints = q_meas_;
    request.wrist_roll_ref = wrist_roll_ref;
    request.use_tof_approach_ik_bias = use_tof_approach_ik_bias();
    request.has_frozen_pre_grasp_joints = has_frozen_pre_grasp_joints_;
    request.frozen_pre_grasp_joint_positions = frozen_pre_grasp_joint_positions_;
    request.has_frozen_pre_grasp_point = has_frozen_pre_grasp_point_;
    request.frozen_pre_grasp_point = frozen_pre_grasp_point_;
    request.has_locked_object_point = has_locked_object_point_;
    request.locked_object_point = locked_object_point_;
    request.phase_name = ArmDebug::phaseName(auto_phase_);

    const auto result = kinematics_.solveGoalFromPoint(request);
    if (!result)
    {
        target_valid_ = false;
        return false;
    }

    goal_ = result->goal;
    goal_within_joint_limits(goal_, false);
    target_valid_ = true;
    return true;
}

double ArmUpperLayerNode::arm_goal_error_sum() const
{
    return kinematics_.goalErrorSum(goal_, q_meas_);
}

bool ArmUpperLayerNode::arm_goal_reached(double tol) const
{
    return kinematics_.goalReached(goal_, q_meas_, tol);
}

double ArmUpperLayerNode::point_distance(const TargetPoint &a, const TargetPoint &b)
{
    return GraspPlanner::pointDistance(a, b);
}

bool ArmUpperLayerNode::cartesian_goal_reached(double tol) const
{
    geometry_msgs::msg::Point current;
    if (!lookup_current_arm_point(current))
    {
        return false;
    }

    const double err = cartesian_error_to_goal(current);
    return std::isfinite(err) && err <= tol;
}

bool ArmUpperLayerNode::freeze_ready() const
{
    return has_locked_object_point_ &&
           arm_goal_reached(joint_reach_tol_rad_) &&
           cartesian_goal_reached(freeze_cartesian_tol_m_);
}

bool ArmUpperLayerNode::use_tof_approach_ik_bias() const
{
    return auto_phase_ == AutoPhase::TOF_APPROACH ||
           auto_phase_ == AutoPhase::GRIPPER_CLOSE ||
           auto_phase_ == AutoPhase::GRASP_HOLD;
}

double ArmUpperLayerNode::wrap_pi(double angle)
{
    return ArmKinematics::wrapPi(angle);
}

bool ArmUpperLayerNode::approach_axis(TargetVector &axis) const
{
    if (has_frozen_approach_axis_)
    {
        axis = frozen_approach_axis_;
    }
    else if (has_locked_object_point_)
    {
        axis = locked_approach_axis_;
    }
    else
    {
        axis = last_valid_approach_axis_;
    }

    const double n = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (n <= 1e-6 || !std::isfinite(n))
    {
        return false;
    }

    axis.x /= n;
    axis.y /= n;
    axis.z /= n;
    return true;
}

void ArmUpperLayerNode::freeze_pre_grasp_point()
{
    frozen_pre_grasp_point_ = pre_grasp_point_;
    has_frozen_pre_grasp_point_ = true;
    tracked_pre_grasp_candidate_ = pre_grasp_point_;
    has_tracked_pre_grasp_candidate_ = true;
    frozen_pre_grasp_joint_positions_ = q_meas_;
    has_frozen_pre_grasp_joints_ = frozen_pre_grasp_joint_positions_.size() > 2;

    TargetVector axis;
    if (approach_axis(axis))
    {
        frozen_approach_axis_ = axis;
        has_frozen_approach_axis_ = true;
    }
}

double ArmUpperLayerNode::tof_age_sec() const
{
    if (!has_tof_range_ || last_tof_time_sec_ < 0.0)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return this->now().seconds() - last_tof_time_sec_;
}

bool ArmUpperLayerNode::tof_range_valid() const
{
    const double age = tof_age_sec();
    return has_tof_range_ &&
           std::isfinite(latest_tof_range_m_) &&
           std::isfinite(age) &&
           age <= tof_timeout_sec_;
}

bool ArmUpperLayerNode::init_tof_joint_approach()
{
    TofInitRequest request;
    request.goal = goal_;
    request.frozen_pre_grasp_joint_positions = frozen_pre_grasp_joint_positions_;
    request.has_frozen_pre_grasp_joints = has_frozen_pre_grasp_joints_;
    request.range_valid = tof_range_valid();
    request.latest_range_m = latest_tof_range_m_;
    request.current_forward_progress_m = current_forward_progress();
    request.joint_count = joint_name_.size();
    request.gripper_open_pos = gripper_open_pos_;

    return tof_approach_.init(
        request,
        [this](const std::vector<double> &candidate)
        {
            return goal_within_joint_limits(candidate, false);
        });
}

bool ArmUpperLayerNode::run_tof_joint_probe_step()
{
    TofStepRequest request;
    request.goal = goal_;
    request.range_valid = tof_range_valid();
    request.latest_range_m = latest_tof_range_m_;
    request.current_forward_progress_m = current_forward_progress();
    request.target_inside_gripper = tof_target_inside_gripper();
    request.probe_goal_reached = arm_goal_reached(tof_joint_probe_reach_tol_rad_);
    request.now_sec = this->now().seconds();
    request.joint_count = joint_name_.size();
    request.gripper_open_pos = gripper_open_pos_;

    const auto output = tof_approach_.step(
        request,
        [this](const std::vector<double> &candidate)
        {
            return goal_within_joint_limits(candidate, false);
        },
        [](size_t index, double value)
        {
            return display_joint_value(index, value);
        });

    if (output.has_goal)
    {
        goal_ = output.goal;
        target_valid_ = output.target_valid;
    }
    if (output.has_command_time)
    {
        command_time_from_start_sec_ = output.command_time_sec;
    }

    return output.result == TofApproachResult::RUNNING ||
           output.result == TofApproachResult::TARGET_INSIDE;
}

bool ArmUpperLayerNode::hold_last_safe_tof_joint_goal()
{
    std::vector<double> hold_goal;
    if (!tof_approach_.holdLastSafeGoal(joint_name_.size(), gripper_close_pos_, hold_goal))
    {
        return false;
    }

    goal_ = hold_goal;
    target_valid_ = goal_within_joint_limits(goal_, false);
    return target_valid_;
}

double ArmUpperLayerNode::current_forward_progress() const
{
    geometry_msgs::msg::Point current;
    if (!lookup_current_arm_point(current))
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return tof_approach_.currentForwardProgress(
        has_frozen_pre_grasp_point_,
        has_frozen_approach_axis_,
        {current.x, current.y, current.z},
        frozen_pre_grasp_point_,
        frozen_approach_axis_);
}

void ArmUpperLayerNode::stop_tof_approach_and_hold()
{
    tof_approach_.stopAndHold(frozen_pre_grasp_point_);
    grasp_start_requested_ = false;
    auto_phase_ = AutoPhase::PRE_GRASP_HOLD;
    phase_start_time_sec_ = -1.0;
    hold_start_time_sec_ = this->now().seconds();
    hold_reached_logged_ = true;
}

bool ArmUpperLayerNode::tof_target_inside_gripper() const
{
    return tof_approach_.targetInsideGripper(tof_range_valid(), latest_tof_range_m_);
}

bool ArmUpperLayerNode::lookup_current_arm_point(geometry_msgs::msg::Point &out) const
{
    if (!tf_buffer_)
    {
        return false;
    }

    TargetPoint point;
    if (!arm_tf_lookup_.lookupCurrentArmPoint(
            *tf_buffer_, arm_base_frame_, end_effector_frame_, point))
    {
        return false;
    }

    out.x = point.x;
    out.y = point.y;
    out.z = point.z;
    return true;
}

double ArmUpperLayerNode::cartesian_error_to_goal(const geometry_msgs::msg::Point &current) const
{
    if (!has_commanded_goal_point_)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    const double dx = commanded_goal_point_.x - current.x;
    const double dy = commanded_goal_point_.y - current.y;
    const double dz = commanded_goal_point_.z - current.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double ArmUpperLayerNode::current_target_lock_max_deviation() const
{
    if (target_lock_.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return target_lock_.currentMaxTargetDeviation(last_valid_approach_axis_);
}

double ArmUpperLayerNode::current_target_lock_max_bearing_deviation() const
{
    if (target_lock_.empty())
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return target_lock_.currentMaxBearingDeviation(last_valid_approach_axis_);
}

ArmDebugSnapshot ArmUpperLayerNode::make_debug_snapshot()
{
    DebugSnapshotInput input;
    input.stamp = this->now();
    input.now_sec = this->now().seconds();
    input.base_frame = arm_base_frame_;
    input.end_effector_frame = end_effector_frame_;

    input.auto_enabled = auto_enabled_;
    input.phase = auto_phase_;
    input.target_valid = target_valid_;
    input.has_joint_state = has_joint_state_;
    input.has_locked_object_point = has_locked_object_point_;

    input.locked_object_point = locked_object_point_;
    input.pre_grasp_point = pre_grasp_point_;
    input.commanded_goal_point = commanded_goal_point_;
    input.frozen_pre_grasp_point = frozen_pre_grasp_point_;
    input.tracked_pre_grasp_candidate = tracked_pre_grasp_candidate_;

    input.has_commanded_goal_point = has_commanded_goal_point_;
    geometry_msgs::msg::Point current_arm_point;
    input.has_current_arm_point = lookup_current_arm_point(current_arm_point);
    input.current_arm_point = {current_arm_point.x, current_arm_point.y, current_arm_point.z};
    input.freeze_cartesian_tol_m = freeze_cartesian_tol_m_;
    input.hold_frozen = has_frozen_pre_grasp_point_;
    input.has_tracked_pre_grasp_candidate = has_tracked_pre_grasp_candidate_;
    input.hold_target_exit_tol_m = hold_target_exit_tol_m_;
    input.grasp_start_requested = grasp_start_requested_;

    input.has_tof_range = tof_range_valid();
    input.tof_range_m = latest_tof_range_m_;
    input.tof_age_sec = tof_age_sec();
    input.desired_grasp_range_m = desired_grasp_range_m_;
    input.gripper_inside_range_m = gripper_inside_range_m_;
    input.gripper_inner_extra_approach_m = gripper_inner_extra_approach_m_;
    input.tof_approach_step_m = tof_approach_step_m_;
    input.tof_target_inside_gripper = tof_target_inside_gripper();
    input.tof_debug = tof_approach_.debugState();

    input.has_frozen_pre_grasp_joints = has_frozen_pre_grasp_joints_;
    input.frozen_pre_grasp_joint_positions = frozen_pre_grasp_joint_positions_;

    input.current_joint_positions = q_meas_;
    input.target_joint_positions = goal_;
    input.joint_error_sum = arm_goal_error_sum();
    input.joint_reach_tol_rad = joint_reach_tol_rad_;
    input.joint_goal_reached = arm_goal_reached(joint_reach_tol_rad_);

    input.hold_start_time_sec = hold_start_time_sec_;
    input.hold_time_sec = hold_time_sec_;

    input.target_lock_samples = static_cast<int32_t>(target_lock_.sampleCount());
    input.target_lock_min_samples = target_lock_min_samples_;
    input.target_lock_max_deviation_m = current_target_lock_max_deviation();
    input.target_lock_pos_tol_m = target_lock_pos_tol_m_;
    input.target_lock_max_bearing_deviation_rad = current_target_lock_max_bearing_deviation();

    input.last_limit_rejection = last_limit_rejection_;
    return debug_snapshot_builder_.build(input);
}

void ArmUpperLayerNode::publish_auto_debug()
{
    pub_auto_debug_->publish(ArmDebug::buildDebugMessage(make_debug_snapshot()));
}

void ArmUpperLayerNode::publish_auto_status()
{
    pub_auto_status_->publish(ArmDebug::buildStatusMessage(make_debug_snapshot()));
}

rcl_interfaces::msg::SetParametersResult ArmUpperLayerNode::on_parameters_changed(const std::vector<rclcpp::Parameter> &params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "ok";

    for (const auto &p : params)
    {
        const auto &name = p.get_name();
        if (name == "pre_grasp_distance_m")
        {
            pre_grasp_distance_m_ = p.as_double();
        }
        else if (name == "pre_grasp_lift_z_m")
        {
            pre_grasp_lift_z_m_ = p.as_double();
        }
        else if (name == "pre_grasp_lateral_offset_m")
        {
            pre_grasp_lateral_offset_m_ = p.as_double();
        }
        else if (name == "target_lock_bearing_tol_rad")
        {
            target_lock_bearing_tol_rad_ = p.as_double();
        }
        else if (name == "max_forward_approach_m")
        {
            max_forward_approach_m_ = p.as_double();
        }
        else if (name == "gripper_open_pos")
        {
            gripper_open_pos_ = p.as_double();
        }
        else if (name == "gripper_close_pos")
        {
            gripper_close_pos_ = p.as_double();
        }
        else if (name == "gripper_action_time_sec")
        {
            gripper_action_time_sec_ = p.as_double();
        }
        else if (name == "lift_distance_m")
        {
            lift_distance_m_ = p.as_double();
        }
        else if (name == "lift_command_time_sec")
        {
            lift_command_time_sec_ = p.as_double();
        }
        else if (name == "lift_cartesian_tol_m")
        {
            lift_cartesian_tol_m_ = p.as_double();
        }
        else if (name == "numerical_ik_max_iterations")
        {
            numerical_ik_max_iterations_ = p.as_int();
        }
        else if (name == "numerical_ik_position_tol_m")
        {
            numerical_ik_position_tol_m_ = p.as_double();
        }
        else if (name == "numerical_ik_axis_tol_rad")
        {
            numerical_ik_axis_tol_rad_ = p.as_double();
        }
        else if (name == "numerical_ik_accept_position_m")
        {
            numerical_ik_accept_position_m_ = p.as_double();
        }
        else if (name == "numerical_ik_accept_axis_rad")
        {
            numerical_ik_accept_axis_rad_ = p.as_double();
        }
        else if (name == "numerical_ik_max_step_rad")
        {
            numerical_ik_max_step_rad_ = p.as_double();
        }
        else if (name == "numerical_ik_damping")
        {
            numerical_ik_damping_ = p.as_double();
        }
        else if (name == "numerical_ik_axis_weight_m")
        {
            numerical_ik_axis_weight_m_ = p.as_double();
        }
        else if (name == "tof_joint_probe_step_rad")
        {
            tof_joint_probe_step_rad_ = p.as_double();
        }
        else if (name == "tof_joint_probe_shoulder_ratio")
        {
            tof_joint_probe_shoulder_ratio_ = p.as_double();
        }
        else if (name == "tof_joint_probe_wrist_ratio")
        {
            tof_joint_probe_wrist_ratio_ = p.as_double();
        }
        else if (name == "tof_joint_probe_command_time_sec")
        {
            tof_joint_probe_command_time_sec_ = p.as_double();
        }
        else if (name == "tof_joint_probe_reach_tol_rad")
        {
            tof_joint_probe_reach_tol_rad_ = p.as_double();
        }
        else if (name == "tof_success_range_delta_m")
        {
            tof_success_range_delta_m_ = p.as_double();
        }
        else if (name == "tof_fail_range_increase_m")
        {
            tof_fail_range_increase_m_ = p.as_double();
        }
        else if (name == "tof_max_failed_probe_count")
        {
            tof_max_failed_probe_count_ = p.as_int();
        }
    }

    if (has_locked_object_point_)
    {
        if (has_frozen_pre_grasp_point_)
        {
            tracked_pre_grasp_candidate_ = compute_pre_grasp_point(locked_object_point_);
            has_tracked_pre_grasp_candidate_ = true;
        }
        else
        {
            update_pre_grasp_point();
        }
    }

    sync_params_from_members();
    kinematics_.setParams(params_.kinematics);
    target_lock_.setParams(params_.target_lock);
    grasp_planner_.setParams(params_.grasp);
    trajectory_builder_.setJointNames(joint_name_);
    tof_approach_.setParams(params_.tof);
    return result;
}

ArmParamValues ArmUpperLayerNode::current_param_values() const
{
    ArmParamValues values;
    values.arm_base_frame = arm_base_frame_;
    values.end_effector_frame = end_effector_frame_;
    values.target_sensor_frame = target_sensor_frame_;
    values.min_effective_range = min_effective_range_;
    values.L1 = L1_;
    values.L2 = L2_;
    values.saturate_reach = saturate_reach_;
    values.shoulder_rot_x = shoulder_rot_x_;
    values.shoulder_rot_y = shoulder_rot_y_;
    values.shoulder_rot_z = shoulder_rot_z_;
    values.shoulder_pitch_offset_y = shoulder_pitch_offset_y_;
    values.shoulder_pitch_offset_z = shoulder_pitch_offset_z_;
    values.wrist_pitch_level_bias = wrist_pitch_level_bias_;
    values.pre_grasp_distance_m = pre_grasp_distance_m_;
    values.pre_grasp_lift_z_m = pre_grasp_lift_z_m_;
    values.pre_grasp_lateral_offset_m = pre_grasp_lateral_offset_m_;
    values.target_lock_min_samples = target_lock_min_samples_;
    values.target_lock_pos_tol_m = target_lock_pos_tol_m_;
    values.hold_time_sec = hold_time_sec_;
    values.joint_reach_tol_rad = joint_reach_tol_rad_;
    values.hold_exit_tol_rad = hold_exit_tol_rad_;
    values.target_filter_alpha = target_filter_alpha_;
    values.freeze_cartesian_tol_m = freeze_cartesian_tol_m_;
    values.hold_target_exit_tol_m = hold_target_exit_tol_m_;
    values.target_lock_bearing_tol_rad = target_lock_bearing_tol_rad_;
    values.desired_grasp_range_m = desired_grasp_range_m_;
    values.gripper_inside_range_m = gripper_inside_range_m_;
    values.gripper_inner_extra_approach_m = gripper_inner_extra_approach_m_;
    values.tof_approach_step_m = tof_approach_step_m_;
    values.min_forward_approach_m = min_forward_approach_m_;
    values.max_forward_approach_m = max_forward_approach_m_;
    values.tof_timeout_sec = tof_timeout_sec_;
    values.gripper_open_pos = gripper_open_pos_;
    values.gripper_close_pos = gripper_close_pos_;
    values.gripper_action_time_sec = gripper_action_time_sec_;
    values.lift_distance_m = lift_distance_m_;
    values.lift_command_time_sec = lift_command_time_sec_;
    values.lift_cartesian_tol_m = lift_cartesian_tol_m_;
    values.approach_reach_tol_rad = approach_reach_tol_rad_;
    values.numerical_ik_max_iterations = numerical_ik_max_iterations_;
    values.numerical_ik_position_tol_m = numerical_ik_position_tol_m_;
    values.numerical_ik_axis_tol_rad = numerical_ik_axis_tol_rad_;
    values.numerical_ik_accept_position_m = numerical_ik_accept_position_m_;
    values.numerical_ik_accept_axis_rad = numerical_ik_accept_axis_rad_;
    values.numerical_ik_fd_step_rad = numerical_ik_fd_step_rad_;
    values.numerical_ik_max_step_rad = numerical_ik_max_step_rad_;
    values.numerical_ik_damping = numerical_ik_damping_;
    values.numerical_ik_axis_weight_m = numerical_ik_axis_weight_m_;
    values.numerical_ik_position_cost_weight = numerical_ik_position_cost_weight_;
    values.numerical_ik_axis_cost_weight = numerical_ik_axis_cost_weight_;
    values.tof_range_increase_tol_m = tof_range_increase_tol_m_;
    values.tof_progress_reverse_tol_m = tof_progress_reverse_tol_m_;
    values.tof_cartesian_error_stop_m = tof_cartesian_error_stop_m_;
    values.tof_joint_probe_step_rad = tof_joint_probe_step_rad_;
    values.tof_joint_probe_shoulder_ratio = tof_joint_probe_shoulder_ratio_;
    values.tof_joint_probe_wrist_ratio = tof_joint_probe_wrist_ratio_;
    values.tof_joint_probe_command_time_sec = tof_joint_probe_command_time_sec_;
    values.tof_joint_probe_reach_tol_rad = tof_joint_probe_reach_tol_rad_;
    values.tof_success_range_delta_m = tof_success_range_delta_m_;
    values.tof_fail_range_increase_m = tof_fail_range_increase_m_;
    values.tof_max_failed_probe_count = tof_max_failed_probe_count_;
    values.tof_ik_elbow_only_penalty = tof_ik_elbow_only_penalty_;
    values.tof_ik_shoulder_bonus = tof_ik_shoulder_bonus_;
    values.tof_ik_shoulder_balance_ratio = tof_ik_shoulder_balance_ratio_;
    values.wrist_to_grasp_local_x_m = wrist_to_grasp_local_x_m_;
    values.wrist_to_grasp_forward_offset_m = wrist_to_grasp_forward_offset_m_;
    values.wrist_to_grasp_local_z_m = wrist_to_grasp_local_z_m_;
    values.joint_names = joint_name_;
    values.params = ArmParamLoader::toArmUpperParams(values);
    return values;
}

void ArmUpperLayerNode::apply_param_values(const ArmParamValues &values)
{
    arm_base_frame_ = values.arm_base_frame;
    end_effector_frame_ = values.end_effector_frame;
    target_sensor_frame_ = values.target_sensor_frame;
    min_effective_range_ = values.min_effective_range;
    L1_ = values.L1;
    L2_ = values.L2;
    saturate_reach_ = values.saturate_reach;
    shoulder_rot_x_ = values.shoulder_rot_x;
    shoulder_rot_y_ = values.shoulder_rot_y;
    shoulder_rot_z_ = values.shoulder_rot_z;
    shoulder_pitch_offset_y_ = values.shoulder_pitch_offset_y;
    shoulder_pitch_offset_z_ = values.shoulder_pitch_offset_z;
    wrist_pitch_level_bias_ = values.wrist_pitch_level_bias;
    pre_grasp_distance_m_ = values.pre_grasp_distance_m;
    pre_grasp_lift_z_m_ = values.pre_grasp_lift_z_m;
    pre_grasp_lateral_offset_m_ = values.pre_grasp_lateral_offset_m;
    target_lock_min_samples_ = values.target_lock_min_samples;
    target_lock_pos_tol_m_ = values.target_lock_pos_tol_m;
    hold_time_sec_ = values.hold_time_sec;
    joint_reach_tol_rad_ = values.joint_reach_tol_rad;
    hold_exit_tol_rad_ = values.hold_exit_tol_rad;
    target_filter_alpha_ = values.target_filter_alpha;
    freeze_cartesian_tol_m_ = values.freeze_cartesian_tol_m;
    hold_target_exit_tol_m_ = values.hold_target_exit_tol_m;
    target_lock_bearing_tol_rad_ = values.target_lock_bearing_tol_rad;
    desired_grasp_range_m_ = values.desired_grasp_range_m;
    gripper_inside_range_m_ = values.gripper_inside_range_m;
    gripper_inner_extra_approach_m_ = values.gripper_inner_extra_approach_m;
    tof_approach_step_m_ = values.tof_approach_step_m;
    min_forward_approach_m_ = values.min_forward_approach_m;
    max_forward_approach_m_ = values.max_forward_approach_m;
    tof_timeout_sec_ = values.tof_timeout_sec;
    gripper_open_pos_ = values.gripper_open_pos;
    gripper_close_pos_ = values.gripper_close_pos;
    gripper_action_time_sec_ = values.gripper_action_time_sec;
    lift_distance_m_ = values.lift_distance_m;
    lift_command_time_sec_ = values.lift_command_time_sec;
    lift_cartesian_tol_m_ = values.lift_cartesian_tol_m;
    approach_reach_tol_rad_ = values.approach_reach_tol_rad;
    numerical_ik_max_iterations_ = values.numerical_ik_max_iterations;
    numerical_ik_position_tol_m_ = values.numerical_ik_position_tol_m;
    numerical_ik_axis_tol_rad_ = values.numerical_ik_axis_tol_rad;
    numerical_ik_accept_position_m_ = values.numerical_ik_accept_position_m;
    numerical_ik_accept_axis_rad_ = values.numerical_ik_accept_axis_rad;
    numerical_ik_fd_step_rad_ = values.numerical_ik_fd_step_rad;
    numerical_ik_max_step_rad_ = values.numerical_ik_max_step_rad;
    numerical_ik_damping_ = values.numerical_ik_damping;
    numerical_ik_axis_weight_m_ = values.numerical_ik_axis_weight_m;
    numerical_ik_position_cost_weight_ = values.numerical_ik_position_cost_weight;
    numerical_ik_axis_cost_weight_ = values.numerical_ik_axis_cost_weight;
    tof_range_increase_tol_m_ = values.tof_range_increase_tol_m;
    tof_progress_reverse_tol_m_ = values.tof_progress_reverse_tol_m;
    tof_cartesian_error_stop_m_ = values.tof_cartesian_error_stop_m;
    tof_joint_probe_step_rad_ = values.tof_joint_probe_step_rad;
    tof_joint_probe_shoulder_ratio_ = values.tof_joint_probe_shoulder_ratio;
    tof_joint_probe_wrist_ratio_ = values.tof_joint_probe_wrist_ratio;
    tof_joint_probe_command_time_sec_ = values.tof_joint_probe_command_time_sec;
    tof_joint_probe_reach_tol_rad_ = values.tof_joint_probe_reach_tol_rad;
    tof_success_range_delta_m_ = values.tof_success_range_delta_m;
    tof_fail_range_increase_m_ = values.tof_fail_range_increase_m;
    tof_max_failed_probe_count_ = values.tof_max_failed_probe_count;
    tof_ik_elbow_only_penalty_ = values.tof_ik_elbow_only_penalty;
    tof_ik_shoulder_bonus_ = values.tof_ik_shoulder_bonus;
    tof_ik_shoulder_balance_ratio_ = values.tof_ik_shoulder_balance_ratio;
    wrist_to_grasp_local_x_m_ = values.wrist_to_grasp_local_x_m;
    wrist_to_grasp_forward_offset_m_ = values.wrist_to_grasp_forward_offset_m;
    wrist_to_grasp_local_z_m_ = values.wrist_to_grasp_local_z_m;
    joint_name_ = values.joint_names;
    params_ = values.params;
}

void ArmUpperLayerNode::sync_params_from_members()
{
    params_ = ArmParamLoader::toArmUpperParams(current_param_values());
}

}  // namespace gimbal_mani::arm_upper
