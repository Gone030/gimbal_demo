#include <cmath>
#include <string>
#include <vector>
#include <deque>
#include <chrono>
#include <algorithm>
#include <limits>
#include <sstream>
#include <iomanip>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "gimbal_mani/ik_2link_2d.hpp"
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

class Arm_UpperLayerMain : public rclcpp::Node
{
public:
    Arm_UpperLayerMain() : Node("arm_upper_layer_main")
    {
        pre_grasp_distance_m_ =
            this->declare_parameter<double>("pre_grasp_distance_m", pre_grasp_distance_m_);
        pre_grasp_lift_z_m_ =
            this->declare_parameter<double>("pre_grasp_lift_z_m", pre_grasp_lift_z_m_);
        pre_grasp_lateral_offset_m_ =
            this->declare_parameter<double>("pre_grasp_lateral_offset_m", pre_grasp_lateral_offset_m_);
        target_lock_bearing_tol_rad_ =
            this->declare_parameter<double>("target_lock_bearing_tol_rad", target_lock_bearing_tol_rad_);
        max_forward_approach_m_ =
            this->declare_parameter<double>("max_forward_approach_m", max_forward_approach_m_);
        gripper_open_pos_ =
            this->declare_parameter<double>("gripper_open_pos", gripper_open_pos_);
        gripper_close_pos_ =
            this->declare_parameter<double>("gripper_close_pos", gripper_close_pos_);
        gripper_action_time_sec_ =
            this->declare_parameter<double>("gripper_action_time_sec", gripper_action_time_sec_);
        tof_joint_probe_step_rad_ =
            this->declare_parameter<double>("tof_joint_probe_step_rad", tof_joint_probe_step_rad_);
        tof_joint_probe_shoulder_ratio_ =
            this->declare_parameter<double>("tof_joint_probe_shoulder_ratio", tof_joint_probe_shoulder_ratio_);
        tof_joint_probe_wrist_ratio_ =
            this->declare_parameter<double>("tof_joint_probe_wrist_ratio", tof_joint_probe_wrist_ratio_);
        tof_joint_probe_command_time_sec_ =
            this->declare_parameter<double>("tof_joint_probe_command_time_sec", tof_joint_probe_command_time_sec_);
        tof_joint_probe_reach_tol_rad_ =
            this->declare_parameter<double>("tof_joint_probe_reach_tol_rad", tof_joint_probe_reach_tol_rad_);
        tof_success_range_delta_m_ =
            this->declare_parameter<double>("tof_success_range_delta_m", tof_success_range_delta_m_);
        tof_fail_range_increase_m_ =
            this->declare_parameter<double>("tof_fail_range_increase_m", tof_fail_range_increase_m_);
        tof_max_failed_probe_count_ =
            this->declare_parameter<int>("tof_max_failed_probe_count", tof_max_failed_probe_count_);

        joint_name_ = {
            "Shoulder_Rotation",
            "Shoulder_Pitch",
            "Elbow",
            "Wrist_Pitch",
            "Wrist_Roll",
            "Gripper"};
        goal_.assign(joint_name_.size(), 0.0);
        home_goal_ = {0.0, -0.5 * M_PI, 0.5 * M_PI, -0.5 * M_PI, 0.0, 0.0};

        pub_arm_traj_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_in/arm", 10);
        pub_auto_debug_ = this->create_publisher<gimbal_mani::msg::ArmAutoDebug>(
            "/arm/auto_debug", 10);
        pub_auto_status_ = this->create_publisher<std_msgs::msg::String>(
            "/arm/auto_status", 10);

        sub_target_ = this->create_subscription<gimbal_mani::msg::TargetBearingRange>(
            "/target", 10,
            std::bind(&Arm_UpperLayerMain::on_target, this, std::placeholders::_1));

        sub_home_ = this->create_subscription<std_msgs::msg::Empty>(
            "/arm/home", 10,
            std::bind(&Arm_UpperLayerMain::on_home, this, std::placeholders::_1));

        sub_auto_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/arm/auto_enable", 10,
            std::bind(&Arm_UpperLayerMain::on_auto_enable, this, std::placeholders::_1));

        sub_grasp_start_ = this->create_subscription<std_msgs::msg::Empty>(
            "/arm/grasp_start", 10,
            std::bind(&Arm_UpperLayerMain::on_grasp_start, this, std::placeholders::_1));

        sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&Arm_UpperLayerMain::on_joint_state, this, std::placeholders::_1));

        sub_tof_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/gimbal/tof_distance", rclcpp::SensorDataQoS(),
            std::bind(&Arm_UpperLayerMain::on_tof_scan, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&Arm_UpperLayerMain::on_parameters_changed, this, std::placeholders::_1));

        const double publish_hz = 50.0;
        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&Arm_UpperLayerMain::on_timer, this));
    }

private:
    enum class AutoPhase
    {
        IDLE,
        TARGET_ACQUIRE,
        TRACK_PRE_GRASP,
        PRE_GRASP_HOLD,
        GRIPPER_OPEN,
        TOF_APPROACH,
        GRIPPER_CLOSE,
        GRASP_HOLD,
    };

    struct TargetSample
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double approach_x{1.0};
        double approach_y{0.0};
        double approach_z{0.0};
        double bearing_yaw{0.0};
        double bearing_pitch{0.0};
        double object_yaw{0.0};
    };

    struct TargetPoint
    {
        double x{0.0};
        double y{0.0};
        double z{0.0};
    };

    struct TargetVector
    {
        double x{1.0};
        double y{0.0};
        double z{0.0};
    };

    void on_timer()
    {
        if (!auto_enabled_)
        {
            publish_current_goal();
            publish_auto_debug();
            publish_auto_status();
            return;
        }

        switch (auto_phase_)
        {
        case AutoPhase::IDLE:
            break;

        case AutoPhase::TARGET_ACQUIRE:
        {
            if (plan_locked_points())
            {
                if (!solve_goal_from_point(pre_grasp_point_, locked_object_yaw_))
                {
                    target_valid_ = false;
                    break;
                }

                command_time_from_start_sec_ = 1.0;
                auto_phase_ = AutoPhase::TRACK_PRE_GRASP;

                // RCLCPP_INFO(this->get_logger(),
                //             "TARGET_ACQUIRE done: obj=(%.3f, %.3f, %.3f) pre=(%.3f, %.3f, %.3f)",
                //             locked_object_point_.x, locked_object_point_.y, locked_object_point_.z,
                //             pre_grasp_point_.x, pre_grasp_point_.y, pre_grasp_point_.z);
            }
            break;
        }

        case AutoPhase::TRACK_PRE_GRASP:
        {
            if (!solve_goal_from_point(pre_grasp_point_, locked_object_yaw_))
            {
                target_valid_ = false;
                break;
            }

            command_time_from_start_sec_ = 1.0;
            if (freeze_ready())
            {
                freeze_pre_grasp_point();
                auto_phase_ = AutoPhase::PRE_GRASP_HOLD;
                hold_start_time_sec_ = this->now().seconds();
                hold_reached_logged_ = false;
            }
            break;
        }

        case AutoPhase::PRE_GRASP_HOLD:
        {
            if (!has_frozen_pre_grasp_point_)
            {
                freeze_pre_grasp_point();
            }

            if (has_tracked_pre_grasp_candidate_)
            {
                const double drift = point_distance(tracked_pre_grasp_candidate_, frozen_pre_grasp_point_);
                if (drift > hold_target_exit_tol_m_)
                {
                    pre_grasp_point_ = tracked_pre_grasp_candidate_;
                    has_frozen_pre_grasp_point_ = false;
                    auto_phase_ = AutoPhase::TRACK_PRE_GRASP;
                    hold_start_time_sec_ = -1.0;
                    hold_reached_logged_ = false;
                    break;
                }
            }

            if (!solve_goal_from_point(frozen_pre_grasp_point_, locked_object_yaw_))
            {
                target_valid_ = false;
                break;
            }

            command_time_from_start_sec_ = 1.0;
            if (!arm_goal_reached(hold_exit_tol_rad_))
            {
                auto_phase_ = AutoPhase::TRACK_PRE_GRASP;
                hold_start_time_sec_ = -1.0;
                hold_reached_logged_ = false;
                break;
            }

            const double held = this->now().seconds() - hold_start_time_sec_;
            if (held >= hold_time_sec_ && !hold_reached_logged_)
            {
                RCLCPP_INFO(this->get_logger(), "PRE_GRASP_HOLD reached.");
                hold_reached_logged_ = true;
            }
            if (hold_reached_logged_ && grasp_start_requested_)
            {
                grasp_start_requested_ = false;
                if (tof_approach_blocked_)
                {
                    RCLCPP_WARN(this->get_logger(),
                                "Ignoring grasp_start: TOF approach is blocked after repeated unsafe motion.");
                    break;
                }
                auto_phase_ = AutoPhase::GRIPPER_OPEN;
                phase_start_time_sec_ = this->now().seconds();
            }
            break;
        }

        case AutoPhase::GRIPPER_OPEN:
        {
            if (!solve_goal_from_point(frozen_pre_grasp_point_, locked_object_yaw_))
            {
                target_valid_ = false;
                break;
            }

            goal_[5] = gripper_open_pos_;
            command_time_from_start_sec_ = gripper_action_time_sec_;
            if (elapsed_in_phase() >= gripper_action_time_sec_ &&
                init_tof_joint_approach())
            {
                auto_phase_ = AutoPhase::TOF_APPROACH;
                phase_start_time_sec_ = this->now().seconds();
            }
            break;
        }

        case AutoPhase::TOF_APPROACH:
        {
            if (tof_target_inside_gripper())
            {
                auto_phase_ = AutoPhase::GRIPPER_CLOSE;
                phase_start_time_sec_ = this->now().seconds();
                break;
            }

            if (!run_tof_joint_probe_step())
            {
                stop_tof_approach_and_hold();
            }
            break;
        }

        case AutoPhase::GRIPPER_CLOSE:
        {
            if (!hold_last_safe_tof_joint_goal())
            {
                target_valid_ = false;
                break;
            }

            goal_[5] = gripper_close_pos_;
            command_time_from_start_sec_ = gripper_action_time_sec_;
            if (elapsed_in_phase() >= gripper_action_time_sec_)
            {
                auto_phase_ = AutoPhase::GRASP_HOLD;
            }
            break;
        }

        case AutoPhase::GRASP_HOLD:
        {
            if (!hold_last_safe_tof_joint_goal())
            {
                target_valid_ = false;
                break;
            }

            goal_[5] = gripper_close_pos_;
            command_time_from_start_sec_ = 1.0;
            break;
        }
        }

        publish_current_goal();
        publish_auto_debug();
        publish_auto_status();
    }

    void on_target(const gimbal_mani::msg::TargetBearingRange &msg)
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

        double target_x = 0.0;
        double target_y = 0.0;
        double target_z = 0.0;
        double ray_ux = 0.0;
        double ray_uy = 0.0;
        double ray_uz = 0.0;
        if (!target_to_arm_base_point(msg, target_x, target_y, target_z, ray_ux, ray_uy, ray_uz))
        {
            return;
        }

        if (auto_phase_ == AutoPhase::TARGET_ACQUIRE)
        {
            target_lock_buffer_.push_back(
                {target_x, target_y, target_z,
                 ray_ux, ray_uy, ray_uz,
                 msg.yaw, msg.pitch, msg.object_yaw});
            while (static_cast<int>(target_lock_buffer_.size()) > target_lock_min_samples_)
            {
                target_lock_buffer_.pop_front();
            }
            return;
        }
    }

    void on_home(const std_msgs::msg::Empty &)
    {
        auto_enabled_ = false;
        auto_phase_ = AutoPhase::IDLE;
        reset_auto_sequence();

        goal_ = home_goal_;
        command_time_from_start_sec_ = 2.0;
        target_valid_ = true;
    }

    void on_auto_enable(const std_msgs::msg::Bool &msg)
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

    void on_grasp_start(const std_msgs::msg::Empty &)
    {
        if (auto_enabled_ &&
            auto_phase_ == AutoPhase::PRE_GRASP_HOLD &&
            has_frozen_pre_grasp_point_)
        {
            grasp_start_requested_ = true;
        }
    }

    void on_tof_scan(const sensor_msgs::msg::LaserScan &msg)
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

    void on_joint_state(const sensor_msgs::msg::JointState &msg)
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

    bool target_to_arm_base_point(const gimbal_mani::msg::TargetBearingRange &msg,
                                  double &x_out, double &y_out, double &z_out,
                                  double &ux_out, double &uy_out, double &uz_out)
    {
        if (msg.header.frame_id.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Dropping target: header.frame_id is empty.");
            return false;
        }
        target_sensor_frame_ = msg.header.frame_id;

        const double range_use = std::max(min_effective_range_, msg.range);
        const geometry_msgs::msg::PointStamped p_sensor = [&]()
        {
            geometry_msgs::msg::PointStamped p;
            p.header = msg.header;
            p.point.x = range_use;
            p.point.y = 0.0;
            p.point.z = 0.0;
            return p;
        }();

        geometry_msgs::msg::PointStamped p_base;
        try
        {
            const rclcpp::Time tf_time(msg.header.stamp);
            const auto tf = tf_buffer_->lookupTransform(
                arm_base_frame_, msg.header.frame_id, tf_time, rclcpp::Duration::from_seconds(0.03));
            tf2::doTransform(p_sensor, p_base, tf);

            tf2::Quaternion q;
            tf2::fromMsg(tf.transform.rotation, q);
            tf2::Matrix3x3 R(q);
            tf2::Vector3 u_base = R * tf2::Vector3(1.0, 0.0, 0.0);
            const double n = u_base.length();
            if (n <= 1e-9 || !std::isfinite(n))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Invalid ray direction norm after TF rotation.");
                return false;
            }
            ux_out = u_base.x() / n;
            uy_out = u_base.y() / n;
            uz_out = u_base.z() / n;

            // RCLCPP_INFO_THROTTLE(
            //     this->get_logger(), *this->get_clock(), 500,
            //     "target_tf: frame=%s->%s yaw=%.3f pitch=%.3f range=%.3f range_use=%.3f | "
            //     "p_sensor=(%.3f,%.3f,%.3f) p_base=(%.3f,%.3f,%.3f) u_base=(%.3f,%.3f,%.3f)",
            //     msg.header.frame_id.c_str(), arm_base_frame_.c_str(),
            //     msg.yaw, msg.pitch, msg.range, range_use,
            //     p_sensor.point.x, p_sensor.point.y, p_sensor.point.z,
            //     p_base.point.x, p_base.point.y, p_base.point.z,
            //     ux_out, uy_out, uz_out);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "TF failed (%s -> %s): %s",
                                 msg.header.frame_id.c_str(), arm_base_frame_.c_str(), ex.what());
            return false;
        }

        x_out = p_base.point.x;
        y_out = p_base.point.y;
        z_out = p_base.point.z;
        return true;
    }

    void publish_current_goal()
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

        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.joint_names = joint_name_;

        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = goal_;
        p.time_from_start = rclcpp::Duration::from_seconds(command_time_from_start_sec_);

        traj.points.push_back(p);
        pub_arm_traj_->publish(traj);
    }

    bool goal_within_joint_limits(const std::vector<double> &goal, bool warn)
    {
        if (goal.size() != joint_name_.size())
        {
            last_limit_rejection_ = "goal-size";
            return false;
        }

        static const std::vector<double> min_limits = {
            -0.5 * M_PI,
            -0.5 * M_PI,
            -0.5 * M_PI,
            -M_PI,
            0.0,
            0.0};
        static const std::vector<double> max_limits = {
            0.5 * M_PI,
            0.5 * M_PI,
            0.5 * M_PI,
            0.0,
            0.0,
            1.5};

        constexpr double eps = 1e-6;
        for (size_t i = 0; i < goal.size(); ++i)
        {
            if (goal[i] < min_limits[i] - eps || goal[i] > max_limits[i] + eps)
            {
                std::ostringstream ss;
                ss << joint_name_[i] << ":"
                   << std::fixed << std::setprecision(1) << display_joint_value(i, goal[i]);
                last_limit_rejection_ = ss.str();

                if (warn)
                {
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 1000,
                        "IK rejected: %s=%.3f outside [%.3f, %.3f] rad "
                        "(display %.1f outside [%.1f, %.1f] %s), phase=%s, "
                        "goal=(%.3f, %.3f, %.3f), object=(%.3f, %.3f, %.3f), "
                        "pre=(%.3f, %.3f, %.3f)",
                        joint_name_[i].c_str(), goal[i], min_limits[i], max_limits[i],
                        display_joint_value(i, goal[i]),
                        display_joint_value(i, min_limits[i]),
                        display_joint_value(i, max_limits[i]),
                        joint_display_unit(i),
                        phase_name(),
                        commanded_goal_point_.x, commanded_goal_point_.y, commanded_goal_point_.z,
                        locked_object_point_.x, locked_object_point_.y, locked_object_point_.z,
                        pre_grasp_point_.x, pre_grasp_point_.y, pre_grasp_point_.z);
                }
                return false;
            }
        }

        last_limit_rejection_.clear();
        return true;
    }

    static double display_joint_value(size_t index, double value)
    {
        if (index == 5)
        {
            return value;
        }
        return value * 180.0 / M_PI;
    }

    static const char *joint_display_unit(size_t index)
    {
        return index == 5 ? "raw" : "deg";
    }

    template <typename Candidate>
    void log_ik_candidates(const Candidate &up, const Candidate &dn,
                           const std::string &selected)
    {
        auto finite_cost = [](double value)
        {
            return std::isfinite(value) ? value : -1.0;
        };

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "IK candidates phase=%s selected=%s | "
            "up: avail=%d valid=%d sh=%.1f el=%.1f wrist=%.1f cost=%.3f | "
            "dn: avail=%d valid=%d sh=%.1f el=%.1f wrist=%.1f cost=%.3f",
            phase_name(), selected.c_str(),
            up.available ? 1 : 0,
            up.valid ? 1 : 0,
            display_joint_value(1, up.shoulder),
            display_joint_value(2, up.elbow),
            display_joint_value(3, up.wrist),
            finite_cost(up.cost),
            dn.available ? 1 : 0,
            dn.valid ? 1 : 0,
            display_joint_value(1, dn.shoulder),
            display_joint_value(2, dn.elbow),
            display_joint_value(3, dn.wrist),
            finite_cost(dn.cost));
    }

    void reset_auto_sequence()
    {
        target_lock_buffer_.clear();
        locked_object_point_ = {};
        locked_approach_axis_ = {};
        pre_grasp_point_ = {};
        commanded_goal_point_ = {};
        frozen_pre_grasp_point_ = {};
        tracked_pre_grasp_candidate_ = {};
        tof_approach_point_ = {};
        has_locked_object_point_ = false;
        has_commanded_goal_point_ = false;
        has_frozen_pre_grasp_point_ = false;
        has_tracked_pre_grasp_candidate_ = false;
        has_frozen_pre_grasp_joints_ = false;
        has_frozen_approach_axis_ = false;
        has_tof_approach_point_ = false;
        grasp_start_requested_ = false;
        locked_object_yaw_ = 0.0;
        tof_forward_distance_m_ = 0.0;
        last_tof_approach_range_m_ = std::numeric_limits<double>::quiet_NaN();
        last_tof_forward_progress_m_ = std::numeric_limits<double>::quiet_NaN();
        tof_approach_faulted_ = false;
        tof_approach_blocked_ = false;
        tof_approach_reverse_retry_used_ = false;
        tof_approach_axis_sign_ = 1.0;
        last_safe_tof_joint_positions_.clear();
        tof_pending_probe_ = false;
        tof_active_probe_index_ = -1;
        tof_preferred_probe_index_ = 0;
        tof_probe_failed_mask_ = 0;
        tof_probe_failed_count_ = 0;
        tof_probe_start_range_m_ = std::numeric_limits<double>::quiet_NaN();
        tof_probe_start_progress_m_ = std::numeric_limits<double>::quiet_NaN();
        tof_probe_start_time_sec_ = -1.0;
        last_valid_approach_axis_ = {};
        last_limit_rejection_.clear();
        phase_start_time_sec_ = -1.0;
        hold_start_time_sec_ = -1.0;
        hold_reached_logged_ = false;
        target_valid_ = false;
    }

    TargetSample mean_locked_target() const
    {
        TargetSample m;
        if (target_lock_buffer_.empty())
        {
            return m;
        }

        double yaw_c = 0.0;
        double yaw_s = 0.0;
        double approach_x = 0.0;
        double approach_y = 0.0;
        double approach_z = 0.0;
        for (const auto &p : target_lock_buffer_)
        {
            m.x += p.x;
            m.y += p.y;
            m.z += p.z;
            approach_x += p.approach_x;
            approach_y += p.approach_y;
            approach_z += p.approach_z;
            yaw_c += std::cos(p.object_yaw);
            yaw_s += std::sin(p.object_yaw);
        }

        const double n = static_cast<double>(target_lock_buffer_.size());
        m.x /= n;
        m.y /= n;
        m.z /= n;
        const auto approach = normalized_vector({approach_x / n, approach_y / n, approach_z / n},
                                                last_valid_approach_axis_);
        m.approach_x = approach.x;
        m.approach_y = approach.y;
        m.approach_z = approach.z;
        m.bearing_yaw = mean_bearing_yaw();
        m.bearing_pitch = mean_bearing_pitch();
        m.object_yaw = std::atan2(yaw_s, yaw_c);
        return m;
    }

    double max_locked_target_deviation(const TargetSample &mean) const
    {
        double max_d = 0.0;
        for (const auto &p : target_lock_buffer_)
        {
            const double dx = p.x - mean.x;
            const double dy = p.y - mean.y;
            const double dz = p.z - mean.z;
            const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
            max_d = std::max(max_d, d);
        }
        return max_d;
    }

    double max_locked_bearing_deviation(const TargetSample &mean) const
    {
        double max_d = 0.0;
        for (const auto &p : target_lock_buffer_)
        {
            const double dyaw = wrap_pi(p.bearing_yaw - mean.bearing_yaw);
            const double dpitch = p.bearing_pitch - mean.bearing_pitch;
            max_d = std::max(max_d, std::hypot(dyaw, dpitch));
        }
        return max_d;
    }

    bool plan_locked_points()
    {
        if (static_cast<int>(target_lock_buffer_.size()) < target_lock_min_samples_)
        {
            return false;
        }

        const auto mean = mean_locked_target();
        const double max_dev = max_locked_target_deviation(mean);
        if (max_dev > target_lock_pos_tol_m_)
        {
            return false;
        }
        const double max_bearing_dev = max_locked_bearing_deviation(mean);
        if (max_bearing_dev > target_lock_bearing_tol_rad_)
        {
            return false;
        }

        locked_object_point_.x = mean.x;
        locked_object_point_.y = mean.y;
        locked_object_point_.z = mean.z;
        locked_approach_axis_ = normalized_vector(
            {mean.approach_x, mean.approach_y, mean.approach_z},
            last_valid_approach_axis_);
        last_valid_approach_axis_ = locked_approach_axis_;
        locked_object_yaw_ = mean.object_yaw;
        has_locked_object_point_ = true;

        return update_pre_grasp_point();
    }

    void update_tracked_target(const TargetSample &sample, bool candidate_only)
    {
        const double alpha = clamp(target_filter_alpha_, 0.0, 1.0);
        if (!has_locked_object_point_)
        {
            locked_object_point_.x = sample.x;
            locked_object_point_.y = sample.y;
            locked_object_point_.z = sample.z;
            locked_approach_axis_ = normalized_vector(
                {sample.approach_x, sample.approach_y, sample.approach_z},
                last_valid_approach_axis_);
            last_valid_approach_axis_ = locked_approach_axis_;
            locked_object_yaw_ = sample.object_yaw;
            has_locked_object_point_ = true;
        }
        else
        {
            locked_object_point_.x += alpha * (sample.x - locked_object_point_.x);
            locked_object_point_.y += alpha * (sample.y - locked_object_point_.y);
            locked_object_point_.z += alpha * (sample.z - locked_object_point_.z);
            locked_approach_axis_.x += alpha * (sample.approach_x - locked_approach_axis_.x);
            locked_approach_axis_.y += alpha * (sample.approach_y - locked_approach_axis_.y);
            locked_approach_axis_.z += alpha * (sample.approach_z - locked_approach_axis_.z);
            locked_approach_axis_ = normalized_vector(locked_approach_axis_, last_valid_approach_axis_);
            last_valid_approach_axis_ = locked_approach_axis_;
            locked_object_yaw_ += alpha * wrap_pi(sample.object_yaw - locked_object_yaw_);
            locked_object_yaw_ = wrap_pi(locked_object_yaw_);
        }

        if (candidate_only)
        {
            tracked_pre_grasp_candidate_ = compute_pre_grasp_point(locked_object_point_);
            has_tracked_pre_grasp_candidate_ = true;
        }
        else
        {
            update_pre_grasp_point();
        }
    }

    bool update_pre_grasp_point()
    {
        pre_grasp_point_ = compute_pre_grasp_point(locked_object_point_);
        tracked_pre_grasp_candidate_ = pre_grasp_point_;
        has_tracked_pre_grasp_candidate_ = true;
        return true;
    }

    TargetPoint compute_pre_grasp_point(const TargetPoint &object_point)
    {
        TargetPoint out;
        TargetVector axis;
        if (!approach_axis(axis))
        {
            RCLCPP_WARN(this->get_logger(), "compute_pre_grasp_point: approach axis is invalid.");
            return out;
        }

        TargetVector sensor_to_grasp_perp{0.0, 0.0, 0.0};

        out.x = object_point.x + sensor_to_grasp_perp.x - pre_grasp_distance_m_ * axis.x;
        out.y = object_point.y + sensor_to_grasp_perp.y - pre_grasp_distance_m_ * axis.y;
        out.z = object_point.z + sensor_to_grasp_perp.z - pre_grasp_distance_m_ * axis.z + pre_grasp_lift_z_m_;

        return out;
    }

    bool solve_goal_from_point(const TargetPoint &p, double wrist_roll_ref)
    {
        const TargetPoint wrist_target = grasp_to_wrist_target(p);
        const double x = wrist_target.x;
        const double y = wrist_target.y;
        const double z = wrist_target.z;

        const double x_sr = x - shoulder_rot_x_;
        const double y_sr = y - shoulder_rot_y_;
        const double z_sr = z - shoulder_rot_z_;

        const double psi_raw = std::atan2(x_sr, -y_sr);
        const double psi = nearest_equivalent_angle(psi_raw, q_meas_[0]);

        const double c = std::cos(psi_raw);
        const double s = std::sin(psi_raw);

        const double y_plane = z_sr;
        const double z_plane = s * x_sr - c * y_sr;

        double rho = z_plane - shoulder_pitch_offset_y_;
        double z2 = y_plane - shoulder_pitch_offset_z_;

        // RCLCPP_INFO_THROTTLE(
        //     this->get_logger(), *this->get_clock(), 500,
        //     "solve_goal: p=(%.3f,%.3f,%.3f) rho=%.3f z2=%.3f",
        //     x, y, z, rho, z2);

        if (saturate_reach_)
        {
            const double rr = std::sqrt((rho * rho) + (z2 * z2));
            if (rr >= 1e-9)
            {
                const double r_max = (L1_ + L2_);
                const double r_min = std::fabs(L1_ - L2_);
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
        const bool ok_up = ik_2link_2d(rho, z2, L1_, L2_, +1, th1_up, th2_up);
        const bool ok_dn = ik_2link_2d(rho, z2, L1_, L2_, -1, th1_dn, th2_dn);

        if (!ok_up && !ok_dn)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "IK unreachable: rho=%.4f z=%.4f (L1=%.3f L2=%.3f)", rho, z2, L1_, L2_);
            return false;
        }

        const double prev_shoulder_pitch = q_meas_[1];
        const double prev_elbow = q_meas_[2];

        const double elbow_up = nearest_equivalent_angle(ik_elbow_to_joint_elbow(th2_up), prev_elbow);
        const double shoulder_pitch_up =
            nearest_equivalent_angle(ik_shoulder_to_joint_shoulder(th1_up, elbow_up), prev_shoulder_pitch);
        const double elbow_dn = nearest_equivalent_angle(ik_elbow_to_joint_elbow(th2_dn), prev_elbow);
        const double shoulder_pitch_dn =
            nearest_equivalent_angle(ik_shoulder_to_joint_shoulder(th1_dn, elbow_dn), prev_shoulder_pitch);

        auto cost = [&](double a1, double a2)
        {
            const double w_sh = 0.7;
            const double w_el = 1.1;
            double value = w_sh * std::fabs(a1 - prev_shoulder_pitch) +
                           w_el * std::fabs(a2 - prev_elbow);

            if (use_tof_approach_ik_bias() && has_frozen_pre_grasp_joints_ &&
                frozen_pre_grasp_joint_positions_.size() > 2)
            {
                const double shoulder_delta =
                    std::fabs(wrap_pi(a1 - frozen_pre_grasp_joint_positions_[1]));
                const double elbow_delta =
                    std::fabs(wrap_pi(a2 - frozen_pre_grasp_joint_positions_[2]));
                const double elbow_only =
                    std::max(0.0, elbow_delta - tof_ik_shoulder_balance_ratio_ * shoulder_delta);

                value += tof_ik_elbow_only_penalty_ * elbow_only;
                value -= tof_ik_shoulder_bonus_ * shoulder_delta;
            }

            if (use_tof_approach_ik_bias() && has_frozen_pre_grasp_point_ &&
                has_locked_object_point_)
            {
                const auto candidate_point =
                    wrist_to_grasp_point(fk_point_from_solution(psi_raw, a1, a2));
                const double axis_x = locked_object_point_.x - frozen_pre_grasp_point_.x;
                const double axis_y = locked_object_point_.y - frozen_pre_grasp_point_.y;
                const double axis_z = locked_object_point_.z - frozen_pre_grasp_point_.z;
                const double axis_n = std::sqrt(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);
                if (axis_n > 1e-6)
                {
                    const double progress =
                        ((candidate_point.x - frozen_pre_grasp_point_.x) * axis_x +
                         (candidate_point.y - frozen_pre_grasp_point_.y) * axis_y +
                         (candidate_point.z - frozen_pre_grasp_point_.z) * axis_z) /
                        axis_n;
                    const double height_error = std::fabs(candidate_point.z - p.z);

                    value -= 3.0 * progress;
                    value += 4.0 * height_error;
                }
            }

            return value;
        };

        commanded_goal_point_ = p;
        has_commanded_goal_point_ = true;

        std::vector<double> best_goal;
        double best_cost = std::numeric_limits<double>::infinity();
        bool has_valid_candidate = false;
        std::string selected_candidate{"none"};

        struct IkCandidateDebug
        {
            bool available{false};
            bool valid{false};
            double shoulder{0.0};
            double elbow{0.0};
            double wrist{0.0};
            double cost{std::numeric_limits<double>::quiet_NaN()};
        };

        IkCandidateDebug up_debug;
        IkCandidateDebug dn_debug;

        auto try_candidate = [&](const char *name, bool ok, double shoulder_pitch, double elbow,
                                 IkCandidateDebug &debug)
        {
            if (!ok)
            {
                return;
            }

            debug.available = true;

            auto candidate = goal_;
            candidate[0] = psi;
            candidate[1] = shoulder_pitch;
            candidate[2] = elbow;
            const double wrist_pitch_raw =
                -(candidate[1] + candidate[2]) + wrist_pitch_level_bias_;
            candidate[3] = nearest_equivalent_angle(wrist_pitch_raw, q_meas_[3]);
            (void)wrist_roll_ref;
            candidate[4] = 0.0;

            debug.shoulder = candidate[1];
            debug.elbow = candidate[2];
            debug.wrist = candidate[3];

            if (!goal_within_joint_limits(candidate, false))
            {
                return;
            }

            const double c = cost(shoulder_pitch, elbow);
            debug.valid = true;
            debug.cost = c;
            if (!has_valid_candidate || c < best_cost)
            {
                best_goal = candidate;
                best_cost = c;
                has_valid_candidate = true;
                selected_candidate = name;
            }
        };

        try_candidate("up", ok_up, shoulder_pitch_up, elbow_up, up_debug);
        try_candidate("dn", ok_dn, shoulder_pitch_dn, elbow_dn, dn_debug);

        log_ik_candidates(up_debug, dn_debug, selected_candidate);

        if (!has_valid_candidate)
        {
            auto rejected = goal_;
            rejected[0] = psi;
            rejected[1] = ok_up ? shoulder_pitch_up : shoulder_pitch_dn;
            rejected[2] = ok_up ? elbow_up : elbow_dn;
            rejected[3] = nearest_equivalent_angle(
                -(rejected[1] + rejected[2]) + wrist_pitch_level_bias_, q_meas_[3]);
            rejected[4] = 0.0;
            goal_within_joint_limits(rejected, true);
            target_valid_ = false;
            return false;
        }

        goal_ = best_goal;
        goal_within_joint_limits(goal_, false);
        target_valid_ = true;
        return true;
    }

    double arm_goal_error_sum() const
    {
        return std::fabs(wrap_pi(goal_[0] - q_meas_[0])) +
               std::fabs(wrap_pi(goal_[1] - q_meas_[1])) +
               std::fabs(wrap_pi(goal_[2] - q_meas_[2])) +
               0.5 * std::fabs(wrap_pi(goal_[3] - q_meas_[3]));
    }

    bool arm_goal_reached(double tol) const
    {
        return arm_goal_error_sum() <= tol;
    }

    static double point_distance(const TargetPoint &a, const TargetPoint &b)
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    bool cartesian_goal_reached(double tol) const
    {
        geometry_msgs::msg::Point current;
        if (!lookup_current_arm_point(current))
        {
            return false;
        }

        const double err = cartesian_error_to_goal(current);
        return std::isfinite(err) && err <= tol;
    }

    bool freeze_ready() const
    {
        return has_locked_object_point_ &&
               arm_goal_reached(joint_reach_tol_rad_) &&
               cartesian_goal_reached(freeze_cartesian_tol_m_);
    }

    bool use_tof_approach_ik_bias() const
    {
        return auto_phase_ == AutoPhase::TOF_APPROACH ||
               auto_phase_ == AutoPhase::GRIPPER_CLOSE ||
               auto_phase_ == AutoPhase::GRASP_HOLD;
    }

    static double ik_elbow_to_joint_elbow(double ik_elbow)
    {
        return wrap_pi(0.5 * M_PI - ik_elbow);
    }

    static double joint_elbow_to_ik_elbow(double joint_elbow)
    {
        return wrap_pi(0.5 * M_PI - joint_elbow);
    }

    static double ik_shoulder_to_joint_shoulder(double ik_shoulder, double joint_elbow)
    {
        return wrap_pi(ik_shoulder - joint_elbow);
    }

    static double joint_shoulder_to_ik_shoulder(double joint_shoulder, double joint_elbow)
    {
        return wrap_pi(joint_shoulder + joint_elbow);
    }

    static TargetVector normalized_vector(const TargetVector &v, const TargetVector &fallback)
    {
        const double n = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (n <= 1e-9 || !std::isfinite(n))
        {
            return fallback;
        }

        return {v.x / n, v.y / n, v.z / n};
    }

    static TargetVector cross_vector(const TargetVector &a, const TargetVector &b)
    {
        return {
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
    }

    double mean_bearing_yaw() const
    {
        double c = 0.0;
        double s = 0.0;
        for (const auto &p : target_lock_buffer_)
        {
            c += std::cos(p.bearing_yaw);
            s += std::sin(p.bearing_yaw);
        }
        return std::atan2(s, c);
    }

    double mean_bearing_pitch() const
    {
        if (target_lock_buffer_.empty())
        {
            return 0.0;
        }

        double sum = 0.0;
        for (const auto &p : target_lock_buffer_)
        {
            sum += p.bearing_pitch;
        }
        return sum / static_cast<double>(target_lock_buffer_.size());
    }

    TargetPoint fk_point_from_solution(double psi_raw, double shoulder_joint, double elbow_joint) const
    {
        const double shoulder_pitch = joint_shoulder_to_ik_shoulder(shoulder_joint, elbow_joint);
        const double elbow_ik = joint_elbow_to_ik_elbow(elbow_joint);
        const double z_plane =
            shoulder_pitch_offset_y_ +
            L1_ * std::cos(shoulder_pitch) +
            L2_ * std::cos(shoulder_pitch + elbow_ik);
        const double y_plane =
            shoulder_pitch_offset_z_ +
            L1_ * std::sin(shoulder_pitch) +
            L2_ * std::sin(shoulder_pitch + elbow_ik);

        const double c = std::cos(psi_raw);
        const double s = std::sin(psi_raw);

        TargetPoint out;
        out.x = shoulder_rot_x_ + s * z_plane;
        out.y = shoulder_rot_y_ - c * z_plane;
        out.z = shoulder_rot_z_ + y_plane;
        return out;
    }

    bool approach_axis(TargetVector &axis) const
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

    bool wrist_to_grasp_offset_base(TargetVector &offset) const
    {
        TargetVector forward;
        if (!approach_axis(forward))
        {
            return false;
        }

        const TargetVector world_up{0.0, 0.0, 1.0};
        TargetVector side = normalized_vector(cross_vector(world_up, forward), {1.0, 0.0, 0.0});
        if (std::fabs(side.x) < 1e-6 && std::fabs(side.y) < 1e-6 && std::fabs(side.z) < 1e-6)
        {
            side = {1.0, 0.0, 0.0};
        }

        TargetVector up = normalized_vector(cross_vector(forward, side), world_up);

        offset.x = wrist_to_grasp_local_x_m_ * side.x +
                   wrist_to_grasp_forward_offset_m_ * forward.x +
                   wrist_to_grasp_local_z_m_ * up.x;
        offset.y = wrist_to_grasp_local_x_m_ * side.y +
                   wrist_to_grasp_forward_offset_m_ * forward.y +
                   wrist_to_grasp_local_z_m_ * up.y;
        offset.z = wrist_to_grasp_local_x_m_ * side.z +
                   wrist_to_grasp_forward_offset_m_ * forward.z +
                   wrist_to_grasp_local_z_m_ * up.z;
        return true;
    }

    TargetPoint grasp_to_wrist_target(const TargetPoint &grasp_point) const
    {
        TargetVector offset;
        if (!wrist_to_grasp_offset_base(offset))
        {
            return grasp_point;
        }

        TargetPoint out = grasp_point;
        out.x -= offset.x;
        out.y -= offset.y;
        out.z -= offset.z;
        return out;
    }

    TargetPoint wrist_to_grasp_point(const TargetPoint &wrist_point) const
    {
        TargetVector offset;
        if (!wrist_to_grasp_offset_base(offset))
        {
            return wrist_point;
        }

        TargetPoint out = wrist_point;
        out.x += offset.x;
        out.y += offset.y;
        out.z += offset.z;
        return out;
    }

    void freeze_pre_grasp_point()
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

    double elapsed_in_phase() const
    {
        if (phase_start_time_sec_ < 0.0)
        {
            return 0.0;
        }
        return this->now().seconds() - phase_start_time_sec_;
    }

    double tof_age_sec() const
    {
        if (!has_tof_range_ || last_tof_time_sec_ < 0.0)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return this->now().seconds() - last_tof_time_sec_;
    }

    bool tof_range_valid() const
    {
        const double age = tof_age_sec();
        return has_tof_range_ &&
               std::isfinite(latest_tof_range_m_) &&
               std::isfinite(age) &&
               age <= tof_timeout_sec_;
    }

    bool init_tof_joint_approach()
    {
        if (!has_frozen_pre_grasp_joints_ ||
            frozen_pre_grasp_joint_positions_.size() != joint_name_.size() ||
            !tof_range_valid())
        {
            return false;
        }

        last_safe_tof_joint_positions_ = goal_;
        if (last_safe_tof_joint_positions_.size() != joint_name_.size())
        {
            last_safe_tof_joint_positions_ = frozen_pre_grasp_joint_positions_;
        }
        if (last_safe_tof_joint_positions_.size() != joint_name_.size())
        {
            return false;
        }
        last_safe_tof_joint_positions_[4] = 0.0;
        last_safe_tof_joint_positions_[5] = gripper_open_pos_;
        if (!goal_within_joint_limits(last_safe_tof_joint_positions_, false))
        {
            return false;
        }
        tof_pending_probe_ = false;
        tof_probe_failed_mask_ = 0;
        tof_probe_failed_count_ = 0;
        tof_preferred_probe_index_ = 0;
        tof_approach_faulted_ = false;
        tof_approach_blocked_ = false;
        tof_forward_distance_m_ = 0.0;
        last_tof_approach_range_m_ = latest_tof_range_m_;
        last_tof_forward_progress_m_ = current_forward_progress();
        return true;
    }

    struct TofProbeCandidate
    {
        double shoulder_scale{0.0};
        double elbow_scale{0.0};
        double wrist_scale{0.0};
    };

    std::vector<TofProbeCandidate> tof_probe_candidates() const
    {
        const double sh = tof_joint_probe_shoulder_ratio_;
        const double wr = tof_joint_probe_wrist_ratio_;
        return {
            {0.0, 1.0, -wr},
            {0.0, 1.0, -0.5 * wr},
            {sh, 1.0, -wr},
            {-sh, 1.0, -wr},
            {0.0, 0.5, -wr},
            {0.0, 0.0, -wr}};
    }

    bool issue_tof_joint_probe()
    {
        if (last_safe_tof_joint_positions_.size() != joint_name_.size() ||
            !tof_range_valid())
        {
            tof_approach_blocked_ = true;
            return false;
        }

        const auto probes = tof_probe_candidates();
        const size_t n = probes.size();
        if (n == 0)
        {
            tof_approach_blocked_ = true;
            return false;
        }

        const double step = std::max(0.0, tof_joint_probe_step_rad_);
        for (size_t k = 0; k < n; ++k)
        {
            const size_t idx = (static_cast<size_t>(tof_preferred_probe_index_) + k) % n;
            if ((tof_probe_failed_mask_ & (1u << idx)) != 0)
            {
                continue;
            }

            auto candidate = last_safe_tof_joint_positions_;
            candidate[1] += probes[idx].shoulder_scale * step;
            candidate[2] += probes[idx].elbow_scale * step;
            candidate[3] += probes[idx].wrist_scale * step;
            candidate[4] = 0.0;
            candidate[5] = gripper_open_pos_;

            if (!goal_within_joint_limits(candidate, false))
            {
                tof_probe_failed_mask_ |= (1u << idx);
                ++tof_probe_failed_count_;
                continue;
            }

            goal_ = candidate;
            target_valid_ = true;
            command_time_from_start_sec_ = tof_joint_probe_command_time_sec_;
            tof_pending_probe_ = true;
            tof_active_probe_index_ = static_cast<int>(idx);
            tof_probe_start_time_sec_ = this->now().seconds();
            tof_probe_start_range_m_ = latest_tof_range_m_;
            tof_probe_start_progress_m_ = current_forward_progress();
            RCLCPP_INFO(
                this->get_logger(),
                "TOF joint probe idx=%zu sh=%.1f el=%.1f wrist=%.1f start_range=%.4f",
                idx,
                display_joint_value(1, goal_[1]),
                display_joint_value(2, goal_[2]),
                display_joint_value(3, goal_[3]),
                tof_probe_start_range_m_);
            return true;
        }

        tof_approach_blocked_ = true;
        return false;
    }

    bool evaluate_tof_joint_probe()
    {
        goal_[5] = gripper_open_pos_;
        const double elapsed = this->now().seconds() - tof_probe_start_time_sec_;
        if (elapsed < tof_joint_probe_command_time_sec_)
        {
            return true;
        }

        if (!arm_goal_reached(tof_joint_probe_reach_tol_rad_))
        {
            return true;
        }

        if (!tof_range_valid())
        {
            return true;
        }

        const double range_delta = tof_probe_start_range_m_ - latest_tof_range_m_;
        const double progress = current_forward_progress();
        const double progress_delta =
            std::isfinite(progress) && std::isfinite(tof_probe_start_progress_m_)
                ? progress - tof_probe_start_progress_m_
                : std::numeric_limits<double>::quiet_NaN();
        const bool range_increased = range_delta <= -tof_fail_range_increase_m_;

        if (tof_target_inside_gripper() || range_delta >= tof_success_range_delta_m_)
        {
            last_safe_tof_joint_positions_ = goal_;
            tof_pending_probe_ = false;
            tof_preferred_probe_index_ = std::max(0, tof_active_probe_index_);
            tof_probe_failed_mask_ = 0;
            tof_probe_failed_count_ = 0;
            tof_forward_distance_m_ = clamp(
                tof_forward_distance_m_ + std::max(0.0, tof_approach_step_m_),
                min_forward_approach_m_,
                max_forward_approach_m_);
            last_tof_approach_range_m_ = latest_tof_range_m_;
            last_tof_forward_progress_m_ = progress;
            RCLCPP_INFO(
                this->get_logger(),
                "TOF joint probe accepted idx=%d range_delta=%.4f progress_delta=%.4f",
                tof_active_probe_index_, range_delta, progress_delta);
            return true;
        }

        {
            if (tof_active_probe_index_ >= 0)
            {
                tof_probe_failed_mask_ |= (1u << static_cast<size_t>(tof_active_probe_index_));
            }
            ++tof_probe_failed_count_;
            goal_ = last_safe_tof_joint_positions_;
            goal_[4] = 0.0;
            goal_[5] = gripper_open_pos_;
            tof_pending_probe_ = false;
            RCLCPP_WARN(
                this->get_logger(),
                "TOF joint probe rejected idx=%d range_delta=%.4f progress_delta=%.4f increased=%d failed=%d",
                tof_active_probe_index_, range_delta, progress_delta,
                range_increased ? 1 : 0, tof_probe_failed_count_);
        }

        if (tof_probe_failed_count_ >= tof_max_failed_probe_count_)
        {
            tof_approach_blocked_ = true;
            return false;
        }

        return true;
    }

    bool run_tof_joint_probe_step()
    {
        if (tof_approach_blocked_)
        {
            return false;
        }

        if (tof_pending_probe_)
        {
            return evaluate_tof_joint_probe();
        }

        if (last_safe_tof_joint_positions_.size() == joint_name_.size() &&
            !arm_goal_reached(tof_joint_probe_reach_tol_rad_))
        {
            goal_ = last_safe_tof_joint_positions_;
            goal_[4] = 0.0;
            goal_[5] = gripper_open_pos_;
            target_valid_ = goal_within_joint_limits(goal_, false);
            return target_valid_;
        }

        return issue_tof_joint_probe();
    }

    bool hold_last_safe_tof_joint_goal()
    {
        if (last_safe_tof_joint_positions_.size() != joint_name_.size())
        {
            return false;
        }

        goal_ = last_safe_tof_joint_positions_;
        goal_[4] = 0.0;
        goal_[5] = gripper_close_pos_;
        target_valid_ = goal_within_joint_limits(goal_, false);
        return target_valid_;
    }

    bool plan_tof_approach_point()
    {
        if (!has_frozen_pre_grasp_point_ ||
            !has_frozen_approach_axis_ ||
            !tof_range_valid())
        {
            return false;
        }

        tof_forward_distance_m_ = min_forward_approach_m_;
        update_tof_approach_point_from_forward_distance();
        has_tof_approach_point_ = true;
        tof_approach_faulted_ = false;
        last_tof_approach_range_m_ = latest_tof_range_m_;
        last_tof_forward_progress_m_ = current_forward_progress();
        return true;
    }

    bool extend_tof_approach_point()
    {
        if (!has_frozen_pre_grasp_point_ ||
            !has_frozen_approach_axis_ ||
            tof_target_inside_gripper())
        {
            return false;
        }

        const double next_forward = clamp(
            tof_forward_distance_m_ + std::max(0.0, tof_approach_step_m_),
            min_forward_approach_m_,
            max_forward_approach_m_);
        if (next_forward <= tof_forward_distance_m_ + 1e-6)
        {
            return false;
        }

        tof_forward_distance_m_ = next_forward;
        update_tof_approach_point_from_forward_distance();
        has_tof_approach_point_ = true;
        last_tof_approach_range_m_ = latest_tof_range_m_;
        last_tof_forward_progress_m_ = current_forward_progress();
        return true;
    }

    double current_forward_progress() const
    {
        if (!has_frozen_pre_grasp_point_ || !has_frozen_approach_axis_)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        geometry_msgs::msg::Point current;
        if (!lookup_current_arm_point(current))
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        TargetVector axis = signed_tof_approach_axis();
        return (current.x - frozen_pre_grasp_point_.x) * axis.x +
               (current.y - frozen_pre_grasp_point_.y) * axis.y +
               (current.z - frozen_pre_grasp_point_.z) * axis.z;
    }

    bool tof_approach_progress_ok()
    {
        if (tof_approach_faulted_)
        {
            return false;
        }

        const double progress = current_forward_progress();
        if (std::isfinite(progress))
        {
            if (progress < -tof_progress_reverse_tol_m_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "TOF approach stopped: gripper moved backward, progress=%.4f m",
                            progress);
                tof_approach_faulted_ = true;
                tof_approach_blocked_ = true;
                return false;
            }

            if (std::isfinite(last_tof_forward_progress_m_) &&
                progress + tof_progress_reverse_tol_m_ < last_tof_forward_progress_m_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "TOF approach stopped: gripper progress decreased %.4f -> %.4f m",
                            last_tof_forward_progress_m_, progress);
                tof_approach_faulted_ = true;
                tof_approach_blocked_ = true;
                return false;
            }
        }

        if (tof_range_valid() && std::isfinite(last_tof_approach_range_m_) &&
            latest_tof_range_m_ > last_tof_approach_range_m_ + tof_range_increase_tol_m_)
        {
            if (!tof_approach_reverse_retry_used_)
            {
                tof_approach_axis_sign_ *= -1.0;
                tof_approach_reverse_retry_used_ = true;
                RCLCPP_WARN(this->get_logger(),
                            "TOF approach stopped: range increased %.4f -> %.4f m, reversing approach sign to %.0f",
                            last_tof_approach_range_m_, latest_tof_range_m_, tof_approach_axis_sign_);
            }
            else
            {
                tof_approach_blocked_ = true;
                RCLCPP_WARN(this->get_logger(),
                            "TOF approach blocked: range increased again %.4f -> %.4f m",
                            last_tof_approach_range_m_, latest_tof_range_m_);
            }
            tof_approach_faulted_ = true;
            return false;
        }

        geometry_msgs::msg::Point current;
        if (arm_goal_reached(approach_reach_tol_rad_) &&
            lookup_current_arm_point(current))
        {
            const double cart_err = cartesian_error_to_goal(current);
            if (std::isfinite(cart_err) &&
                cart_err > tof_cartesian_error_stop_m_)
            {
                RCLCPP_WARN(this->get_logger(),
                            "TOF approach stopped: joint goal reached but cartesian error is %.4f m",
                            cart_err);
                tof_approach_faulted_ = true;
                tof_approach_blocked_ = true;
                return false;
            }
        }

        return true;
    }

    void stop_tof_approach_and_hold()
    {
        tof_forward_distance_m_ = 0.0;
        tof_approach_point_ = frozen_pre_grasp_point_;
        has_tof_approach_point_ = false;
        grasp_start_requested_ = false;
        auto_phase_ = AutoPhase::PRE_GRASP_HOLD;
        phase_start_time_sec_ = -1.0;
        hold_start_time_sec_ = this->now().seconds();
        hold_reached_logged_ = true;
    }

    TargetVector signed_tof_approach_axis() const
    {
        return {
            tof_approach_axis_sign_ * frozen_approach_axis_.x,
            tof_approach_axis_sign_ * frozen_approach_axis_.y,
            tof_approach_axis_sign_ * frozen_approach_axis_.z};
    }

    void update_tof_approach_point_from_forward_distance()
    {
        TargetVector axis = signed_tof_approach_axis();
        tof_approach_point_.x =
            frozen_pre_grasp_point_.x + tof_forward_distance_m_ * axis.x;
        tof_approach_point_.y =
            frozen_pre_grasp_point_.y + tof_forward_distance_m_ * axis.y;
        tof_approach_point_.z =
            frozen_pre_grasp_point_.z + tof_forward_distance_m_ * axis.z;
    }

    bool tof_target_inside_gripper() const
    {
        return tof_range_valid() &&
               latest_tof_range_m_ <= gripper_inside_range_m_;
    }

    bool lookup_sensor_to_grasp_perpendicular_offset(const TargetVector &axis,
                                                     TargetVector &perp_out) const
    {
        if (!tf_buffer_)
        {
            return false;
        }

        try
        {
            const auto tf_sensor = tf_buffer_->lookupTransform(
                arm_base_frame_, target_sensor_frame_, tf2::TimePointZero);
            const auto tf_grasp = tf_buffer_->lookupTransform(
                arm_base_frame_, end_effector_frame_, tf2::TimePointZero);

            const double ox =
                tf_grasp.transform.translation.x - tf_sensor.transform.translation.x;
            const double oy =
                tf_grasp.transform.translation.y - tf_sensor.transform.translation.y;
            const double oz =
                tf_grasp.transform.translation.z - tf_sensor.transform.translation.z;
            const double parallel = ox * axis.x + oy * axis.y + oz * axis.z;

            perp_out.x = ox - parallel * axis.x;
            perp_out.y = oy - parallel * axis.y;
            perp_out.z = oz - parallel * axis.z;
            return true;
        }
        catch (const tf2::TransformException &)
        {
            return false;
        }
    }

    static double nearest_equivalent_angle(double target, double reference)
    {
        return reference + wrap_pi(target - reference);
    }

    static geometry_msgs::msg::Point to_point_msg(const TargetPoint &p)
    {
        geometry_msgs::msg::Point out;
        out.x = p.x;
        out.y = p.y;
        out.z = p.z;
        return out;
    }

    const char *phase_name() const
    {
        switch (auto_phase_)
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
        case AutoPhase::GRASP_HOLD:
            return "GRASP_HOLD";
        }
        return "UNKNOWN";
    }

    bool lookup_current_arm_point(geometry_msgs::msg::Point &out) const
    {
        if (!tf_buffer_)
        {
            return false;
        }

        try
        {
            const auto tf = tf_buffer_->lookupTransform(
                arm_base_frame_, end_effector_frame_, tf2::TimePointZero);
            out.x = tf.transform.translation.x;
            out.y = tf.transform.translation.y;
            out.z = tf.transform.translation.z;
            return true;
        }
        catch (const tf2::TransformException &)
        {
            return false;
        }
    }

    double cartesian_error_to_goal(const geometry_msgs::msg::Point &current) const
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

    double current_target_lock_max_deviation() const
    {
        if (target_lock_buffer_.empty())
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return max_locked_target_deviation(mean_locked_target());
    }

    double current_target_lock_max_bearing_deviation() const
    {
        if (target_lock_buffer_.empty())
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return max_locked_bearing_deviation(mean_locked_target());
    }

    double current_hold_elapsed_sec() const
    {
        if (auto_phase_ != AutoPhase::PRE_GRASP_HOLD || hold_start_time_sec_ < 0.0)
        {
            return 0.0;
        }
        return this->now().seconds() - hold_start_time_sec_;
    }

    double current_hold_target_drift() const
    {
        if (!has_frozen_pre_grasp_point_ || !has_tracked_pre_grasp_candidate_)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return point_distance(tracked_pre_grasp_candidate_, frozen_pre_grasp_point_);
    }

    double shoulder_pitch_delta_from_hold() const
    {
        if (!has_frozen_pre_grasp_joints_ || frozen_pre_grasp_joint_positions_.size() <= 1)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return wrap_pi(goal_[1] - frozen_pre_grasp_joint_positions_[1]);
    }

    double elbow_delta_from_hold() const
    {
        if (!has_frozen_pre_grasp_joints_ || frozen_pre_grasp_joint_positions_.size() <= 2)
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        return wrap_pi(goal_[2] - frozen_pre_grasp_joint_positions_[2]);
    }

    double shoulder_to_elbow_delta_ratio() const
    {
        const double d_sh = shoulder_pitch_delta_from_hold();
        const double d_el = elbow_delta_from_hold();
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

    void publish_auto_debug()
    {
        gimbal_mani::msg::ArmAutoDebug msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = arm_base_frame_;

        msg.auto_enabled = auto_enabled_;
        msg.phase = phase_name();

        msg.target_valid = target_valid_;
        msg.has_joint_state = has_joint_state_;
        msg.has_locked_object_point = has_locked_object_point_;

        msg.base_frame = arm_base_frame_;
        msg.end_effector_frame = end_effector_frame_;

        msg.object_point_base = to_point_msg(locked_object_point_);
        msg.pre_grasp_hold_point_base = to_point_msg(pre_grasp_point_);
        msg.commanded_goal_point_base = to_point_msg(commanded_goal_point_);
        msg.frozen_pre_grasp_point_base = to_point_msg(frozen_pre_grasp_point_);
        msg.tracked_pre_grasp_candidate_base = to_point_msg(tracked_pre_grasp_candidate_);

        geometry_msgs::msg::Point current_arm_point;
        msg.has_current_arm_point = lookup_current_arm_point(current_arm_point);
        msg.current_arm_point_base = current_arm_point;
        msg.cartesian_error_to_goal_m = msg.has_current_arm_point
                                            ? cartesian_error_to_goal(current_arm_point)
                                            : std::numeric_limits<double>::quiet_NaN();
        msg.freeze_cartesian_tol_m = freeze_cartesian_tol_m_;
        msg.cartesian_goal_reached = msg.has_current_arm_point &&
                                     std::isfinite(msg.cartesian_error_to_goal_m) &&
                                     msg.cartesian_error_to_goal_m <= freeze_cartesian_tol_m_;
        msg.freeze_ready = has_locked_object_point_ &&
                           arm_goal_reached(joint_reach_tol_rad_) &&
                           msg.cartesian_goal_reached;
        msg.hold_frozen = has_frozen_pre_grasp_point_;
        msg.hold_target_drift_m = current_hold_target_drift();
        msg.hold_target_exit_tol_m = hold_target_exit_tol_m_;
        msg.grasp_start_requested = grasp_start_requested_;
        msg.has_tof_range = tof_range_valid();
        msg.tof_range_m = latest_tof_range_m_;
        msg.tof_age_sec = tof_age_sec();
        msg.desired_grasp_range_m = desired_grasp_range_m_;
        msg.gripper_inside_range_m = gripper_inside_range_m_;
        msg.gripper_inner_extra_approach_m = gripper_inner_extra_approach_m_;
        msg.tof_approach_step_m = tof_approach_step_m_;
        msg.tof_forward_distance_m = tof_forward_distance_m_;
        msg.tof_target_inside_gripper = tof_target_inside_gripper();
        msg.has_tof_approach_point = has_tof_approach_point_;
        msg.tof_approach_point_base = to_point_msg(tof_approach_point_);
        msg.has_frozen_pre_grasp_joints = has_frozen_pre_grasp_joints_;
        msg.shoulder_pitch_delta_from_hold_rad = shoulder_pitch_delta_from_hold();
        msg.elbow_delta_from_hold_rad = elbow_delta_from_hold();
        msg.shoulder_to_elbow_delta_ratio = shoulder_to_elbow_delta_ratio();

        msg.current_joint_positions = q_meas_;
        msg.target_joint_positions = goal_;
        msg.joint_error_sum = arm_goal_error_sum();
        msg.joint_reach_tol_rad = joint_reach_tol_rad_;
        msg.joint_goal_reached = arm_goal_reached(joint_reach_tol_rad_);

        msg.hold_elapsed_sec = current_hold_elapsed_sec();
        msg.hold_time_sec = hold_time_sec_;

        msg.target_lock_samples = static_cast<int32_t>(target_lock_buffer_.size());
        msg.target_lock_min_samples = target_lock_min_samples_;
        msg.target_lock_max_deviation_m = current_target_lock_max_deviation();
        msg.target_lock_pos_tol_m = target_lock_pos_tol_m_;

        pub_auto_debug_->publish(msg);
    }

    void publish_auto_status()
    {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3)
           << "phase=" << phase_name()
           << " tof=" << latest_tof_range_m_
           << " in=" << (tof_target_inside_gripper() ? 1 : 0)
           << " fwd=" << tof_forward_distance_m_
           << " probe=" << tof_active_probe_index_
           << " fails=" << tof_probe_failed_count_
           << " block=" << (tof_approach_blocked_ ? 1 : 0)
           << " bear=" << current_target_lock_max_bearing_deviation()
           << " limit=" << (last_limit_rejection_.empty() ? "ok" : last_limit_rejection_)
           << " sh=" << shoulder_pitch_delta_from_hold()
           << " el=" << elbow_delta_from_hold()
           << " r=" << shoulder_to_elbow_delta_ratio();

        std_msgs::msg::String msg;
        msg.data = ss.str();
        pub_auto_status_->publish(msg);
    }

    rcl_interfaces::msg::SetParametersResult on_parameters_changed(const std::vector<rclcpp::Parameter> &params)
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

        return result;
    }

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
    double gripper_inside_range_m_{0.035};
    double gripper_inner_extra_approach_m_{0.02};
    double tof_approach_step_m_{0.01};
    double min_forward_approach_m_{0.0};
    double max_forward_approach_m_{0.085};
    double tof_timeout_sec_{0.5};
    double gripper_open_pos_{0.8};
    double gripper_close_pos_{0.0};
    double gripper_action_time_sec_{0.8};
    double approach_reach_tol_rad_{0.25};
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
    std::deque<TargetSample> target_lock_buffer_;
    TargetPoint locked_object_point_{};
    TargetVector locked_approach_axis_{};
    TargetPoint pre_grasp_point_{};
    TargetPoint commanded_goal_point_{};
    TargetPoint frozen_pre_grasp_point_{};
    TargetPoint tracked_pre_grasp_candidate_{};
    TargetPoint tof_approach_point_{};
    std::vector<double> frozen_pre_grasp_joint_positions_;
    bool has_locked_object_point_{false};
    bool has_commanded_goal_point_{false};
    bool has_frozen_pre_grasp_point_{false};
    bool has_tracked_pre_grasp_candidate_{false};
    bool has_frozen_pre_grasp_joints_{false};
    bool has_tof_approach_point_{false};
    bool has_frozen_approach_axis_{false};
    TargetVector frozen_approach_axis_{};
    double locked_object_yaw_{0.0};
    double hold_start_time_sec_{-1.0};
    double phase_start_time_sec_{-1.0};
    bool hold_reached_logged_{false};
    bool grasp_start_requested_{false};
    bool has_tof_range_{false};
    double latest_tof_range_m_{std::numeric_limits<double>::quiet_NaN()};
    double last_tof_time_sec_{-1.0};
    double tof_forward_distance_m_{0.0};
    double last_tof_approach_range_m_{std::numeric_limits<double>::quiet_NaN()};
    double last_tof_forward_progress_m_{std::numeric_limits<double>::quiet_NaN()};
    bool tof_approach_faulted_{false};
    bool tof_approach_blocked_{false};
    bool tof_approach_reverse_retry_used_{false};
    double tof_approach_axis_sign_{1.0};
    std::vector<double> last_safe_tof_joint_positions_;
    bool tof_pending_probe_{false};
    int tof_active_probe_index_{-1};
    int tof_preferred_probe_index_{0};
    uint32_t tof_probe_failed_mask_{0};
    int tof_probe_failed_count_{0};
    double tof_probe_start_range_m_{std::numeric_limits<double>::quiet_NaN()};
    double tof_probe_start_progress_m_{std::numeric_limits<double>::quiet_NaN()};
    double tof_probe_start_time_sec_{-1.0};
    TargetVector last_valid_approach_axis_{};

    std::vector<std::string> joint_name_;
    std::vector<double> goal_;
    std::vector<double> home_goal_;
    std::vector<double> q_meas_{std::vector<double>(6, 0.0)};
    std::string last_limit_rejection_;
    bool has_joint_state_{false};

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_traj_;
    rclcpp::Publisher<gimbal_mani::msg::ArmAutoDebug>::SharedPtr pub_auto_debug_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_auto_status_;
    rclcpp::Subscription<gimbal_mani::msg::TargetBearingRange>::SharedPtr sub_target_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_home_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_auto_enable_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_grasp_start_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_tof_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Arm_UpperLayerMain>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
