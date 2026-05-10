#include <cmath>
#include <string>
#include <vector>
#include <deque>
#include <chrono>
#include <algorithm>
#include <limits>
#include <sstream>
#include <iomanip>

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

class Arm_UpperLayerMain : public rclcpp::Node
{
public:
    Arm_UpperLayerMain() : Node("arm_upper_layer_main")
    {
        L1_ = this->declare_parameter<double>("L1", 0.116);
        L2_ = this->declare_parameter<double>("L2", 0.135);

        saturate_reach_ = this->declare_parameter<bool>("saturate_reach", true);
        arm_base_frame_ = this->declare_parameter<std::string>("arm_base_frame", "Base");
        end_effector_frame_ = this->declare_parameter<std::string>("end_effector_frame", "gimbal_mount_link");
        min_effective_range_ = this->declare_parameter<double>("min_effective_range", 0.0);
        wrist_pitch_level_bias_ = this->declare_parameter<double>("wrist_pitch_level_bias", -1.5);

        pre_grasp_distance_m_ =
            this->declare_parameter<double>("pre_grasp_distance_m", 0.02);
        pre_grasp_lift_z_m_ =
            this->declare_parameter<double>("pre_grasp_lift_z_m", 0.03);
        pre_grasp_lateral_offset_m_ =
            this->declare_parameter<double>("pre_grasp_lateral_offset_m", 0.023);

        target_lock_min_samples_ = this->declare_parameter<int>("target_lock_min_samples", 8);
        target_lock_pos_tol_m_ = this->declare_parameter<double>("target_lock_pos_tol_m", 0.01);
        hold_time_sec_ = this->declare_parameter<double>("hold_time_sec", 0.4);
        joint_reach_tol_rad_ = this->declare_parameter<double>("joint_reach_tol_rad", 0.25);
        hold_exit_tol_rad_ = this->declare_parameter<double>("hold_exit_tol_rad", 0.35);
        target_filter_alpha_ = this->declare_parameter<double>("target_filter_alpha", 0.05);
        freeze_cartesian_tol_m_ = this->declare_parameter<double>("freeze_cartesian_tol_m", 0.5);
        hold_target_exit_tol_m_ = this->declare_parameter<double>("hold_target_exit_tol_m", 0.02);
        desired_grasp_range_m_ = this->declare_parameter<double>("desired_grasp_range_m", 0.03);
        gripper_inside_range_m_ = this->declare_parameter<double>("gripper_inside_range_m", 0.035);
        gripper_inner_extra_approach_m_ = this->declare_parameter<double>("gripper_inner_extra_approach_m", 0.02);
        tof_approach_step_m_ = this->declare_parameter<double>("tof_approach_step_m", 0.01);
        min_forward_approach_m_ = this->declare_parameter<double>("min_forward_approach_m", 0.0);
        max_forward_approach_m_ = this->declare_parameter<double>("max_forward_approach_m", 0.2);
        tof_timeout_sec_ = this->declare_parameter<double>("tof_timeout_sec", 0.5);
        gripper_open_pos_ = this->declare_parameter<double>("gripper_open_pos", 0.5);
        gripper_close_pos_ = this->declare_parameter<double>("gripper_close_pos", 0.0);
        gripper_action_time_sec_ = this->declare_parameter<double>("gripper_action_time_sec", 0.4);
        approach_reach_tol_rad_ = this->declare_parameter<double>("approach_reach_tol_rad", 0.25);
        tof_ik_elbow_only_penalty_ = this->declare_parameter<double>("tof_ik_elbow_only_penalty", 8.0);
        tof_ik_shoulder_bonus_ = this->declare_parameter<double>("tof_ik_shoulder_bonus", 1.0);
        tof_ik_shoulder_balance_ratio_ = this->declare_parameter<double>("tof_ik_shoulder_balance_ratio", 0.8);

        shoulder_rot_x_ = this->declare_parameter<double>("shoulder_rot_x", 0.0);
        shoulder_rot_y_ = this->declare_parameter<double>("shoulder_rot_y", -0.0452);
        shoulder_rot_z_ = this->declare_parameter<double>("shoulder_rot_z", 0.0165);

        shoulder_pitch_offset_y_ = this->declare_parameter<double>("shoulder_pitch_offset_y", 0.1025);
        shoulder_pitch_offset_z_ = this->declare_parameter<double>("shoulder_pitch_offset_z", 0.0306);

        joint_name_ = {
            "Shoulder_Rotation",
            "Shoulder_Pitch",
            "Elbow",
            "Wrist_Pitch",
            "Wrist_Roll",
            "Gripper"};
        goal_.assign(joint_name_.size(), 0.0);
        home_goal_ = {0.0, -1.4, 1.4, -1.0, 0.0, 0.5};

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

        const double publish_hz = this->declare_parameter<double>("pub_hz", 50.0);
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
        double object_yaw{0.0};
    };

    struct TargetPoint
    {
        double x{0.0};
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
                plan_tof_approach_point())
            {
                auto_phase_ = AutoPhase::TOF_APPROACH;
                phase_start_time_sec_ = this->now().seconds();
            }
            break;
        }

        case AutoPhase::TOF_APPROACH:
        {
            if (!has_tof_approach_point_)
            {
                if (!plan_tof_approach_point())
                {
                    target_valid_ = false;
                    break;
                }
            }

            if (!solve_goal_from_point(tof_approach_point_, locked_object_yaw_))
            {
                target_valid_ = false;
                break;
            }

            goal_[5] = gripper_open_pos_;
            command_time_from_start_sec_ = 1.0;
            if (tof_target_inside_gripper() && arm_goal_reached(approach_reach_tol_rad_))
            {
                auto_phase_ = AutoPhase::GRIPPER_CLOSE;
                phase_start_time_sec_ = this->now().seconds();
            }
            else if (arm_goal_reached(approach_reach_tol_rad_))
            {
                extend_tof_approach_point();
            }
            break;
        }

        case AutoPhase::GRIPPER_CLOSE:
        {
            if (!solve_goal_from_point(tof_approach_point_, locked_object_yaw_))
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
            if (!solve_goal_from_point(tof_approach_point_, locked_object_yaw_))
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
            target_lock_buffer_.push_back({target_x, target_y, target_z, msg.object_yaw});
            while (static_cast<int>(target_lock_buffer_.size()) > target_lock_min_samples_)
            {
                target_lock_buffer_.pop_front();
            }
            return;
        }

        if (auto_phase_ == AutoPhase::TRACK_PRE_GRASP ||
            auto_phase_ == AutoPhase::PRE_GRASP_HOLD)
        {
            update_tracked_target({target_x, target_y, target_z, msg.object_yaw},
                                  auto_phase_ == AutoPhase::PRE_GRASP_HOLD);
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

        // System convention: +pitch means looking downward.
        // Convert to math-up convention before spherical -> Cartesian conversion.
        const double pitch_up = -msg.pitch;
        const double cp = std::cos(pitch_up);
        const double range_use = std::max(min_effective_range_, msg.range);
        const double ux_sensor = cp * std::cos(msg.yaw);
        const double uy_sensor = cp * std::sin(msg.yaw);
        const double uz_sensor = std::sin(pitch_up);
        const geometry_msgs::msg::PointStamped p_sensor = [&]()
        {
            geometry_msgs::msg::PointStamped p;
            p.header = msg.header;
            p.point.x = range_use * ux_sensor;
            p.point.y = range_use * uy_sensor;
            p.point.z = range_use * uz_sensor;
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
            tf2::Vector3 u_base = R * tf2::Vector3(ux_sensor, uy_sensor, uz_sensor);
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

        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = this->now();
        traj.joint_names = joint_name_;

        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = goal_;
        p.time_from_start = rclcpp::Duration::from_seconds(command_time_from_start_sec_);

        traj.points.push_back(p);
        pub_arm_traj_->publish(traj);
    }

    void reset_auto_sequence()
    {
        target_lock_buffer_.clear();
        has_locked_object_point_ = false;
        has_commanded_goal_point_ = false;
        has_frozen_pre_grasp_point_ = false;
        has_tracked_pre_grasp_candidate_ = false;
        has_frozen_pre_grasp_joints_ = false;
        has_frozen_approach_axis_ = false;
        has_tof_approach_point_ = false;
        grasp_start_requested_ = false;
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
        for (const auto &p : target_lock_buffer_)
        {
            m.x += p.x;
            m.y += p.y;
            m.z += p.z;
            yaw_c += std::cos(p.object_yaw);
            yaw_s += std::sin(p.object_yaw);
        }

        const double n = static_cast<double>(target_lock_buffer_.size());
        m.x /= n;
        m.y /= n;
        m.z /= n;
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

        locked_object_point_.x = mean.x;
        locked_object_point_.y = mean.y;
        locked_object_point_.z = mean.z;
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
            locked_object_yaw_ = sample.object_yaw;
            has_locked_object_point_ = true;
        }
        else
        {
            locked_object_point_.x += alpha * (sample.x - locked_object_point_.x);
            locked_object_point_.y += alpha * (sample.y - locked_object_point_.y);
            locked_object_point_.z += alpha * (sample.z - locked_object_point_.z);
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
        double vx = object_point.x;
        double vy = object_point.y;
        const double nxy = std::hypot(vx, vy);
        if (nxy > 1e-6)
        {
            vx /= nxy;
            vy /= nxy;
            last_valid_vx_ = vx;
            last_valid_vy_ = vy;
        }
        else
        {
            const double last_n = std::hypot(last_valid_vx_, last_valid_vy_);
            if (last_n <= 1e-6)
            {
                RCLCPP_WARN(this->get_logger(), "plan_locked_points: horizontal approach axis is invalid.");
                return out;
            }
            vx = last_valid_vx_;
            vy = last_valid_vy_;
        }

        const double left_x = -vy;
        const double left_y = vx;

        out.x = object_point.x - pre_grasp_distance_m_ * vx + pre_grasp_lateral_offset_m_ * left_x;
        out.y = object_point.y - pre_grasp_distance_m_ * vy + pre_grasp_lateral_offset_m_ * left_y;
        out.z = object_point.z + pre_grasp_lift_z_m_;

        return out;
    }

    bool solve_goal_from_point(const TargetPoint &p, double wrist_roll_ref)
    {
        const double x = p.x;
        const double y = p.y;
        const double z = p.z;

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

        const double shoulder_pitch_up = nearest_equivalent_angle(th1_up, prev_shoulder_pitch);
        const double elbow_up = nearest_equivalent_angle(th2_up, prev_elbow);
        const double shoulder_pitch_dn = nearest_equivalent_angle(th1_dn, prev_shoulder_pitch);
        const double elbow_dn = nearest_equivalent_angle(th2_dn, prev_elbow);

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

            return value;
        };

        double shoulder_pitch_sel = 0.0;
        double elbow_sel = 0.0;
        if (ok_up && !ok_dn)
        {
            shoulder_pitch_sel = shoulder_pitch_up;
            elbow_sel = elbow_up;
        }
        else if (!ok_up && ok_dn)
        {
            shoulder_pitch_sel = shoulder_pitch_dn;
            elbow_sel = elbow_dn;
        }
        else
        {
            if (cost(shoulder_pitch_up, elbow_up) <= cost(shoulder_pitch_dn, elbow_dn))
            {
                shoulder_pitch_sel = shoulder_pitch_up;
                elbow_sel = elbow_up;
            }
            else
            {
                shoulder_pitch_sel = shoulder_pitch_dn;
                elbow_sel = elbow_dn;
            }
        }

        goal_[0] = psi;
        goal_[1] = shoulder_pitch_sel;
        goal_[2] = elbow_sel;

        const double wrist_pitch_raw = -(goal_[1] + goal_[2]) + wrist_pitch_level_bias_;
        goal_[3] = nearest_equivalent_angle(wrist_pitch_raw, q_meas_[3]);
        goal_[4] = nearest_equivalent_angle(wrist_roll_ref, q_meas_[4]);

        target_valid_ = true;
        commanded_goal_point_ = p;
        has_commanded_goal_point_ = true;
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

    void freeze_pre_grasp_point()
    {
        frozen_pre_grasp_point_ = pre_grasp_point_;
        has_frozen_pre_grasp_point_ = true;
        tracked_pre_grasp_candidate_ = pre_grasp_point_;
        has_tracked_pre_grasp_candidate_ = true;
        frozen_pre_grasp_joint_positions_ = q_meas_;
        has_frozen_pre_grasp_joints_ = frozen_pre_grasp_joint_positions_.size() > 2;

        const double n = std::hypot(last_valid_vx_, last_valid_vy_);
        if (n > 1e-6)
        {
            frozen_approach_axis_x_ = last_valid_vx_ / n;
            frozen_approach_axis_y_ = last_valid_vy_ / n;
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

    bool plan_tof_approach_point()
    {
        if (!has_frozen_pre_grasp_point_ ||
            !has_frozen_approach_axis_ ||
            !tof_range_valid())
        {
            return false;
        }

        tof_forward_distance_m_ = clamp(
            latest_tof_range_m_ - desired_grasp_range_m_ + gripper_inner_extra_approach_m_,
            min_forward_approach_m_,
            max_forward_approach_m_);

        update_tof_approach_point_from_forward_distance();
        has_tof_approach_point_ = true;
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
        return true;
    }

    void update_tof_approach_point_from_forward_distance()
    {
        tof_approach_point_.x =
            frozen_pre_grasp_point_.x + tof_forward_distance_m_ * frozen_approach_axis_x_;
        tof_approach_point_.y =
            frozen_pre_grasp_point_.y + tof_forward_distance_m_ * frozen_approach_axis_y_;
        tof_approach_point_.z = frozen_pre_grasp_point_.z;
    }

    bool tof_target_inside_gripper() const
    {
        return tof_range_valid() &&
               latest_tof_range_m_ <= gripper_inside_range_m_;
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
        geometry_msgs::msg::Point current_arm_point;
        const bool has_current = lookup_current_arm_point(current_arm_point);
        const double cart_err = has_current
                                    ? cartesian_error_to_goal(current_arm_point)
                                    : std::numeric_limits<double>::quiet_NaN();
        const double hold_drift = current_hold_target_drift();

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3)
           << "phase=" << phase_name()
           << " auto=" << (auto_enabled_ ? 1 : 0)
           << " frozen=" << (has_frozen_pre_grasp_point_ ? 1 : 0)
           << " grasp_req=" << (grasp_start_requested_ ? 1 : 0)
           << " tof_ok=" << (tof_range_valid() ? 1 : 0)
           << " inside=" << (tof_target_inside_gripper() ? 1 : 0)
           << " tof=" << latest_tof_range_m_
           << " fwd=" << tof_forward_distance_m_
           << " d_sh=" << shoulder_pitch_delta_from_hold()
           << " d_el=" << elbow_delta_from_hold()
           << " sh_el=" << shoulder_to_elbow_delta_ratio()
           << " joint_err=" << arm_goal_error_sum()
           << " cart_err=" << cart_err
           << " drift=" << hold_drift;

        if (has_commanded_goal_point_)
        {
            ss << " goal=("
               << commanded_goal_point_.x << ","
               << commanded_goal_point_.y << ","
               << commanded_goal_point_.z << ")";
        }

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
            if (name == "L1")
            {
                L1_ = p.as_double();
            }
            else if (name == "L2")
            {
                L2_ = p.as_double();
            }
            else if (name == "saturate_reach")
            {
                saturate_reach_ = p.as_bool();
            }
            else if (name == "arm_base_frame")
            {
                arm_base_frame_ = p.as_string();
            }
            else if (name == "end_effector_frame")
            {
                end_effector_frame_ = p.as_string();
            }
            else if (name == "min_effective_range")
            {
                min_effective_range_ = p.as_double();
            }
            else if (name == "wrist_pitch_level_bias")
            {
                wrist_pitch_level_bias_ = p.as_double();
            }
            else if (name == "pre_grasp_distance_m")
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
            else if (name == "target_lock_min_samples")
            {
                target_lock_min_samples_ = p.as_int();
            }
            else if (name == "target_lock_pos_tol_m")
            {
                target_lock_pos_tol_m_ = p.as_double();
            }
            else if (name == "hold_time_sec")
            {
                hold_time_sec_ = p.as_double();
            }
            else if (name == "joint_reach_tol_rad")
            {
                joint_reach_tol_rad_ = p.as_double();
            }
            else if (name == "hold_exit_tol_rad")
            {
                hold_exit_tol_rad_ = p.as_double();
            }
            else if (name == "target_filter_alpha")
            {
                target_filter_alpha_ = p.as_double();
            }
            else if (name == "freeze_cartesian_tol_m")
            {
                freeze_cartesian_tol_m_ = p.as_double();
            }
            else if (name == "hold_target_exit_tol_m")
            {
                hold_target_exit_tol_m_ = p.as_double();
            }
            else if (name == "desired_grasp_range_m")
            {
                desired_grasp_range_m_ = p.as_double();
            }
            else if (name == "gripper_inside_range_m")
            {
                gripper_inside_range_m_ = p.as_double();
            }
            else if (name == "gripper_inner_extra_approach_m")
            {
                gripper_inner_extra_approach_m_ = p.as_double();
            }
            else if (name == "tof_approach_step_m")
            {
                tof_approach_step_m_ = p.as_double();
            }
            else if (name == "min_forward_approach_m")
            {
                min_forward_approach_m_ = p.as_double();
            }
            else if (name == "max_forward_approach_m")
            {
                max_forward_approach_m_ = p.as_double();
            }
            else if (name == "tof_timeout_sec")
            {
                tof_timeout_sec_ = p.as_double();
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
            else if (name == "approach_reach_tol_rad")
            {
                approach_reach_tol_rad_ = p.as_double();
            }
            else if (name == "tof_ik_elbow_only_penalty")
            {
                tof_ik_elbow_only_penalty_ = p.as_double();
            }
            else if (name == "tof_ik_shoulder_bonus")
            {
                tof_ik_shoulder_bonus_ = p.as_double();
            }
            else if (name == "tof_ik_shoulder_balance_ratio")
            {
                tof_ik_shoulder_balance_ratio_ = p.as_double();
            }
            else if (name == "pub_hz")
            {
                // runtime change not applied to existing timer period in this implementation
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
    std::string end_effector_frame_{"gimbal_mount_link"};
    double min_effective_range_{0.0};

    double shoulder_rot_x_{0.0};
    double shoulder_rot_y_{-0.0452};
    double shoulder_rot_z_{0.0165};

    double shoulder_pitch_offset_y_{0.1025};
    double shoulder_pitch_offset_z_{0.0306};

    double wrist_pitch_level_bias_{-1.5};

    double pre_grasp_distance_m_{0.08};
    double pre_grasp_lift_z_m_{0.01};
    double pre_grasp_lateral_offset_m_{0.0};

    int target_lock_min_samples_{8};
    double target_lock_pos_tol_m_{0.01};
    double hold_time_sec_{0.4};
    double joint_reach_tol_rad_{0.25};
    double hold_exit_tol_rad_{0.35};
    double target_filter_alpha_{0.25};
    double freeze_cartesian_tol_m_{0.5};
    double hold_target_exit_tol_m_{0.02};
    double desired_grasp_range_m_{0.03};
    double gripper_inside_range_m_{0.035};
    double gripper_inner_extra_approach_m_{0.02};
    double tof_approach_step_m_{0.01};
    double min_forward_approach_m_{0.0};
    double max_forward_approach_m_{0.10};
    double tof_timeout_sec_{0.5};
    double gripper_open_pos_{0.5};
    double gripper_close_pos_{0.0};
    double gripper_action_time_sec_{0.4};
    double approach_reach_tol_rad_{0.25};
    double tof_ik_elbow_only_penalty_{8.0};
    double tof_ik_shoulder_bonus_{1.0};
    double tof_ik_shoulder_balance_ratio_{0.8};

    AutoPhase auto_phase_{AutoPhase::IDLE};
    std::deque<TargetSample> target_lock_buffer_;
    TargetPoint locked_object_point_{};
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
    double frozen_approach_axis_x_{0.0};
    double frozen_approach_axis_y_{0.0};
    double locked_object_yaw_{0.0};
    double hold_start_time_sec_{-1.0};
    double phase_start_time_sec_{-1.0};
    bool hold_reached_logged_{false};
    bool grasp_start_requested_{false};
    bool has_tof_range_{false};
    double latest_tof_range_m_{std::numeric_limits<double>::quiet_NaN()};
    double last_tof_time_sec_{-1.0};
    double tof_forward_distance_m_{0.0};
    double last_valid_vx_{0.0};
    double last_valid_vy_{-1.0};

    std::vector<std::string> joint_name_;
    std::vector<double> goal_;
    std::vector<double> home_goal_;
    std::vector<double> q_meas_{std::vector<double>(6, 0.0)};
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
