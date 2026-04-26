#include <cmath>
#include <string>
#include <vector>
#include <deque>
#include <chrono>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "gimbal_mani/ik_2link_2d.hpp"
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
        min_effective_range_ = this->declare_parameter<double>("min_effective_range", 0.0);
        wrist_pitch_level_bias_ = this->declare_parameter<double>("wrist_pitch_level_bias", -1.5);

        pre_grasp_distance_m_ =
            this->declare_parameter<double>("pre_grasp_distance_m", 0.08);
        pre_grasp_lift_z_m_ =
            this->declare_parameter<double>("pre_grasp_lift_z_m", 0.01);

        target_lock_min_samples_ = this->declare_parameter<int>("target_lock_min_samples", 8);
        target_lock_pos_tol_m_ = this->declare_parameter<double>("target_lock_pos_tol_m", 0.01);
        hold_time_sec_ = this->declare_parameter<double>("hold_time_sec", 0.4);
        joint_reach_tol_rad_ = this->declare_parameter<double>("joint_reach_tol_rad", 0.25);

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

        sub_target_ = this->create_subscription<gimbal_mani::msg::TargetBearingRange>(
            "/target", 10,
            std::bind(&Arm_UpperLayerMain::on_target, this, std::placeholders::_1));

        sub_home_ = this->create_subscription<std_msgs::msg::Empty>(
            "/arm/home", 10,
            std::bind(&Arm_UpperLayerMain::on_home, this, std::placeholders::_1));

        sub_auto_enable_ = this->create_subscription<std_msgs::msg::Bool>(
            "/arm/auto_enable", 10,
            std::bind(&Arm_UpperLayerMain::on_auto_enable, this, std::placeholders::_1));

        sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&Arm_UpperLayerMain::on_joint_state, this, std::placeholders::_1));

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
        TARGET_LOCK,
        PRE_GRASP_MOVE,
        PRE_GRASP_HOLD,
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
            return;
        }

        switch (auto_phase_)
        {
        case AutoPhase::IDLE:
            break;

        case AutoPhase::TARGET_LOCK:
        {
            if (plan_locked_points())
            {
                if (!solve_goal_from_point(pre_grasp_point_, locked_object_yaw_))
                {
                    target_valid_ = false;
                    break;
                }

                command_time_from_start_sec_ = 1.0;
                auto_phase_ = AutoPhase::PRE_GRASP_MOVE;

                RCLCPP_INFO(this->get_logger(),
                            "TARGET_LOCK done: obj=(%.3f, %.3f, %.3f) pre=(%.3f, %.3f, %.3f)",
                            locked_object_point_.x, locked_object_point_.y, locked_object_point_.z,
                            pre_grasp_point_.x, pre_grasp_point_.y, pre_grasp_point_.z);
            }
            break;
        }

        case AutoPhase::PRE_GRASP_MOVE:
        {
            if (!solve_goal_from_point(pre_grasp_point_, locked_object_yaw_))
            {
                target_valid_ = false;
                break;
            }

            command_time_from_start_sec_ = 1.0;
            if (arm_goal_reached(joint_reach_tol_rad_))
            {
                auto_phase_ = AutoPhase::PRE_GRASP_HOLD;
                hold_start_time_sec_ = this->now().seconds();
            }
            break;
        }

        case AutoPhase::PRE_GRASP_HOLD:
        {
            if (!solve_goal_from_point(pre_grasp_point_, locked_object_yaw_))
            {
                target_valid_ = false;
                break;
            }

            command_time_from_start_sec_ = 1.0;
            if (!arm_goal_reached(joint_reach_tol_rad_))
            {
                auto_phase_ = AutoPhase::PRE_GRASP_MOVE;
                break;
            }

            const double held = this->now().seconds() - hold_start_time_sec_;
            if (held >= hold_time_sec_)
            {
                RCLCPP_INFO(this->get_logger(), "PRE_GRASP_HOLD done.");
                auto_enabled_ = false;
                auto_phase_ = AutoPhase::IDLE;
            }
            break;
        }
        }

        publish_current_goal();
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

        if (auto_phase_ != AutoPhase::TARGET_LOCK)
        {
            return;
        }

        target_lock_buffer_.push_back({target_x, target_y, target_z, msg.object_yaw});
        while (static_cast<int>(target_lock_buffer_.size()) > target_lock_min_samples_)
        {
            target_lock_buffer_.pop_front();
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
            auto_phase_ = AutoPhase::TARGET_LOCK;
            target_valid_ = false;
        }
        else
        {
            auto_phase_ = AutoPhase::IDLE;
            reset_auto_sequence();
            target_valid_ = false;
        }
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

            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 500,
                "target_tf: frame=%s->%s yaw=%.3f pitch=%.3f range=%.3f range_use=%.3f | "
                "p_sensor=(%.3f,%.3f,%.3f) p_base=(%.3f,%.3f,%.3f) u_base=(%.3f,%.3f,%.3f)",
                msg.header.frame_id.c_str(), arm_base_frame_.c_str(),
                msg.yaw, msg.pitch, msg.range, range_use,
                p_sensor.point.x, p_sensor.point.y, p_sensor.point.z,
                p_base.point.x, p_base.point.y, p_base.point.z,
                ux_out, uy_out, uz_out);
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
        hold_start_time_sec_ = -1.0;
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

        double vx = mean.x;
        double vy = mean.y;
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
                return false;
            }
            vx = last_valid_vx_;
            vy = last_valid_vy_;
        }

        pre_grasp_point_.x = mean.x - pre_grasp_distance_m_ * vx;
        pre_grasp_point_.y = mean.y - pre_grasp_distance_m_ * vy;
        pre_grasp_point_.z = mean.z + pre_grasp_lift_z_m_;

        return true;
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

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "solve_goal: p=(%.3f,%.3f,%.3f) rho=%.3f z2=%.3f",
            x, y, z, rho, z2);

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
            return w_sh * std::fabs(a1 - prev_shoulder_pitch) +
                   w_el * std::fabs(a2 - prev_elbow);
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

    static double nearest_equivalent_angle(double target, double reference)
    {
        return reference + wrap_pi(target - reference);
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
            else if (name == "pub_hz")
            {
                // runtime change not applied to existing timer period in this implementation
            }
        }

        if (has_locked_object_point_)
        {
            plan_locked_points();
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
    double min_effective_range_{0.0};

    double shoulder_rot_x_{0.0};
    double shoulder_rot_y_{-0.0452};
    double shoulder_rot_z_{0.0165};

    double shoulder_pitch_offset_y_{0.1025};
    double shoulder_pitch_offset_z_{0.0306};

    double wrist_pitch_level_bias_{-1.5};

    double pre_grasp_distance_m_{0.08};
    double pre_grasp_lift_z_m_{0.01};

    int target_lock_min_samples_{8};
    double target_lock_pos_tol_m_{0.01};
    double hold_time_sec_{0.4};
    double joint_reach_tol_rad_{0.25};

    AutoPhase auto_phase_{AutoPhase::IDLE};
    std::deque<TargetSample> target_lock_buffer_;
    TargetPoint locked_object_point_{};
    TargetPoint pre_grasp_point_{};
    bool has_locked_object_point_{false};
    double locked_object_yaw_{0.0};
    double hold_start_time_sec_{-1.0};
    double last_valid_vx_{0.0};
    double last_valid_vy_{-1.0};

    std::vector<std::string> joint_name_;
    std::vector<double> goal_;
    std::vector<double> home_goal_;
    std::vector<double> q_meas_{std::vector<double>(6, 0.0)};
    bool has_joint_state_{false};

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_traj_;
    rclcpp::Subscription<gimbal_mani::msg::TargetBearingRange>::SharedPtr sub_target_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_home_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_auto_enable_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
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
