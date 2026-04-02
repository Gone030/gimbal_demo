#include <cmath>
#include <string>
#include <vector>
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
        L1_ = this->declare_parameter<double>("L1", 0.116); // 실제 길이에 맞게 수정해야함
        L2_ = this->declare_parameter<double>("L2", 0.135);

        saturate_reach_ = this->declare_parameter<bool>("saturate_reach", true);
        arm_base_frame_ = this->declare_parameter<std::string>("arm_base_frame", "Base");
        target_single_shot_mode_ = this->declare_parameter<bool>("target_single_shot_mode", true);
        wrist_pitch_level_bias_ = this->declare_parameter<double>("wrist_pitch_level_bias", -1.5);

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
    void on_timer()
    {
        if (recompute_requested_ && auto_enabled_ && has_last_target_)
        {
            recompute_requested_ = false;
            on_target(last_target_msg_);
        }

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

        if (single_shot_command_)
        {
            single_shot_command_ = false;
            target_valid_ = false;
        }
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
        has_last_target_ = true;
        last_target_msg_ = msg;

        if (!std::isfinite(msg.yaw) || !std::isfinite(msg.pitch) || !std::isfinite(msg.range) || msg.range <= 0.0)
        {
            target_valid_ = false;
            return;
        }

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!target_to_arm_base_point(msg, x, y, z))
        {
            target_valid_ = false;
            return;
        }

        const double x_sr = x - shoulder_rot_x_;
        const double y_sr = y - shoulder_rot_y_;
        const double z_sr = z - shoulder_rot_z_;

        // Shoulder_Rotation is not msg.yaw.
        // It must be computed from the transformed target point around the real yaw axis.
        // Frame convention fix: in this model, forward on yaw plane maps to -y.
        const double psi_raw = std::atan2(x_sr, -y_sr);
        const double psi = nearest_equivalent_angle(psi_raw, q_meas_[0]);

        const double c = std::cos(psi_raw);
        const double s = std::sin(psi_raw);

        // target in Shoulder_Rotation_Pitch frame (after desired yaw alignment)
        const double x_plane = c * x_sr + s * y_sr;   // should be near 0
        const double y_plane = z_sr;
        const double z_plane = s * x_sr - c * y_sr;

        // move origin from Shoulder_Rotation pivot to Shoulder_Pitch pivot
        double rho = y_plane - shoulder_pitch_offset_y_;
        double z2  = z_plane - shoulder_pitch_offset_z_;
        if (saturate_reach_)
        {
            const double rr = std::sqrt((rho * rho) + (z2 * z2));
            if (rr >= 1e-9)
            {
                const double r_max = (L1_ + L2_);
                const double r_min = std::fabs(L1_ - L2_);
                double rr_sat = rr;
                if (rr > r_max)
                    rr_sat = r_max;
                else if (rr < r_min)
                    rr_sat = r_min;

                if (rr_sat != rr)
                {
                    const double s = rr_sat / rr;
                    rho *= s;
                    z2 *= s;
                }
            }
        }

        double th1_up = 0.0, th2_up = 0.0;
        double th1_dn = 0.0, th2_dn = 0.0;
        const bool ok_up = ik_2link_2d(rho, z2, L1_, L2_, +1, th1_up, th2_up);
        const bool ok_dn = ik_2link_2d(rho, z2, L1_, L2_, -1, th1_dn, th2_dn);

        if (!ok_up && !ok_dn)
        {
            target_valid_ = false;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "IK unreachable: rho=%.4f z=%.4f (L1=%.3f L2=%.3f)", rho, z2, L1_, L2_);
            return;
        }

        const double prev_shoulder_pitch = q_meas_[1];
        const double prev_elbow = q_meas_[2];

        const double shoulder_pitch_up = nearest_equivalent_angle(-th1_up, prev_shoulder_pitch);
        const double elbow_up = nearest_equivalent_angle(th2_up, prev_elbow);
        const double shoulder_pitch_dn = nearest_equivalent_angle(-th1_dn, prev_shoulder_pitch);
        const double elbow_dn = nearest_equivalent_angle(th2_dn, prev_elbow);

        auto cost = [&](double a1, double a2)
        {
            return std::fabs(a1 - prev_shoulder_pitch) + std::fabs(a2 - prev_elbow);
        };

        double shoulder_pitch_sel = 0.0, elbow_sel = 0.0;
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

        // Keep wrist pitch axis roughly horizontal with a fixed bias (URDF zero policy).
        const double wrist_pitch_raw = -(goal_[1] + goal_[2]) + wrist_pitch_level_bias_;
        goal_[3] = nearest_equivalent_angle(wrist_pitch_raw, q_meas_[3]);

        goal_[4] = nearest_equivalent_angle(msg.object_yaw, q_meas_[4]); // temp, 비전, 프레임 정책에 따라 바뀔 수 있음

        command_time_from_start_sec_ = 1.0;
        if (target_single_shot_mode_)
        {
            single_shot_command_ = true;
            auto_enabled_ = false;
            RCLCPP_WARN(this->get_logger(), "Target latched once. Auto tracking disabled.");
        }
        else
        {
            single_shot_command_ = false;
        }
        target_valid_ = true;
    }

    void on_home(const std_msgs::msg::Empty &)
    {
        auto_enabled_ = false;
        goal_ = home_goal_;
        command_time_from_start_sec_ = 2.0;
        single_shot_command_ = true;
        target_valid_ = true;
        RCLCPP_WARN(this->get_logger(), "Arm moving to home pose slowly. Auto tracking disabled.");
    }

    void on_auto_enable(const std_msgs::msg::Bool &msg)
    {
        auto_enabled_ = msg.data;
        RCLCPP_INFO(this->get_logger(), "Arm auto tracking %s.", auto_enabled_ ? "enabled" : "disabled");
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
                                  double &x_out, double &y_out, double &z_out)
    {
        if (msg.header.frame_id.empty())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Dropping target: header.frame_id is empty.");
            return false;
        }

        const double cp = std::cos(msg.pitch);
        const geometry_msgs::msg::PointStamped p_sensor = [&]()
        {
            geometry_msgs::msg::PointStamped p;
            p.header = msg.header;
            p.point.x = msg.range * cp * std::cos(msg.yaw);
            p.point.y = msg.range * cp * std::sin(msg.yaw);
            p.point.z = msg.range * std::sin(msg.pitch);
            return p;
        }();

        geometry_msgs::msg::PointStamped p_base;
        try
        {
            const rclcpp::Time tf_time(msg.header.stamp);
            const auto tf = tf_buffer_->lookupTransform(
                arm_base_frame_, msg.header.frame_id, tf_time, rclcpp::Duration::from_seconds(0.03));
            tf2::doTransform(p_sensor, p_base, tf);
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

    static double nearest_equivalent_angle(double target, double reference)
    {
        // choose target + 2k*pi that is closest to the previous command
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
            else if (name == "target_single_shot_mode")
            {
                target_single_shot_mode_ = p.as_bool();
            }
            else if (name == "wrist_pitch_level_bias")
            {
                wrist_pitch_level_bias_ = p.as_double();
            }
            else if (name == "pub_hz")
            {
                // runtime change not applied to existing timer period in this implementation
            }
        }
        if (result.successful)
        {
            recompute_requested_ = true;
        }
        return result;
    }

private:
    double L1_{0.116}, L2_{0.135};
    bool saturate_reach_{true};
    bool target_valid_{false};
    bool auto_enabled_{true};
    bool single_shot_command_{false};
    double command_time_from_start_sec_{0.1};
    std::string arm_base_frame_{"Base"};

    double shoulder_rot_x_{0.0};
    double shoulder_rot_y_{-0.0452};
    double shoulder_rot_z_{0.0165};

    double shoulder_pitch_offset_y_{0.1025};
    double shoulder_pitch_offset_z_{0.0306};

    bool target_single_shot_mode_{true};
    double wrist_pitch_level_bias_{-1.5};

    std::vector<std::string> joint_name_;
    std::vector<double> goal_;
    std::vector<double> home_goal_;
    std::vector<double> q_meas_{std::vector<double>(6, 0.0)};
    bool has_joint_state_{false};
    gimbal_mani::msg::TargetBearingRange last_target_msg_;
    bool has_last_target_{false};
    bool recompute_requested_{false};

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
