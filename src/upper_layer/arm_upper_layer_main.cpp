#include <cmath>
#include <string>
#include <vector>
#include <chrono>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "gimbal_mani/ik_2link_2d.hpp"
#include "gimbal_mani/msg/target_bearing_range.hpp"

class Arm_UpperLayerMain : public rclcpp::Node
{
public:
    Arm_UpperLayerMain() : Node("arm_upper_layer_main")
    {
        L1_ = this->declare_parameter<double>("L1", 0.175); // 실제 길이에 맞게 수정해야함
        L2_ = this->declare_parameter<double>("L2", 0.1);

        saturate_reach_ = this->declare_parameter<bool>("saturate_reach", true);

        joint_name_ = {
            "Shoulder_Rotation",
            "Shoulder_Pitch",
            "Elbow",
            "Wrist_Pitch",
            "Wrist_Roll",
            "Gripper"};
        goal_.assign(joint_name_.size(), 0.0);

        pub_arm_traj_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_in/arm", 10);

        sub_target_ = this->create_subscription<gimbal_mani::msg::TargetBearingRange>(
            "/target", 10,
            std::bind(&Arm_UpperLayerMain::on_target, this, std::placeholders::_1));

        const double publish_hz = this->declare_parameter<double>("pub_hz", 50.0);
        const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_hz));
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(period),
            std::bind(&Arm_UpperLayerMain::on_timer, this));
    }

private:
    void on_timer()
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
        p.time_from_start = rclcpp::Duration::from_seconds(0.1);

        traj.points.push_back(p);
        pub_arm_traj_->publish(traj);
    }

    void on_target(const gimbal_mani::msg::TargetBearingRange &msg)
    {
        if (!std::isfinite(msg.yaw) || !std::isfinite(msg.pitch) || !std::isfinite(msg.range) || msg.range <= 0.0)
        {
            target_valid_ = false;
            return;
        }

        const double cy = std::cos(msg.yaw);
        const double sy = std::sin(msg.yaw);
        const double cp = std::cos(msg.pitch);
        const double sp = std::sin(msg.pitch);
        const double r = msg.range;

        const double x = r * cp * cy;
        const double y = r * cp * sy;
        const double z = r * sp;

        // yaw(Sholder_Rotation)
        const double psi = std::atan2(y, x);

        // 2D 평면화 : (rho, z)
        double rho = std::sqrt(x * x + y * y);

        double z2 = z;
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

        const double prev_shoulder_pitch = goal_[1];
        const double prev_elbow = goal_[2];

        auto cost = [&](double a1, double a2)
        {
            return std::fabs(wrap_pi(a1 - prev_shoulder_pitch) + std::fabs(wrap_pi(a2 - prev_elbow)));
        };

        double shoulder_pitch_sel = 0.0, elbow_sel = 0.0;
        if (ok_up && !ok_dn)
        {
            shoulder_pitch_sel = th1_up;
            elbow_sel = th2_up;
        }
        else if (!ok_up && ok_dn)
        {
            shoulder_pitch_sel = th1_dn;
            elbow_sel = th2_dn;
        }
        else
        {
            if (cost(th1_up, th2_up) <= cost(th1_dn, th2_dn))
            {
                shoulder_pitch_sel = th1_up;
                elbow_sel = th2_up;
            }
            else
            {
                shoulder_pitch_sel = th1_dn;
                elbow_sel = th2_dn;
            }
        }

        goal_[0] = wrap_pi(psi);
        goal_[1] = shoulder_pitch_sel;
        goal_[2] = elbow_sel;

        goal_[3] = wrap_pi(-(goal_[1] + goal_[2]));

        goal_[4] = wrap_pi(msg.object_yaw); // temp, 비전, 프레임 정책에 따라 바뀔 수 있음

        target_valid_ = true;
    }

private:
    double L1_{0.175}, L2_{0.1};
    bool saturate_reach_{true};
    bool target_valid_{false};

    std::vector<std::string> joint_name_;
    std::vector<double> goal_;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_arm_traj_;
    rclcpp::Subscription<gimbal_mani::msg::TargetBearingRange>::SharedPtr sub_target_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Arm_UpperLayerMain>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
