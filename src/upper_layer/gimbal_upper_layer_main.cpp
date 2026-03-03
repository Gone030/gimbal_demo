#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "gimbal_mani/msg/target_bearing_range.hpp"
#include "gimbal_mani/msg/gimbal_manual_cmd.hpp"

class GimbalUpperLayerMain : public rclcpp::Node
{
public:
    GimbalUpperLayerMain() : Node("gimbal_upper_layer_main")
    {
        traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_in/gimbal", 10);

        tracker_cmd_sub_ = create_subscription<std_msgs::msg::String>(
            "/gimbal/tracker", 10,
            [this](const std_msgs::msg::String &msg)
            {
                tracker_cmd_ = msg.data; // "NONE" / "RED" / "GREEN" / "BLUE"
            });

        manual_cmd_sub_ = create_subscription<gimbal_mani::msg::GimbalManualCmd>(
            "/gimbal/manual", 10,
            [this](const gimbal_mani::msg::GimbalManualCmd &msg)
            {
                manual_pitch_ = msg.pitch;
                manual_yaw_ = msg.yaw;
            });

        tbr_sub_ = create_subscription<gimbal_mani::msg::TargetBearingRange>(
            "/target", 10,
            [this](const gimbal_mani::msg::TargetBearingRange &msg)
            {
                vision_dyaw_ = msg.yaw;
                vision_dpitch_ = msg.pitch;
                has_new_vision_delta_ = true;
            });

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            [this]()
            { main_loop(); });
    }

private:
    bool is_manual_mode() const
    {
        return tracker_cmd_ == "NONE";
    }

    void main_loop()
    {
        const bool manual = is_manual_mode();

        if (manual)
        {
            commanded_yaw_ = manual_yaw_;
            commanded_pitch_ = manual_pitch_;
            has_new_vision_delta_ = false;
        }
        else
        {
            if (has_new_vision_delta_)
            {
                commanded_yaw_ += vision_dyaw_;
                commanded_pitch_ += vision_dpitch_;
                has_new_vision_delta_ = false;
            }
        }
        publish_traj(commanded_yaw_, commanded_pitch_);
    }

    void publish_traj(double yaw, double pitch)
    {
        trajectory_msgs::msg::JointTrajectory traj;

        traj.joint_names = {
            "gimbal_yaw_servo_joint",
            "gimbal_pitch_servo_joint"};

        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = {yaw, pitch};
        p.time_from_start.sec = 0;
        p.time_from_start.nanosec = 100000000; // temp

        traj.points.push_back(p);
        traj_pub_->publish(traj);
    }

    std::string tracker_cmd_ = "NONE";
    double vision_dyaw_ = 0.0;
    double vision_dpitch_ = 0.0;
    bool has_new_vision_delta_ = false;

    double manual_yaw_ = 0.0;
    double manual_pitch_ = 0.0;
    double commanded_yaw_ = 0.0;
    double commanded_pitch_ = 0.0;

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tracker_cmd_sub_;
    rclcpp::Subscription<gimbal_mani::msg::GimbalManualCmd>::SharedPtr manual_cmd_sub_;
    rclcpp::Subscription<gimbal_mani::msg::TargetBearingRange>::SharedPtr tbr_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalUpperLayerMain>());
    rclcpp::shutdown();
    return 0;
}
