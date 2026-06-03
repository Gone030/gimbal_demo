#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class ArmJointTestNode final : public rclcpp::Node
{
public:
    ArmJointTestNode() : Node("arm_joint_test_node")
    {
        joint_names_ = {
            "Shoulder_Rotation",
            "Shoulder_Pitch",
            "Elbow",
            "Wrist_Pitch",
            "Wrist_Roll",
            "Gripper"};

        goal_ = {
            deg_to_rad(declare_parameter<double>("initial_shoulder_rotation_deg", 0.0)),
            deg_to_rad(declare_parameter<double>("initial_shoulder_pitch_deg", 0.0)),
            deg_to_rad(declare_parameter<double>("initial_elbow_deg", 90.0)),
            deg_to_rad(declare_parameter<double>("initial_wrist_pitch_deg", 0.0)),
            deg_to_rad(declare_parameter<double>("initial_wrist_roll_deg", 0.0)),
            declare_parameter<double>("initial_gripper", 0.5)};

        command_time_sec_ = declare_parameter<double>("command_time_sec", 1.0);

        pub_traj_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_in/arm", 10);

        sub_joint_state_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&ArmJointTestNode::on_joint_state, this, std::placeholders::_1));

        sub_shoulder_rotation_ = subscribe_deg("/arm/joint_test/shoulder_rotation_deg", 0);
        sub_shoulder_pitch_ = subscribe_deg("/arm/joint_test/shoulder_pitch_deg", 1);
        sub_elbow_ = subscribe_deg("/arm/joint_test/elbow_deg", 2);
        sub_wrist_pitch_ = subscribe_deg("/arm/joint_test/wrist_pitch_deg", 3);
        sub_wrist_roll_ = subscribe_deg("/arm/joint_test/wrist_roll_deg", 4);

        sub_gripper_ = create_subscription<std_msgs::msg::Float64>(
            "/arm/joint_test/gripper", 10,
            [this](const std_msgs::msg::Float64 &msg)
            {
                set_joint(5, msg.data, false);
            });

        RCLCPP_INFO(
            get_logger(),
            "arm_joint_test_node ready. Degree topics: "
            "/arm/joint_test/{shoulder_rotation_deg,shoulder_pitch_deg,elbow_deg,wrist_pitch_deg,wrist_roll_deg}; "
            "gripper raw topic: /arm/joint_test/gripper");
    }

private:
    static double deg_to_rad(double deg)
    {
        return deg * M_PI / 180.0;
    }

    static double rad_to_deg(double rad)
    {
        return rad * 180.0 / M_PI;
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscribe_deg(
        const std::string &topic, size_t index)
    {
        return create_subscription<std_msgs::msg::Float64>(
            topic, 10,
            [this, index](const std_msgs::msg::Float64 &msg)
            {
                set_joint(index, deg_to_rad(msg.data), true);
            });
    }

    void on_joint_state(const sensor_msgs::msg::JointState &msg)
    {
        if (msg.name.size() != msg.position.size())
        {
            return;
        }

        for (size_t goal_i = 0; goal_i < joint_names_.size(); ++goal_i)
        {
            for (size_t msg_i = 0; msg_i < msg.name.size(); ++msg_i)
            {
                if (msg.name[msg_i] == joint_names_[goal_i])
                {
                    goal_[goal_i] = msg.position[msg_i];
                    break;
                }
            }
        }
    }

    void set_joint(size_t index, double value, bool report_degrees)
    {
        if (index >= goal_.size())
        {
            return;
        }

        goal_[index] = value;
        if (report_degrees)
        {
            RCLCPP_INFO(get_logger(), "%s command %.2f deg accepted.", joint_names_[index].c_str(), rad_to_deg(value));
        }
        else
        {
            RCLCPP_INFO(get_logger(), "%s command %.3f accepted.", joint_names_[index].c_str(), value);
        }
        publish_goal();
    }

    void publish_goal()
    {
        trajectory_msgs::msg::JointTrajectory traj;
        traj.header.stamp = now();
        traj.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = goal_;
        point.time_from_start = rclcpp::Duration::from_seconds(command_time_sec_);
        traj.points.push_back(point);

        pub_traj_->publish(traj);
    }

    std::vector<std::string> joint_names_;
    std::vector<double> goal_;
    double command_time_sec_{1.0};

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_traj_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_shoulder_rotation_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_shoulder_pitch_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_elbow_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_wrist_pitch_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_wrist_roll_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_gripper_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmJointTestNode>());
    rclcpp::shutdown();
    return 0;
}
