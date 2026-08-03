#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace gimbal_mani::arm_upper
{

class TrajectoryBuilder
{
public:
    TrajectoryBuilder() = default;
    explicit TrajectoryBuilder(std::vector<std::string> joint_names);

    void setJointNames(const std::vector<std::string> &joint_names);
    trajectory_msgs::msg::JointTrajectory build(
        const std::vector<double> &goal,
        const rclcpp::Time &stamp,
        double time_from_start_sec) const;

private:
    std::vector<std::string> joint_names_;
};

}  // namespace gimbal_mani::arm_upper
