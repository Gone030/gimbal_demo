#include "gimbal_mani/arm_upper/trajectory_builder.hpp"

#include <utility>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace gimbal_mani::arm_upper
{

TrajectoryBuilder::TrajectoryBuilder(std::vector<std::string> joint_names)
    : joint_names_(std::move(joint_names))
{
}

void TrajectoryBuilder::setJointNames(const std::vector<std::string> &joint_names)
{
    joint_names_ = joint_names;
}

trajectory_msgs::msg::JointTrajectory TrajectoryBuilder::build(
    const std::vector<double> &goal,
    const rclcpp::Time &stamp,
    double time_from_start_sec) const
{
    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = stamp;
    traj.joint_names = joint_names_;

    trajectory_msgs::msg::JointTrajectoryPoint p;
    p.positions = goal;
    p.time_from_start = rclcpp::Duration::from_seconds(time_from_start_sec);

    traj.points.push_back(p);
    return traj;
}

}  // namespace gimbal_mani::arm_upper
