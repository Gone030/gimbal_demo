#pragma once

#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "gimbal_mani/arm_upper/arm_types.hpp"
#include "gimbal_mani/msg/target_bearing_range.hpp"

namespace gimbal_mani::arm_upper
{

struct TargetProjection
{
    TargetSample sample;
    std::string sensor_frame;
    double range_use{0.0};
};

class TargetProjector
{
public:
    TargetProjector(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

    std::optional<TargetProjection> project(
        const gimbal_mani::msg::TargetBearingRange &msg,
        const tf2_ros::Buffer &tf_buffer,
        const std::string &arm_base_frame,
        double min_effective_range) const;

private:
    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
};

}  // namespace gimbal_mani::arm_upper
