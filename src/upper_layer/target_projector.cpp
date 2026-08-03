#include "gimbal_mani/arm_upper/target_projector.hpp"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gimbal_mani::arm_upper
{

TargetProjector::TargetProjector(rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
    : logger_(logger),
      clock_(clock)
{
}

std::optional<TargetProjection> TargetProjector::project(
    const gimbal_mani::msg::TargetBearingRange &msg,
    const tf2_ros::Buffer &tf_buffer,
    const std::string &arm_base_frame,
    double min_effective_range) const
{
    if (msg.header.frame_id.empty())
    {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                             "Dropping target: header.frame_id is empty.");
        return std::nullopt;
    }

    const double range_use = std::max(min_effective_range, msg.range);
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
    TargetVector approach_axis;
    try
    {
        const rclcpp::Time tf_time(msg.header.stamp);
        const auto tf = tf_buffer.lookupTransform(
            arm_base_frame, msg.header.frame_id, tf_time, rclcpp::Duration::from_seconds(0.03));
        tf2::doTransform(p_sensor, p_base, tf);

        tf2::Quaternion q;
        tf2::fromMsg(tf.transform.rotation, q);
        tf2::Matrix3x3 R(q);
        tf2::Vector3 u_base = R * tf2::Vector3(1.0, 0.0, 0.0);
        const double n = u_base.length();
        if (n <= 1e-9 || !std::isfinite(n))
        {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                                 "Invalid ray direction norm after TF rotation.");
            return std::nullopt;
        }
        approach_axis.x = u_base.x() / n;
        approach_axis.y = u_base.y() / n;
        approach_axis.z = u_base.z() / n;

        // RCLCPP_INFO_THROTTLE(
        //     logger_, *clock_, 500,
        //     "target_tf: frame=%s->%s yaw=%.3f pitch=%.3f range=%.3f range_use=%.3f | "
        //     "p_sensor=(%.3f,%.3f,%.3f) p_base=(%.3f,%.3f,%.3f) u_base=(%.3f,%.3f,%.3f)",
        //     msg.header.frame_id.c_str(), arm_base_frame.c_str(),
        //     msg.yaw, msg.pitch, msg.range, range_use,
        //     p_sensor.point.x, p_sensor.point.y, p_sensor.point.z,
        //     p_base.point.x, p_base.point.y, p_base.point.z,
        //     approach_axis.x, approach_axis.y, approach_axis.z);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                             "TF failed (%s -> %s): %s",
                             msg.header.frame_id.c_str(), arm_base_frame.c_str(), ex.what());
        return std::nullopt;
    }

    TargetProjection projection;
    projection.sensor_frame = msg.header.frame_id;
    projection.range_use = range_use;
    projection.sample = {
        p_base.point.x,
        p_base.point.y,
        p_base.point.z,
        approach_axis.x,
        approach_axis.y,
        approach_axis.z,
        msg.yaw,
        msg.pitch,
        msg.object_yaw};
    return projection;
}

}  // namespace gimbal_mani::arm_upper
