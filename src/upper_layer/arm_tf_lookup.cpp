#include "gimbal_mani/arm_upper/arm_tf_lookup.hpp"

#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace gimbal_mani::arm_upper
{

bool ArmTfLookup::lookupCurrentArmPoint(
    tf2_ros::Buffer &tf_buffer,
    const std::string &arm_base_frame,
    const std::string &end_effector_frame,
    Vec3 &out_point) const
{
    try
    {
        const auto tf = tf_buffer.lookupTransform(
            arm_base_frame, end_effector_frame, tf2::TimePointZero);
        out_point.x = tf.transform.translation.x;
        out_point.y = tf.transform.translation.y;
        out_point.z = tf.transform.translation.z;
        return true;
    }
    catch (const tf2::TransformException &)
    {
        return false;
    }
}

}  // namespace gimbal_mani::arm_upper
