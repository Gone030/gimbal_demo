#pragma once

#include <string>

#include <tf2_ros/buffer.h>

#include "gimbal_mani/arm_upper/arm_types.hpp"

namespace gimbal_mani::arm_upper
{

class ArmTfLookup
{
public:
    bool lookupCurrentArmPoint(
        tf2_ros::Buffer &tf_buffer,
        const std::string &arm_base_frame,
        const std::string &end_effector_frame,
        Vec3 &out_point) const;
};

}  // namespace gimbal_mani::arm_upper
