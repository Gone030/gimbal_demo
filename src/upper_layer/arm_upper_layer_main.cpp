#include "gimbal_mani/arm_upper/arm_upper_layer_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<gimbal_mani::arm_upper::ArmUpperLayerNode>());
    rclcpp::shutdown();
    return 0;
}
