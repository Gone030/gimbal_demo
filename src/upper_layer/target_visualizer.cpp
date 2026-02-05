#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "target_calculator.hpp"

using std::placeholders::_1;

class TargetVisualizer : public rclcpp::Node{
    public:
        TargetVisualizer() : Node("target_visualizer"){

            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/gimbal/tof_distance", rclcpp::SensorDataQoS(), std::bind(&TargetVisualizer::scan_callback, this, _1)
            );

            joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&TargetVisualizer::joint_callback, this, _1)
            );

            marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
                "target_marker", 10
            );

            RCLCPP_INFO(this->get_logger(), "Target Visualizing");
        }

    private:
        void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
            for(size_t i = 0; i < msg->name.size(); i++){
                if(msg->name[i] == "yaw_joint") current_yaw_ = msg->position[i];
                else if(msg->name[i] == "pitch_joint") current_pitch_ = msg->position[i];
            }
        }

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
            float dist = msg->ranges[0];

            if(std::isinf(dist) || dist < 0.05) return;

            Point3D target = calc_.calculate(dist, current_yaw_, current_pitch_);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "base_link"; // 짐벌의 뿌리가 되는 링크 이름 (필수 확인!)
            marker.header.stamp = this->now();
            marker.ns = "gimbal_target";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE; // 구체 모양
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position.x = target.x;
            marker.pose.position.y = target.y;
            marker.pose.position.z = target.z;

            marker.scale.x = 0.1; // 지름 10cm
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0; // 투명도
            marker.color.r = 1.0; // 빨간색
            marker.color.g = 0.0;
            marker.color.b = 0.0;

            marker_pub_->publish(marker);

        }


        float current_yaw_ = 0.0f;
        float current_pitch_ = 0.0f;
        TargetCalculator calc_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
