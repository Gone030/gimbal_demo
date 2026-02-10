#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "gimbal_mani/msg/target_bearing_range.hpp"
#include "gimbal_mani/msg/gimbal_manual_cmd.hpp"

class GimbalUpperLayerMain : public rclcpp::Node{
    public:
        GimbalUpperLayerMain() : Node("gimbal_upper_layer_main"){
            traj_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/joint_trajectory_in/gimbal", 10);

            tracker_cmd_sub_ = create_subscription<std_msgs::msg::String>(
                "/gimbal/tracker", 10,
                [this](const std_msgs::msg::String &msg){
                    tracker_cmd_ = msg.data; // "NONE" / "RED" / "GREEN" / "BLUE"
                });

            manual_cmd_sub_  = create_subscription<gimbal_mani::msg::GimbalManualCmd>(
                "/gimbal/manual", 10,
                [this](const gimbal_mani::msg::GimbalManualCmd &msg){
                    manual_pitch_ = msg.pitch;
                    manual_yaw_ = msg.yaw;
                });

            tbr_sub_ = create_subscription<gimbal_mani::msg::TargetBearingRange>(
                "/gimbal_manual", 10,
                [this](const gimbal_mani::msg::TargetBearingRange &msg){
                    vision_yaw_ = msg.yaw;
                    vision_pitch_ = msg.pitch;

                });

            timer_ = create_wall_timer(
                std::chrono::milliseconds(50),
                [this]()
                { main_loop(); });
        }

    private:
        bool is_manual_mode() const{
            return tracker_cmd_ == "NONE";
        }

        void main_loop(){
            const bool manual = is_manual_mode();

            double goal_yaw = 0.0;
            double goal_pitch = 0.0;

            if(manual){
                goal_yaw = manual_yaw_;
                goal_pitch = manual_pitch_;
            }else{
                goal_yaw = vision_yaw_;
                goal_pitch = vision_pitch_;
            }
            publish_traj(goal_yaw, goal_pitch);
        }

        void publish_traj(double yaw, double pitch){
            trajectory_msgs::msg::JointTrajectory traj;

            traj.joint_names = {
                "gimbal_yaw_servo_joint",
                "gimbal_pitch_servo_joint"
            };

            trajectory_msgs::msg::JointTrajectoryPoint p;
            p.positions = {yaw, pitch};
            p.time_from_start.sec = 0;
            p.time_from_start.nanosec = 100000000; //temp

            traj.points.push_back(p);
            traj_pub_->publish(traj);
        }

        std::string tracker_cmd_ = "NONE";
        double vision_yaw_ = 0.0;
        double vision_pitch_ = 0.0;


        double manual_yaw_ = 0.0;
        double manual_pitch_ = 0.0;

        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tracker_cmd_sub_;
        rclcpp::Subscription<gimbal_mani::msg::GimbalManualCmd>::SharedPtr manual_cmd_sub_;
        rclcpp::Subscription<gimbal_mani::msg::TargetBearingRange>::SharedPtr tbr_sub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalUpperLayerMain>());
  rclcpp::shutdown();
  return 0;
}
