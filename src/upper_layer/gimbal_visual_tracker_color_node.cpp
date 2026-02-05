#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class VisualTracker : public rclcpp::Node{
    public:
        VisualTracker() : Node("visual_tracker"){
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/gimbal/gimbal_camera/image_raw",
                rclcpp::SensorDataQoS(),
                std::bind(&VisualTracker::image_callback, this, _1)
            );

            joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10,
                std::bind(&VisualTracker::joint_callback, this, _1)
            );

            command_sub_ = this->create_subscription<std_msgs::msg::String>(
                "/tracker/command", 10,
                [this](const std_msgs::msg::String::SharedPtr msg){
                    target_color_ = msg->data;
                }
            );

            traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/gimbal_controller/joint_trajectory", 10
            );

            debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/tracker/debug_image", 10
            );

            current_yaw_ = 0.0;
            current_pitch_ = 0.0;

            target_color_ = "NONE";

            RCLCPP_INFO(this->get_logger(), "Visual tracking activating.");
        }

    private:
        void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                // msg->name과 position을 매칭해서 내 관절 값 찾기
                for (size_t i = 0; i < msg->name.size(); ++i)
                {
                if (msg->name[i] == "yaw_joint") current_yaw_ = msg->position[i];
                else if (msg->name[i] == "pitch_joint") current_pitch_ = msg->position[i];
                }
            }
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg){
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }catch(cv_bridge::Exception& e){
                RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
                return;
            }

            cv::Mat hsv_img, mask;
            cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);

            if (target_color_ == "NONE") {
                cv::putText(cv_ptr->image, "Target: NONE", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
                debug_image_pub_->publish(*cv_ptr->toImageMsg());
                return;
            }

            if (target_color_ == "BLUE") {
                cv::inRange(hsv_img, cv::Scalar(100, 120, 70), cv::Scalar(140, 255, 255), mask);
            } else if (target_color_ == "GREEN") {
                cv::inRange(hsv_img, cv::Scalar(40, 120, 70), cv::Scalar(80, 255, 255), mask);
            } else { // RED (기본값)
                cv::Mat mask1, mask2;
                cv::inRange(hsv_img, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
                cv::inRange(hsv_img, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
                mask = mask1 | mask2;
            }


            int center_x = msg->width / 2;
            int center_y = msg->height / 2;

            cv::line(cv_ptr->image, cv::Point(center_x - 20, center_y), cv::Point(center_x + 20, center_y), cv::Scalar(0, 255, 0), 2);
            cv::line(cv_ptr->image, cv::Point(center_x, center_y - 20), cv::Point(center_x, center_y + 20), cv::Scalar(0, 255, 0), 2);

            cv::putText(cv_ptr->image, "Target: " + target_color_, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);

            // ============================

            // 무게중심 찾기(Centroid)
            cv::Moments m = cv::moments(mask);
            if(m.m00 > 1000){
                int cx = int(m.m10 / m.m00);
                int cy = int(m.m01 / m.m00);

                cv::circle(cv_ptr->image, cv::Point(cx, cy), 10, cv::Scalar(0, 255, 255), 3);
                cv::line(cv_ptr->image, cv::Point(cx, cy), cv::Point(center_x, center_y), cv::Scalar(0, 0, 255), 1);

                // 오차 계산
                int error_x = center_x - cx;
                int error_y = center_y - cy;

                int deadzone = 20;

                if(std::abs(error_x) < deadzone) error_x = 0;
                if(std::abs(error_y) < deadzone) error_y = 0;

                double gain_yaw = 0.0005;
                double gain_pitch = 0.00025;

                double target_yaw = current_yaw_ + (error_x * gain_yaw);
                double target_pitch = current_pitch_ - (error_y * gain_pitch);



                if(error_x != 0 || error_y != 0){
                    publish_trajectory(target_yaw, target_pitch);;
                }

                // RCLCPP_INFO(this->get_logger(),"error x : %d, current yaw = %f", error_x, current_yaw_);
                // RCLCPP_INFO(this->get_logger(),"error y : %d, current pitch = %f", error_y, current_pitch_);

            }
            debug_image_pub_->publish(*cv_ptr->toImageMsg());
        }
        void publish_trajectory(double yaw, double pitch){
            trajectory_msgs::msg::JointTrajectory traj;
            traj.joint_names = {"yaw_joint", "pitch_joint"};

            trajectory_msgs::msg::JointTrajectoryPoint p;
            p.positions = {yaw, pitch};
            p.time_from_start.sec = 0;
            p.time_from_start.nanosec = 100000000;

            traj.points.push_back(p);
            traj_pub_->publish(traj);
        }

        double current_yaw_;
        double current_pitch_;

        double filtered_yaw_ = 0.0;
        double filtered_pitch_ = 0.0;

        double last_sent_yaw_ = 0.0;
        double last_sent_pitch_ = 0.0;
        std::string target_color_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
