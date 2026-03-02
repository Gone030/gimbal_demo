#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <opencv2/opencv.hpp>
#include "gimbal_mani/msg/target_bearing_range.hpp"

class VisionNode : public rclcpp::Node
{
public:
    VisionNode() : Node("vision_node")
    {

        // input_mode: "sim" or "real"
        input_mode_ = declare_parameter<std::string>("input_mode", "sim");

        // sim image topic name
        sim_image_topic_ = declare_parameter<std::string>("sim_image_topic", "/gimbal/image_raw");

        // real device
        device_ = declare_parameter<std::string>("device", "/dev/video0");

        // capture fps / timer period
        fps_ = declare_parameter<double>("fps", 30.0);

        // ---- pub
        tbr_pub_ = create_publisher<gimbal_mani::msg::TargetBearingRange>(
            "/target", 10);

        tof_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/gimbal/tof_distance", 10,
            [this](const sensor_msgs::msg::LaserScan &msg)
            { on_tof_scan(msg); });

        command_sub_ = create_subscription<std_msgs::msg::String>(
            "/gimbal/tracker", 10,
            [this](const std_msgs::msg::String &msg)
            {
                target_color_ = msg.data;
            });

        // ---- mode select
        if (input_mode_ == "sim")
        {
            image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                sim_image_topic_, 10,
                [this](const sensor_msgs::msg::Image &msg)
                { on_sim_image(msg); });
        }
        else if (input_mode_ == "real")
        {
            start_real_capture();
        }
        else
        {
            RCLCPP_FATAL(get_logger(), "Invalid input_mode: %s", input_mode_.c_str());
            throw std::runtime_error("invalid input_mode");
        }
    }

private:
    void on_sim_image(const sensor_msgs::msg::Image &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        if (target_color_ == "NONE")
        {
            return;
        }

        cv::Mat hsv_img, mask;
        cv::cvtColor(cv_ptr->image, hsv_img, cv::COLOR_BGR2HSV);

        if (target_color_ == "BLUE")
        {
            cv::inRange(hsv_img, cv::Scalar(100, 120, 70), cv::Scalar(140, 255, 255), mask);
        }
        else if (target_color_ == "GREEN")
        {
            cv::inRange(hsv_img, cv::Scalar(40, 120, 70), cv::Scalar(80, 255, 255), mask);
        }
        else
        { // RED
            cv::Mat mask1, mask2;
            cv::inRange(hsv_img, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
            cv::inRange(hsv_img, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);
            mask = mask1 | mask2;
        }

        const int center_x = static_cast<int>(msg.width) / 2;
        const int center_y = static_cast<int>(msg.height) / 2;

        cv::Moments m = cv::moments(mask);
        if (m.m00 <= 1000)
            return;

        const int cx = static_cast<int>(m.m10 / m.m00);
        const int cy = static_cast<int>(m.m01 / m.m00);

        int error_x = center_x - cx;
        int error_y = center_y - cy;

        const int deadzone = 20;
        if (std::abs(error_x) < deadzone)
            error_x = 0;
        if (std::abs(error_y) < deadzone)
            error_y = 0;

        const double gain_yaw = 0.0005;
        const double gain_pitch = 0.00025;

        const double dyaw = (error_x * gain_yaw);
        const double dpitch = -(error_y * gain_pitch);

        if (error_x != 0 || error_y != 0)
        {
            publish_target(dyaw, dpitch, tof_range_m_, 0.0);
        }
    }

    void on_tof_scan(const sensor_msgs::msg::LaserScan &msg)
    {
        if (msg.ranges.empty())
            return;

        const float r0 = msg.ranges[0];
        if (!std::isfinite(r0))
            return;

        if (std::isfinite(msg.range_min) && r0 < msg.range_min)
            return;
        if (std::isfinite(msg.range_max) && r0 > msg.range_max)
            return;

        tof_range_m_ = static_cast<double>(r0);
    }

    void start_real_capture()
    {
        cap_.open(device_, cv::CAP_V4L2);
        if (!cap_.isOpened())
        {
            RCLCPP_FATAL(get_logger(), "Failed to open device: %s", device_.c_str());
            throw std::runtime_error("failed to open video device");
        }

        const auto period_ms = static_cast<int>(1000.0 / std::max(1.0, fps_));
        timer_ = create_wall_timer(
            std::chrono::milliseconds(period_ms),
            [this]()
            { poll_real_frame(); });
    }

    void poll_real_frame()
    {
        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty())
        {
            RCLCPP_WARN(get_logger(), "Failed to read frame");
            return;
        }

        // TODO:
        // - frame에서 타겟 검출/추정
        // - TargetBearingRange 채우기
        // publish_target(...)

    }

    void publish_target(double yaw, double pitch, double range, double object_yaw)
    {
        gimbal_mani::msg::TargetBearingRange out;
        out.header.stamp = now();
        out.header.frame_id = "gimbal_camera_link";
        out.yaw = yaw;
        out.pitch = pitch;
        out.range = range;
        out.object_yaw = object_yaw;
        tbr_pub_->publish(out);
    }

private:
    // params
    std::string input_mode_;
    std::string sim_image_topic_;
    std::string device_;
    double fps_ = 30.0;

    std::string target_color_ = "NONE";
    double current_yaw_ = 0.0;
    double current_pitch_ = 0.0;
    double tof_range_m_ = std::numeric_limits<double>::quiet_NaN();


    // ros
    rclcpp::Publisher<gimbal_mani::msg::TargetBearingRange>::SharedPtr tbr_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr tof_sub_;

    // real capture
    cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionNode>());
    rclcpp::shutdown();
    return 0;
}
