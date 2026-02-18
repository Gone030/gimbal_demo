#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

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
    void on_sim_image(const sensor_msgs::msg::Image & /*msg*/)
    {
        // TODO:
        // - msg → cv::Mat 변환
        // - 타겟 검출/추정
        // - TargetBearingRange 채우기
        // publish_target(...)

        // 스켈레톤이므로 publish만 형태로 남김
        publish_dummy();
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

        publish_dummy();
    }

    void publish_dummy()
    {
        // TODO: 실제 로직 들어가면 삭제
        gimbal_mani::msg::TargetBearingRange out;
        out.header.stamp = now();
        out.header.frame_id = "REPLACE_WITH_ACTUAL_FRAME_ID";
        out.yaw = 0.0;
        out.pitch = 0.0;
        out.range = 0.0;
        out.object_yaw = 0.0;

        tbr_pub_->publish(out);
    }

private:
    // params
    std::string input_mode_;
    std::string sim_image_topic_;
    std::string device_;
    double fps_ = 30.0;

    // ros
    rclcpp::Publisher<gimbal_mani::msg::TargetBearingRange>::SharedPtr tbr_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

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
