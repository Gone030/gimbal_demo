#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include "gimbal_mani/msg/gimbal_manual_cmd.hpp"

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class GimbalTeleopNode : public rclcpp::Node
{
public:
    GimbalTeleopNode() : Node("gimbal_teleop_node"),
                         target_yaw_(0.0),
                         target_pitch_(0.0),
                         step_rad_(0.02),
                         manual_mode_(true)
    {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&GimbalTeleopNode::timerCallback, this));

        tracker_cmd_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/gimbal/tracker", 10);

        manual_cmd_pub_ = this->create_publisher<gimbal_mani::msg::GimbalManualCmd>(
            "/gimbal/manual", 10);

        enableRawMode();
    }

    ~GimbalTeleopNode()
    {
        disableRawMode();
    }

private:
    void enableRawMode()
    {
        tcgetattr(STDIN_FILENO, &orig_termios_);
        termios raw = orig_termios_;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    void disableRawMode()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
    }

    int readkey()
    {
        char c;
        int n = read(STDIN_FILENO, &c, 1);
        if (n == 1)
            return static_cast<int>(c);
        return -1;
    }

    void handleKey(int key)
    {
        switch (key)
        {
        case 'w':
        case 'W':
            manual_mode_ = true;
            set_mode_manual();
            target_pitch_ -= step_rad_;
            break;
        case 's':
        case 'S':
            manual_mode_ = true;
            set_mode_manual();
            target_pitch_ += step_rad_;
            break;
        case 'a':
        case 'A':
            manual_mode_ = true;
            set_mode_manual();
            target_yaw_ += step_rad_;
            break;
        case 'd':
        case 'D':
            manual_mode_ = true;
            set_mode_manual();
            target_yaw_ -= step_rad_;
            break;
        case 'r':
        case 'R':
            manual_mode_ = true;
            set_mode_manual();
            target_pitch_ = 0;
            target_yaw_ = 0;
            break;
        case 'q':
        case 'Q':
            rclcpp::shutdown();
            break;
        case ',':
            manual_mode_ = false;
            set_mode_auto_red();
            break;

        case '.':
            manual_mode_ = false;
            set_mode_auto_green();
            break;

        case '/':
            manual_mode_ = false;
            set_mode_auto_blue();
            break;

        default:
            break;
        }
        publish_manual_cmd(target_yaw_, target_pitch_);
    }

    void timerCallback()
    {
        int key = readkey();
        if (key != -1)
        {
            handleKey(key);
        }
    }

    void set_mode_manual()
    {
        std_msgs::msg::String cmd;
        cmd.data = "NONE";
        tracker_cmd_pub_->publish(cmd);
    }

    void publish_manual_cmd(double yaw, double pitch)
    {
        gimbal_mani::msg::GimbalManualCmd msg;
        msg.yaw = yaw;
        msg.pitch = pitch;
        manual_cmd_pub_->publish(msg);
    }

    void publish_tracker_cmd(const std::string &s)
    {
        std_msgs::msg::String msg;
        msg.data = s;
        tracker_cmd_pub_->publish(msg);
    }

    void set_mode_auto_red() { publish_tracker_cmd("RED"); }
    void set_mode_auto_green() { publish_tracker_cmd("GREEN"); }
    void set_mode_auto_blue() { publish_tracker_cmd("BLUE"); }

    rclcpp::Publisher<gimbal_mani::msg::GimbalManualCmd>::SharedPtr manual_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracker_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    termios orig_termios_;
    double target_yaw_;
    double target_pitch_;
    double step_rad_;
    bool manual_mode_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
