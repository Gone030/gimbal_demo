#include <algorithm>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>

static inline double clamp(double v, double lo, double hi){
    return std::max(lo, std::min(v, hi));
}

static inline double wrap_pi(double x){
    while(x <= -M_PI) x += 2.0 * M_PI;
    while(x > M_PI) x -= 2.0 * M_PI;
    return x;
}

static bool ik_2link_2d(double x, double y,
                        double L1, double L2,
                        int elbow_sign,
                        double &theta1, double &theta2)
{
    const double r2 = x*x + y*y;

    double costh2 = (r2 - L1*L1 - L2*L2) / (2.0 * L2 * L1);
    if(!std::isfinite(costh2)){
        return false;
    }

    if(costh2 < -1.000001 || costh2 > 1.000001){
        return false;
    }
    costh2 = clamp(costh2, -1.0, 1.0);

    double sinth2_sq = 1.0 - costh2 * costh2;
    if(sinth2_sq < 0.0) sinth2_sq = 0.0; // 부동소수점처리
    const double sinth2 = (elbow_sign >= 0 ? +1.0 : -1.0) * std::sqrt(sinth2_sq);

    theta2 = std::atan2(sinth2, costh2);

    const double k1 = L1 + L2 * costh2;
    const double k2 = L2 * sinth2;

    theta1 = std::atan2(y, x) - std::atan2(k2, k1);

    theta1 = wrap_pi(theta1);
    theta2 = wrap_pi(theta2);
    return true;
}

class IKJointStatePublisher : public rclcpp::Node{

public:
    IKJointStatePublisher() : Node("ik_joint_state_publisher"){
        L1_ = this->declare_parameter<double>("L1", 0.175);
        L2_ = this->declare_parameter<double>("L2", 0.1);
        elbow_sign_ = this->declare_parameter<int>("elbow_sign", +1); // +1 or -1

        joint2_name_ = this->declare_parameter<std::string>("joint2_name", "arm_joint2");
        joint3_name_ = this->declare_parameter<std::string>("joint3_name", "arm_joint3");

        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            //!TODO 임시로 모든 조인트 정의, /joint_states 소스 단일화 노드 추가
            "/ik_target_xy", 10,
            [this](const geometry_msgs::msg::Point &msg){
                double th1 = 0.0, th2 = 0.0;
                if (!std::isfinite(msg.x) || !std::isfinite(msg.y)) {
                    RCLCPP_WARN(this->get_logger(), "Invalid target point");
                    return;
                }
                const bool done = ik_2link_2d(msg.x, msg.y, L1_, L2_, elbow_sign_, th1, th2);

                if(!done){
                    RCLCPP_WARN_THROTTLE(
                        this->get_logger(), *this->get_clock(), 2000,
                        "(x = %.6f, y = %.6f) is unreachable for L1 = %.3f, L2 = %.3f",
                        msg.x, msg.y, L1_, L2_
                    );
                    return;
                }

                sensor_msgs::msg::JointState js;
                js.header.stamp = this->now();
                js.name = {joint2_name_, joint3_name_};
                js.position = {th1, th2};
                RCLCPP_INFO(this->get_logger(), "JS publishing");
                pub_->publish(js);
            }
        );

        RCLCPP_INFO(this->get_logger(), "Subscribe : /ik_target_xy (geometry_msgs/point)");
    }

private:
    double L1_{0.175}, L2_{0.1};
    int elbow_sign_{+1};
    std::string joint2_name_{"arm_joint2"};
    std::string joint3_name_{"arm_joint3"};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
