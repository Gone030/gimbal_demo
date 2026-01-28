#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>
#include <chrono>
#include <limits>

#include <urdf/model.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/parameter_client.hpp>

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

        {
            using namespace std::chrono_literals;

            auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

            auto client = std::make_shared<rclcpp::SyncParametersClient>(
                exec,
                this,
                "/robot_state_publisher"
            );

            if (!client->wait_for_service(2s)) {
                throw std::runtime_error("SyncParametersClient: /robot_state_publisher not available");
            }

            auto params = client->get_parameters({"robot_description"});
            if (params.empty() || params[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                throw std::runtime_error("Failed to get robot_description from /robot_state_publisher");
            }

            std::string urdf_xml = params[0].as_string();
            if (urdf_xml.empty()) {
                throw std::runtime_error("robot_description from /robot_state_publisher is empty");
            }

            init_from_urdf(urdf_xml);
        }

        pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/ik_target_xy", 10, std::bind(&IKJointStatePublisher::point_callback, this, std::placeholders::_1)
        );

        js_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&IKJointStatePublisher::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Subscribe : /ik_target_xy (geometry_msgs/point)");
    }

    void point_callback(const geometry_msgs::msg::Point& msg){
        if(!std::isfinite(msg.x) || !std::isfinite(msg.y)){
            RCLCPP_WARN(this->get_logger(), "Invalid target point");
            return;
        }

        auto clamp_joint = [this](const std::string& name, double v){
            auto lim = joint_lim_.find(name);
            if(lim == joint_lim_.end()) return v;
            return clamp(v, lim->second.first, lim->second.second);
        };

        auto ang_diff = [](double a, double b){
            return wrap_pi(a - b);
        };

        double th1_up = 0.0, th2_up = 0.0;
        double th1_down = 0.0, th2_down = 0.0;

        const bool ok_up = ik_2link_2d(msg.x, msg.y, L1_, L2_, +1, th1_up, th2_up);
        const bool ok_down = ik_2link_2d(msg.x, msg.y, L1_, L2_, +1, th1_down, th2_down);

        if(!ok_up && !ok_down){
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "(x = %.6f, y = %.6f) is unreachable for L1 = %.3f, L2 = %.3f",
                msg.x, msg.y, L1_, L2_
            );
            return;
        }

        if(ok_up){
            th1_up = clamp_joint("arm_joint2", th1_up);
            th2_up = clamp_joint("arm_joint3", th2_up);
        }
        if(ok_up){
            th1_down = clamp_joint("arm_joint2", th1_down);
            th2_down = clamp_joint("arm_joint3", th2_down);
        }

        const double prev1 = joint_pos_["arm_joint2"];
        const double prev2 = joint_pos_["arm_joint3"];

        auto cost = [&](double t1, double t2){
            return std::fabs(ang_diff(t1, prev1)) + std::fabs(ang_diff(t2, prev2));
        };

        double th1_sel = 0.0, th2_sel = 0.0;

        if(ok_up && !ok_down){
            th1_sel = th1_up; th2_sel = th2_up;
        }else if(!ok_up && ok_down){
            th1_sel = th1_down; th2_sel = th2_down;
        }else{
            const double c_up = cost(th1_up, th2_up);
            const double c_down = cost(th1_down, th2_down);
            if(c_up <= c_down){
                th1_sel = th1_up; th2_sel = th2_up;
            }else{
                th1_sel = th1_down; th2_sel = th2_down;
            }
        }


        joint_pos_["arm_joint2"] = th1_sel;
        joint_pos_["arm_joint3"] = th2_sel;
    }

    void timer_callback(){
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->now();
        js.name = joint_names_;
        js.position.resize(joint_names_.size());

        for(size_t i = 0; i < joint_names_.size(); i++){
            const auto &name = joint_names_[i];
            js.position[i] = joint_pos_.at(name);
        }
        pub_->publish(js);
    }

    void init_from_urdf(const std::string& urdf_xml){
        urdf::Model model;
        if(!model.initString(urdf_xml)){
            throw std::runtime_error("URDF parse failed");
        }

        joint_names_.clear();
        joint_pos_.clear();

        for(const auto& kv : model.joints_){
            const auto& j = kv.second;
            if(!j)continue;
            if(j->type == urdf::Joint::FIXED) continue;

            joint_names_.push_back(j->name);
            if(j->limits){
                joint_lim_[j->name] = { j->limits->lower, j->limits->upper };
            }
        }

        std::sort(joint_names_.begin(), joint_names_.end());

        for(const auto& name : joint_names_){
            joint_pos_[name] = 0.0;
        }
    }

private:
    double L1_{0.175}, L2_{0.1};
    std::string joint2_name_{"arm_joint2"};
    std::string joint3_name_{"arm_joint3"};

    std::vector<std::string> joint_names_;
    std::unordered_map<std::string, double> joint_pos_;
    std::unordered_map<std::string, std::pair<double,double>> joint_lim_;

    rclcpp::TimerBase::SharedPtr js_timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IKJointStatePublisher>());
    rclcpp::shutdown();
    return 0;
}
