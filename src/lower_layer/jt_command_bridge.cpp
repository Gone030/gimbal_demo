#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <algorithm>

class JTCommandBridge final : public rclcpp::Node
{
public:
    JTCommandBridge() : Node("jt_command_bridge"){
        // ===== Parameters =====
        input_topic_ = this->declare_parameter<std::string>(
            "input_joint_trajectory_topic", "/joint_trajectory_in");

        output_topic_ = this->declare_parameter<std::string>(
            "output_joint_trajectory_topic", "/controller/joint_trajectory");

        allowed_joints_ = this->declare_parameter<std::vector<std::string>>(
            "allowed_joints", std::vector<std::string>{});

        // strict_mode: true면 allowed_joints에 없는 joint가 들어오면 드롭
        strict_mode_ = this->declare_parameter<bool>("strict_mode", true);

        // reorder_to_allowed: true면 joint_names/points를 allowed_joints 순서로 재정렬
        reorder_to_allowed_ = this->declare_parameter<bool>("reorder_to_allowed", true);

        // ===== Publisher / Subscriber =====
        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            output_topic_, rclcpp::QoS(10));

        sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            input_topic_, rclcpp::QoS(10),
            std::bind(&JTCommandBridge::on_msg, this, std::placeholders::_1));

        // cache set for quick membership
        allowed_set_.clear();
        for (const auto &j : allowed_joints_)
            allowed_set_.insert(j);

        RCLCPP_INFO(this->get_logger(),
                    "JTCommandBridge ready. input='%s' output='%s' allowed_joints=%zu strict=%d reorder=%d",
                    input_topic_.c_str(), output_topic_.c_str(), allowed_joints_.size(),
                    strict_mode_ ? 1 : 0, reorder_to_allowed_ ? 1 : 0);
    }

private:
    void on_msg(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg){
        if (!msg)
            return;

        // 기본 검증: points가 비어있으면 아무 것도 안 함
        if (msg->points.empty()){
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Received JointTrajectory with empty points. Dropping.");
            return;
        }

        // allowed_joints 미지정이면 그대로 패스
        if (allowed_joints_.empty()){
            auto out = *msg;
            pub_->publish(out);
            return;
        }

        // 입력 joint_names -> index map
        std::unordered_map<std::string, size_t> in_index;
        in_index.reserve(msg->joint_names.size());
        for (size_t i = 0; i < msg->joint_names.size(); ++i){
            in_index[msg->joint_names[i]] = i;
        }

        // strict 검사
        if (strict_mode_){
            for (const auto &j : msg->joint_names){
                if (allowed_set_.find(j) == allowed_set_.end()){
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "Joint '%s' not in allowed_joints. Dropping message.", j.c_str());
                    return;
                }
            }
        }

        trajectory_msgs::msg::JointTrajectory out;
        out.header = msg->header;

        // reorder_to_allowed이면 allowed_joints 순서로 내보냄
        // 그렇지 않으면 입력 순서를 유지하되 allowed에 포함된 것만 남김
        if (reorder_to_allowed_){
            out.joint_names = allowed_joints_;
        }else{
            out.joint_names.clear();
            out.joint_names.reserve(msg->joint_names.size());
            for (const auto &j : msg->joint_names)
            {
                if (allowed_set_.count(j))
                    out.joint_names.push_back(j);
            }
        }

        // points 재구성: positions/velocities/accelerations/effort의 크기를 joint_names에 맞춤
        out.points.resize(msg->points.size());
        for (size_t p = 0; p < msg->points.size(); ++p){
            const auto &in_pt = msg->points[p];
            auto &out_pt = out.points[p];

            out_pt.time_from_start = in_pt.time_from_start;

            const size_t N = out.joint_names.size();
            out_pt.positions.assign(N, 0.0);

            const bool has_vel = (in_pt.velocities.size() == msg->joint_names.size());
            const bool has_acc = (in_pt.accelerations.size() == msg->joint_names.size());
            const bool has_eff = (in_pt.effort.size() == msg->joint_names.size());

            if (has_vel)
                out_pt.velocities.assign(N, 0.0);
            if (has_acc)
                out_pt.accelerations.assign(N, 0.0);
            if (has_eff)
                out_pt.effort.assign(N, 0.0);

            for (size_t k = 0; k < N; ++k){
                const auto &jname = out.joint_names[k];
                auto it = in_index.find(jname);
                if (it == in_index.end()){
                    // strict_mode_==false일 때만 올 수 있음
                    continue;
                }
                const size_t idx = it->second;

                if (in_pt.positions.size() == msg->joint_names.size()){
                    out_pt.positions[k] = in_pt.positions[idx];
                }else{
                    // positions가 기대 크기가 아니면 드롭
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                         "positions size mismatch (got=%zu expected=%zu). Dropping message.",
                                         in_pt.positions.size(), msg->joint_names.size());
                    return;
                }
                if (has_vel)
                    out_pt.velocities[k] = in_pt.velocities[idx];
                if (has_acc)
                    out_pt.accelerations[k] = in_pt.accelerations[idx];
                if (has_eff)
                    out_pt.effort[k] = in_pt.effort[idx];
            }
        }

        pub_->publish(out);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::vector<std::string> allowed_joints_;
    std::unordered_set<std::string> allowed_set_;

    bool strict_mode_{true};
    bool reorder_to_allowed_{true};

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JTCommandBridge>());
    rclcpp::shutdown();
    return 0;
}
