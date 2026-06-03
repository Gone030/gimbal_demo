from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    arm_joint_test = Node(
        package="gimbal_mani",
        executable="arm_joint_test_node",
        name="arm_joint_test_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    jt_bridge_arm = Node(
        package="gimbal_mani",
        executable="jt_command_bridge",
        name="jt_command_bridge_arm",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"input_joint_trajectory_topic": "/joint_trajectory_in/arm"},
            {"output_joint_trajectory_topic": "/arm_controller/joint_trajectory"},
            {
                "allowed_joints": [
                    "Shoulder_Rotation",
                    "Shoulder_Pitch",
                    "Elbow",
                    "Wrist_Pitch",
                    "Wrist_Roll",
                    "Gripper",
                ]
            },
        ],
    )

    return LaunchDescription([
        arm_joint_test,
        jt_bridge_arm,
    ])
