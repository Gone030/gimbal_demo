from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Upper-layer: arm controller logic (sub: /target, pub: /joint_trajectory_in/arm)
    arm_upper = Node(
        package="gimbal_mani",
        executable="arm_upper_layer_main",
        name="arm_upper_layer_main",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Lower-layer: bridge (in: /joint_trajectory_in/arm -> out: /arm_controller/joint_trajectory)
    jt_bridge_arm = Node(
        package="gimbal_mani",
        executable="jt_command_bridge",
        name="jt_command_bridge_arm",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"input_joint_trajectory_topic": "/joint_trajectory_in/arm"},
            {"output_joint_trajectory_topic": "/arm_controller/joint_trajectory"}
        ]
    )

    return LaunchDescription([
        arm_upper,
        jt_bridge_arm
    ])
