from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Upper-layer: gimbal controller logic (sub: /target, pub: /joint_trajectory_in/gimbal)
    gimbal_upper = Node(
        package="gimbal_mani",
        executable="gimbal_upper_layer_main",
        name="gimbal_upper_layer_main",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Lower-layer: bridge (in: /joint_trajectory_in/gimbal -> out: /gimbal_controller/joint_trajectory)
    jt_bridge_gimbal = Node(
        package="gimbal_mani",
        executable="jt_command_bridge",
        name="jt_command_bridge_gimbal",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"input_joint_trajectory_topic": "/joint_trajectory_in/gimbal"},
            {"output_joint_trajectory_topic": "/gimbal_controller/joint_trajectory"},
        ],
    )

    return LaunchDescription([
        gimbal_upper,
        jt_bridge_gimbal,
    ])
