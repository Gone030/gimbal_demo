from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('gimbal_demo')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'gimbal.xacro'])

    robot_description = Command([
        'xacro ', xacro_path
    ])

    robot_state_publisher_node = Node(
        package= 'robot_state_publisher',
        executable= 'robot_state_publisher',
        name= 'robot_state_publisher',
        output= 'screen',
        parameters=[{'robot_description' : robot_description}]
    )

    joint_state_publisher_gui_node = Node(
        package= 'joint_state_publisher_gui',
        executable= 'joint_state_publisher_gui',
        name= 'joint_state_publisher_gui',
        output= 'screen'
    )

    rviz_node = Node(
        package= 'rviz2',
        executable= 'rviz2',
        name= 'rviz2',
        output= 'screen',
        arguments=['-d', '']
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
