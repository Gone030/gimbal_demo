from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction



def generate_launch_description():
    pkg_share = FindPackageShare('gimbal_demo')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'gimbal.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'camera.rviz'])

    robot_description = Command(['xacro ', xacro_path])

    robot_state_publisher = Node(
        package= 'robot_state_publisher',
        executable= 'robot_state_publisher',
        name= 'robot_state_publisher',
        output= 'screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz_node = Node(
        package= 'rviz2',
        executable= 'rviz2',
        name= 'rviz2',
        output= 'screen',
        arguments=['-d', rviz_config]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']
        )
    )

    spawn_entity = Node(
        package= 'gazebo_ros',
        executable= 'spawn_entity.py',
        arguments= ['-topic', 'robot_description',
                    '-entity', 'gimbal'],
        output= 'screen',
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gimbal_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gimbal_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_state_spawner_delayed = TimerAction(
    period=3.0,
    actions=[joint_state_spawner]
    )

    gimbal_spawner_delayed = TimerAction(
        period=4.0,
        actions=[gimbal_spawner]
    )

    return LaunchDescription([
        rviz_node,
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        joint_state_spawner_delayed,
        gimbal_spawner_delayed,
    ])
