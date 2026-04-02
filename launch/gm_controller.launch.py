import os
import re

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def _sanitize_urdf_xml(xml: str) -> str:
    xml = re.sub(r"<\?xml[^>]*\?>", "", xml).strip()
    xml = re.sub(r"<!--.*?-->", "", xml, flags=re.DOTALL)
    xml = " ".join(xml.split())
    return xml


def generate_launch_description():
    pkg_share = get_package_share_directory('gimbal_mani')

    xacro_path = os.path.join(pkg_share, 'urdf', 'gimbal_mani.urdf.xacro')
    world_path = os.path.join(pkg_share, 'urdf', 'gimbal.sdf')

    mesh_prefix = 'file://' + pkg_share
    doc = xacro.process_file(xacro_path, mappings={'mesh_prefix': mesh_prefix})
    robot_desc = doc.toxml()
    robot_desc = _sanitize_urdf_xml(robot_desc)

    # robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }],
    )

    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={'world': world_path,
                          'gui': 'false',
                          'verbose': 'true'
                          }.items(),
    )

    # Gazebo Spawn Entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'gimbal_mani',
        ],
        output='screen',
    )

    # Controllers spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen',
    )

    gimbal_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gimbal_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '60'
        ],
        output='screen',
    )

    start_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_gimbal_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gimbal_controller_spawner],
        )
    )

    start_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    vision = Node(
        package="gimbal_mani",
        executable="vision_node",
        name="vision_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"input_mode": "sim"},
            {"sim_image_topic": "/gimbal/camera/image_raw"},
        ],
    )


    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        start_joint_state_broadcaster,
        start_gimbal_controller,
        start_arm_controller,
        vision
    ])
