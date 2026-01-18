from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():
    pkg_share = FindPackageShare('gimbal_mani')
    xacro_path = PathJoinSubstitution([pkg_share, 'urdf', 'gimbal.xacro'])
    world_path = PathJoinSubstitution([pkg_share, 'urdf', 'gimbal.sdf'])

    robot_description = Command(['xacro ', xacro_path])

    env_libgl = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    # 1. 로봇 모델 퍼블리셔
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True
                     }],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']
            # [FindPackageShare('gazebo_ros'), '/launch', '/gzserver.launch.py'] # 헤드리스
        ),
        launch_arguments={'world': world_path}.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'gimbal'],
        output='screen',
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
        period=10.0,
        actions=[joint_state_spawner]
    )

    gimbal_spawner_delayed = TimerAction(
        period=14.0,
        actions=[gimbal_spawner]
    )

    # teleop_node = Node(
    #     package='gimbal_mani',
    #     executable='teleop',
    #     name='teleop',
    #     output='screen',
    # )

    tracker_node = Node(
        package='gimbal_mani',
        executable='color_tracker',
        name='color_tracker',
        output='screen',
    )

    target_viz_node = Node(
        package='gimbal_mani',
        executable='target_viz',
        name='target_viz',
        output='screen',
    )

    # gz_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #         '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
    #         '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
    #         '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
    #     ],
    #     output='screen'
    # )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    return LaunchDescription([
        env_libgl,
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        joint_state_spawner_delayed,
        gimbal_spawner_delayed,
        # teleop_node,
        tracker_node,
        target_viz_node,
        # gz_bridge,
        # rviz
    ])
