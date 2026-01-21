import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('gimbal_mani')
    xacro_path = os.path.join(pkg_share, 'urdf', 'gimbal.xacro')

    doc = xacro.process_file(xacro_path)
    robot_desc = doc.toxml()

    robot_state_publisher_node = Node(
        package= 'robot_state_publisher',
        executable= 'robot_state_publisher',
        name= 'robot_state_publisher',
        output= 'screen',
        parameters=[{'robot_description' : robot_desc,
                     'use_sim_time': True
        }]
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
        # joint_state_publisher_gui_node, #!TODO /joint_states 를 해당 노드 혹은 다른 노드에서 퍼블리싱할지 선택
        rviz_node
    ])
