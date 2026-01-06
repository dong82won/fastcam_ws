#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import random

def generate_launch_description():
    # 패키지 경로
    pkg_my_box_bot_description = get_package_share_directory('my_box_bot_description')

    urdf_path = os.path.join(pkg_my_box_bot_description, 'urdf', 'box_bot_meshes_physical_control.urdf')

    # URDF 내용을 읽어오기 위한 Command 객체
    robot_desc = ParameterValue(Command(['cat ', urdf_path]), value_type=str)

    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # 파라미터 참조 (상위 런치 파일에서 정의됨)
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # Base Name or robot
    robot_base_name = "box_bot"
    entity_name = robot_base_name+"-"+str(int(random.random()*100000))

    # 로봇 모델 spawn 노드
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity',entity_name,
            '-file', urdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            # '-topic', '/robot_description',
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        spawn_entity_node
    ])


# # #!/usr/bin/python3
# # # -*- coding: utf-8 -*-
# # import random

# from launch_ros.actions import Node
# from launch import LaunchDescription

# def generate_launch_description():

#     # Position and orientation
#     # [X, Y, Z]
#     position = [0.0, 0.0, 0.2]
#     # [Roll, Pitch, Yaw]
#     orientation = [0.0, 0.0, 0.0]


#     # Spawn ROBOT Set Gazebo
#     spawn_robot = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         name='spawn_entity',
#         output='screen',
#         arguments=['-entity',
#                 entity_name,
#                 '-x', str(position[0]), '-y', str(position[1]), '-z', str(position[2]),
#                 '-R', str(orientation[0]), '-P', str(orientation[1]), '-Y', str(orientation[2]),
#                 '-topic', '/robot_description'
#                 ]
#     )

#     # create and return launch description object
#     return LaunchDescription(
#         [
#             spawn_robot,
#         ]
#     )