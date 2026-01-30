#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('mybot_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'mybot3_cam.xacro')

    # 1. LaunchDescription 객체 생성
    ld = LaunchDescription()

    # 2. 인자 선언 및 추가
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.5')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(z_pose_arg)
    ld.add_action(use_sim_time_arg)

    # 3. 파라미터 가져오기 (Configuration은 액션이 아니므로 add_action 불필요)
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 4. Robot Description 처리
    robot_description_content = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # 5. Robot State Publisher 노드 추가
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    ld.add_action(robot_state_publisher_node)

    # 6. Spawn Entity 노드 추가
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-entity', 'mybot',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ]
    )
    ld.add_action(spawn_entity_node)

    return ld