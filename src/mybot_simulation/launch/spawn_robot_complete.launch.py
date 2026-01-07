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
    xacro_file = os.path.join(pkg_description, 'urdf', 'mybot.xacro')

    # 1. 인자 선언 (이 부분이 있어야 상위 파일에서 값을 받아올 수 있습니다)
    # 기본값(default_value)을 설정해두어 단독 실행 시에도 에러가 안 나게 함
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.5')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    # 2. 파라미터 가져오기
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 3. Robot Description 처리
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 4. Robot State Publisher (use_sim_time 추가)
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

    # 5. Spawn Entity (topic 방식 사용)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=[
            '-entity', 'mybot',
            '-topic', 'robot_description', # 파일 경로 대신 토픽 사용
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ]
    )

    return LaunchDescription([
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        spawn_entity_node
    ])