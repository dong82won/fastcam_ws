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
    # 1. 패키지 및 파일 경로 설정
    pkg_description = get_package_share_directory('mybot_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'mybot.xacro')


    # 2. 런치 인자 정의 (상위 런치에서 값을 넘겨주지 않아도 에러가 나지 않도록 함)
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.0')

    # 3. 로봇 모델 처리 (Xacro -> URDF 문자열 변환)
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 4. Robot State Publisher 노드
    # URDF 데이터를 'robot_description' 토픽으로 퍼블리시함
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 5. Spawn Entity 노드
    # 'robot_description' 토픽을 구독하여 Gazebo 월드에 모델을 생성
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

    # 6. 실행할 노드들 리스트 반환
    return LaunchDescription([
        robot_state_publisher_node,
        spawn_entity_node
    ])