#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_mybot_simulation = get_package_share_directory('mybot_simulation')
    pkg_mybot_description = get_package_share_directory('mybot_description')

    # RViz 설정 파일 경로
    rviz_config_path = os.path.join(pkg_mybot_description, 'rviz', 'urdf_vis.rviz')

    # 1. LaunchDescription 객체 생성
    ld = LaunchDescription()


    # 2. 파라미터 선언 및 추가
    x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position of the robot'
    )
    y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position of the robot'
    )
    z_pose_arg = DeclareLaunchArgument(
        'z_pose', default_value='2.0',
        description='Z position of the robot'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to start RViz'
    )

    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(z_pose_arg)
    ld.add_action(use_rviz_arg)

    # 3. 파라미터 값 참조 변수 생성
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    use_rviz = LaunchConfiguration('use_rviz')

    # 4. 월드 실행 액션 추가 (start_world_complete.launch.py)
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mybot_simulation, 'launch', 'start_world_complete2.launch.py')
        )
    )
    ld.add_action(world_launch)

    # 5. 로봇 스폰 실행 액션 추가
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mybot_simulation, 'launch', 'spawn_robot_complete2.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'use_sim_time': 'true'
        }.items()
    )
    ld.add_action(spawn_launch)

    # 6. RViz 실행 액션 추가
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
        output='screen'
    )
    ld.add_action(rviz_node)

    return ld