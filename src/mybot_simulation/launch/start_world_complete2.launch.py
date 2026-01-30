#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_mybot_simulation = get_package_share_directory('mybot_simulation')

    try:
        # realsense2_description 패키지의 share 경로를 가져옴
        pkg_realsense_share = get_package_share_directory('realsense2_description')
        # Gazebo가 패키지 이름을 찾을 수 있도록 상위 디렉토리(share)를 지정
        pkg_realsense = os.path.abspath(os.path.join(pkg_realsense_share, '..'))
    except PackageNotFoundError:
        print("----------------------------------------------------------------")
        print("WARNING: realsense2_description package not found!")
        print("----------------------------------------------------------------")
        pkg_realsense = ""

    # 모델 경로 설정을 위한 준비
    description_package_name = "mybot_description"
    install_dir = get_package_prefix(description_package_name)

    # 모델 경로들
    gazebo_models_path = os.path.join(pkg_mybot_simulation, 'models', 'small_house')
    my_models_path = os.path.join(pkg_mybot_simulation, 'models')

    # 환경 변수 추가
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += f":{install_dir}/share:{my_models_path}:{gazebo_models_path}:{pkg_realsense}"
    else:
        os.environ['GAZEBO_MODEL_PATH'] = f"{install_dir}/share:{my_models_path}:{gazebo_models_path}:{pkg_realsense}"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] += f":{install_dir}/lib"
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = f"{install_dir}/lib"

    print(f"GAZEBO MODELS PATH: {os.environ['GAZEBO_MODEL_PATH']}")
    print(f"GAZEBO PLUGINS PATH: {os.environ['GAZEBO_PLUGIN_PATH']}")

    # 1. LaunchDescription 객체 생성
    ld = LaunchDescription()

    # 2. Argument 추가
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_mybot_simulation, 'worlds', 'small_house.world'), ''],
        description='SDF world file'
    )
    ld.add_action(world_arg)

    # 3. Gazebo 실행 액션 추가
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )
    ld.add_action(gazebo)

    return ld