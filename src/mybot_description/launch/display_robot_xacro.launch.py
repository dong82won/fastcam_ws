import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
# 1. ParameterValue 임포트 추가
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    pkg_description = get_package_share_directory('mybot_description')

    # xacro_file = os.path.join(pkg_description, 'urdf', 'mybot2.xacro')
    # xacro_file = os.path.join(pkg_description, 'urdf', 'mybot3_cam.xacro')
    xacro_file = os.path.join(pkg_description, 'urdf', 'test.urdf.xacro')

    # 3. 로봇 모델 처리 (Xacro -> URDF 문자열 변환)
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    default_rviz_config_path = PathJoinSubstitution([pkg_description, 'rviz/urdf_vis.rviz'])

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    return LaunchDescription([
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])