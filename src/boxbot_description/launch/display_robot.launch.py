from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# this is the function launch  system will look for
def generate_launch_description():

    # 1. 패키지 경로 및 파일 위치 설정
    pkg_share = FindPackageShare('boxbot_description')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf/box_bot_meshes_physical_control.urdf'])

    default_rviz_config_path = PathJoinSubstitution([pkg_share, 'rviz/urdf_vis.rviz'])


    # 2. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['cat ', urdf_path])}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # 3.RVIZ Configuration
    rviz_config_arg = DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    # create and return launch description object
    return LaunchDescription([
        rviz_config_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
