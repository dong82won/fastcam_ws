from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_visualize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_box_bot_description'),
                'launch',
                #URDF파일로 부터 robot_state_publisher과 rviz 실행한다.
                'urdf_visualize_meshes_control.launch.py'
            ])
        ])
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_box_bot_gazebo'),
                'launch',
                # gazebo 환경에서 로봇을 특정 위치에서 spawn 한다.
                'spawn_robot_description.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        urdf_visualize_launch,
        spawn_robot_launch
    ])