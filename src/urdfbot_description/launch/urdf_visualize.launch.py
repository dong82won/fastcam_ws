import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
# 추가
from launch_ros.parameter_descriptions import ParameterValue

# this is the function launch  system will look for
def generate_launch_description():

    urdf_file = 'urdfbot_simple1.urdf'
    package_description = "urdfbot_description"

    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    with open(robot_desc_path, 'r') as file:
        urdf_content = file.read()
    robot_description = ParameterValue(urdf_content, value_type=str)


    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True,
                    'robot_description': robot_description  # 문자열 형태로 넘기기
					}],
        output="screen"
    )

    # create and return launch description object
    return LaunchDescription(
        [
            robot_state_publisher_node
        ]
    )