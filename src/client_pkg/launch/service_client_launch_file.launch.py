from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()


    client_node = Node(
        package='client_pkg',
        executable='service_client',
        output='screen'
    )

    motion_client_node = Node(
            package='client_pkg',
            executable='motion_client',
            output='screen'
    )

    # ld.add_action(client_node)
    ld.add_action(motion_client_node)

    return ld
