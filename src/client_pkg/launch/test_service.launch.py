from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()


    client_node = Node(
        package='client_pkg',
        executable='service_client',
        output='screen'
    )

    motion2_client_node = Node(
            package='client_pkg',
            executable='motion2_client',
            output='screen',
    )

    keyboard_node = Node(
            package='client_pkg',
            executable='keyboard_publisher',
            name='keyboard_node',
            output='screen',
            # 핵심: xterm을 사용하여 별도의 입력 터미널을 할당합니다.
            prefix="xterm -e",
            shell=True
        )

    # ld.add_action(client_node)
    # ld.add_action(keyboard_node)
    ld.add_action(motion2_client_node)

    return ld
