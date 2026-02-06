import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # -------------------------------------------------------------
    # 1. 설정 파일 경로 정의
    # -------------------------------------------------------------
    # [국도] 안정성 위주 (Rviz, TF용)
    udp_config = 'file:///home/won/cycloneDDS_setting/cycloneDDS_lo_udp.xml'

    # [고속도로] 속도 위주 (카메라, 라이다용)
    shm_config = 'file:///home/won/cycloneDDS_setting/cycloneDDS_lo_shm.xml'

    # -------------------------------------------------------------
    # 2. 환경변수 세트 생성 (기존 환경 복사 필수!)
    # -------------------------------------------------------------
    # Set A: UDP용 (TF가 잘 보임)
    env_udp = os.environ.copy()
    env_udp['CYCLONEDDS_URI'] = udp_config

    # Set B: SHM용 (이미지가 빠름)
    env_shm = os.environ.copy()
    env_shm['CYCLONEDDS_URI'] = shm_config

    # -------------------------------------------------------------
    # 3. URDF 파일 로드
    # -------------------------------------------------------------
    urdf_file = 'urdfbot_simple2.urdf'
    package_description = "urdfbot_description"
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)

    with open(robot_desc_path, 'r') as file:
        urdf_content = file.read()
    robot_description = ParameterValue(urdf_content, value_type=str)

    # -------------------------------------------------------------
    # 4. 노드 배치 (핵심: env 골라 쓰기)
    # -------------------------------------------------------------
    # [Node 1] Robot State Publisher -> UDP 사용
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
        env=env_udp,  # <--- UDP 환경 적용
        output="screen"
    )

    # [Node 2] Rviz2 -> UDP 사용 (TF 문제 해결)
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_vis.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_node',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        env=env_udp,  # <--- UDP 환경 적용
        output="screen"
    )

    # [Node 3] 카메라 노드 -> SHM 사용 (대용량 전송)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_node',
        parameters=[{
            'video_device': '/dev/video6',  # 장치 경로
            'image_size': [640, 480],       # 해상도 (리스트 형태)
            'pixel_format': 'YUYV',         # 픽셀 포맷
            'output_encoding': 'rgb8'       # 출력 인코딩
        }],
        env=env_shm,  # <--- 아까 정의한 고속(SHM) 환경변수 적용
        output="screen"
    )

    joint_state_publisher_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            env=env_udp,  # <--- Rviz와 통신을 위해 UDP 환경 적용
            output="screen"
        )
    

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_node,
        #camera_node
    ])