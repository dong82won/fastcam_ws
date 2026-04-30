import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

class MotionServiceClient(Node):
    def __init__(self):
        super().__init__('motion2_service_client')

        # 1. 서비스 클라이언트 설정
        self.move_cli = self.create_client(Empty, 'moving')
        self.stop_cli = self.create_client(Empty, 'stop')

        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('moving 서비스 대기 중...')
        while not self.stop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('stop 서비스 대기 중...')

        # 2. 토픽 구독 설정
        self.subscription = self.create_subscription(
            String,
            'control_command',
            self.command_callback,
            10)

        self.get_logger().info("컨트롤러 노드가 시작되었습니다. 토픽 대기 중...")

    def command_callback(self, msg):
        """토픽을 받으면 서비스 호출"""
        command = msg.data
        if command == 'm':
            self.get_logger().info('이동(Moving) 서비스 호출...')
            self.call_service(self.move_cli)
        elif command == 's':
            self.get_logger().info('정지(Stop) 서비스 호출...')
            self.call_service(self.stop_cli)

    def call_service(self, client):
        req = Empty.Request()
        future = client.call_async(req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            future.result()
            self.get_logger().info('서비스 호출이 성공적으로 처리되었습니다.')
        except Exception as e:
            self.get_logger().error(f'서비스 호출 실패: {e}')

def main(args=None):
    rclpy.init(args=args)

    node = MotionServiceClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()