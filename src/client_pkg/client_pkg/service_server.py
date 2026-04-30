from std_srvs.srv import Empty
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('service_test_node')

        # 1. 서비스 서버 설정
        self.srv_move = self.create_service(Empty, 'moving', self.move_request_callback)
        self.srv_stop = self.create_service(Empty, 'stop', self.stop_request_callback)

        # 2. 퍼블리셔 설정
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # 3. 로봇의 상태를 제어할 변수 (상태 제어 핵심)
        self.cmd_msg = Twist() # 현재 로봇이 유지해야 할 속도 저장용

        # 4. 타이머 설정 (예: 0.1초마다 실행 = 10Hz)
        # 서비스와 별개로 이 타이머는 계속 돌면서 현재의 self.cmd_msg를 발행합니다.
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Robot Controller Node Started.')

    def move_request_callback(self, request, response):
        """서비스 콜백: 목표 속도만 설정하고 즉시 리턴"""
        self.get_logger().info('Service: Move requested.')
        self.cmd_msg.linear.x = 0.3
        self.cmd_msg.angular.z = 0.3
        return response

    def stop_request_callback(self, request, response):
        """서비스 콜백: 목표 속도를 0으로 설정하고 즉시 리턴"""
        self.get_logger().info('Service: Stop requested.')
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.0
        return response

    def timer_callback(self):
        """타이머 콜백: 실제 로직(퍼블리시)을 담당"""
        # 현재 설정된 속도 값을 주기적으로 발행합니다.
        self.publisher_.publish(self.cmd_msg)
        # (선택 사항) 로봇이 움직이고 있을 때만 로그 출력
        if self.cmd_msg.linear.x != 0.0:
            self.get_logger().info('Publishing: Moving...')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()