import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
import sys

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # 1. 두 개의 클라이언트 생성
        self.move_cli = self.create_client(Empty, 'moving')
        self.stop_cli = self.create_client(Empty, 'stop')

        # 서비스 서버 대기
        while not self.move_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('moving 서비스 대기 중...')
        while not self.stop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('stop 서비스 대기 중...')
        self.req = Empty.Request()

    def send_request(self, command):
        if command == 'm':
            self.get_logger().info('이동(Moving) 명령 전송...')
            return self.move_cli.call_async(self.req)
        elif command == 's':
            self.get_logger().info('정지(Stop) 명령 전송...')
            return self.stop_cli.call_async(self.req)
        return None

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()

    print("\n--- 로봇 제어 프로그램 ---")
    print("m: 이동 명령 (Moving)")
    print("s: 정지 명령 (Stop)")
    print("q: 프로그램 종료")
    print("------------------------\n")

    try:
        while rclpy.ok():
            # 2. 사용자로부터 키보드 입력 받기
            user_input = input("명령을 입력하세요 (m/s/q): ").strip().lower() # strip()으로 공백 제거

            if user_input == 'q':
                print("프로그램을 종료합니다.")
                break 
            
            elif user_input in ['m', 's']:
                # 3. 입력에 따른 서비스 호출
                future = controller.send_request(user_input)

                if future is not None:
                    print("서버의 응답을 기다리는 중...") # 이 메시지 출력 후 멈춘다면 100% 서버 문제
                    
                    # 비동기 응답 대기
                    rclpy.spin_until_future_complete(controller, future)

                    if future.done():
                        try:
                            future.result()
                            status = "이동 시작" if user_input == 'm' else "정지 완료"
                            controller.get_logger().info(f'결과: {status}')
                        except Exception as e:
                            controller.get_logger().error(f'서비스 호출 실패: {e}')
            else:
                # 올바른 들여쓰기 위치
                print("잘못된 입력입니다. m, s, q 중에서만 입력해주세요.")

    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()