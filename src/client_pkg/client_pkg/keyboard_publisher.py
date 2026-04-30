import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import termios
import tty

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'control_command', 10)

        self.settings = termios.tcgetattr(sys.stdin)
        print("\n--- 키보드 입력 노드 ---")
        print("m: Moving 서비스 호출")
        print("s: Stop 서비스 호출")
        print("q: 종료")
        print("-----------------------\n")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()

    try:
        while rclpy.ok():
            key = node.get_key()
            if key in ['m', 's']:
                msg = String()
                msg.data = key
                node.publisher_.publish(msg)
                node.get_logger().info(f"명령 발행: {key}")
            elif key == 'q':
                break
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()