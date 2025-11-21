import sys
import termios
import tty
import select  # 이 모듈이 빠져서 에러가 났었습니다
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

msg = """
F1TENTH Keyboard Controller
---------------------------
Moving around:
        W
   A    S    D

W: Increase Speed
S: Stop (Zero Speed)
A: Steer Left
D: Steer Right
X: Decrease Speed (Reverse)

CTRL-C to quit
"""

class KeyTeleop(Node):
    def __init__(self):
        super().__init__('key_teleop')
        self.publisher_ = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.speed = 0.0
        self.steering_angle = 0.0
        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        # 수정됨: termios.select -> select.select
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(msg)
        try:
            while True:
                key = self.getKey()
                if key == 'w':
                    self.speed += 0.5
                elif key == 'x':
                    self.speed -= 0.5
                elif key == 's':
                    self.speed = 0.0
                    self.steering_angle = 0.0
                elif key == 'a':
                    self.steering_angle += 0.1
                elif key == 'd':
                    self.steering_angle -= 0.1
                elif key == '\x03':  # Ctrl+C
                    break

                # 속도 및 조향각 제한 (너무 빠르지 않게)
                self.speed = max(min(self.speed, 5.0), -3.0)
                self.steering_angle = max(min(self.steering_angle, 0.4), -0.4)

                drive_msg = AckermannDriveStamped()
                drive_msg.drive.speed = float(self.speed)
                drive_msg.drive.steering_angle = float(self.steering_angle)
                self.publisher_.publish(drive_msg)

                if key in ['w', 'x', 's', 'a', 'd']:
                    print(f"Speed: {self.speed:.1f}, Steer: {self.steering_angle:.1f}")

        except Exception as e:
            print(e)

        finally:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            self.publisher_.publish(drive_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = KeyTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
