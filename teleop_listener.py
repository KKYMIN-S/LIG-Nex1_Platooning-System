import sys
sys.path.append('/media/soda/da379f50-c4e9-42d3-8f4a-5f6e4a6550ea/home/soda/rootOnNVMe')
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pop.driving import Driving

class TeleopDrive(Node):
    def __init__(self):
        super().__init__('teleop_drive')
        self.subscriber = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10
        )
        self.car = Driving()

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # 속도값 범위 조정
        throttle = int(abs(linear) * 50)  # 0~99 사이
        self.car.throttle = throttle
        
        #steering_gain = 0.5  # 너무 민감한 회전 방지
        #steering_value = -angular * steering_gain
    
        # 조향값 -1 ~ 1
        self.car.steering = max(-1.0, min(1.0, -angular))  # 반대 방향 보정

        if linear > 0:
            self.car.forward()
        elif linear < 0:
            self.car.backward()
        else:
            self.car.stop()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
