# # odom_publisher.py
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import Twist
# import math
# import time


# class OdomPublisher(Node):
#     def __init__(self):
#         super().__init__('odom_publisher')
#         self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0

#         self.last_time = self.get_clock().now()
#         self.cmd_vel_sub = self.create_subscription(
#             Twist, '/cmd_vel', self.cmd_callback, 10)

#         self.vx = 0.0
#         self.vth = 0.0

#         self.timer = self.create_timer(0.05, self.update_odom)

#     def cmd_callback(self, msg):
#         self.get_logger().info(f"[📥 cmd_vel] vx={msg.linear.x}, vth={msg.angular.z}")
#         self.vx = msg.linear.x
#         self.vth = msg.angular.z
        


#     def update_odom(self):
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
        
#         SCALE = 12.0  # 실험적으로 조정할 보정 계수

#         # ↓ 여기에 직접 SCALE 적용
#         delta_x = self.vx * math.cos(self.theta) * dt * SCALE
#         delta_y = self.vx * math.sin(self.theta) * dt * SCALE
#         delta_theta = self.vth * dt 


#         self.x += delta_x
#         self.y += delta_y
#         self.theta += delta_theta

#         # ⬇️ 여기부터 쿼터니언으로 바꾼 부분
#         qx = 0.0
#         qy = 0.0
#         qz = 0.0
#         qw = 1.0
        
#         ###############################################################
#         qz = math.sin(self.theta / 2.0)
#         qw = math.cos(self.theta / 2.0)
#         #corrected_theta = self.theta + math.pi
#         #qz = math.sin(corrected_theta / 2.0)
#         #qw = math.cos(corrected_theta / 2.0)


#         odom = Odometry()
#         odom.header.stamp = current_time.to_msg()
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = "base_footprint"
#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.orientation.x = qx
#         odom.pose.pose.orientation.y = qy
#         odom.pose.pose.orientation.z = qz
#         odom.pose.pose.orientation.w = qw
#         odom.twist.twist.linear.x = self.vx
#         odom.twist.twist.angular.z = self.vth

#         self.odom_pub.publish(odom)

#         # TF 브로드캐스트
#         t = TransformStamped()
#         t.header.stamp = current_time.to_msg()
#         t.header.frame_id = "odom"
#         t.child_frame_id = "base_footprint"
#         t.transform.translation.x = self.x
#         t.transform.translation.y = self.y
#         t.transform.rotation.x = qx
#         t.transform.rotation.y = qy
#         t.transform.rotation.z = qz
#         t.transform.rotation.w = qw
#         self.tf_broadcaster.sendTransform(t)
        
#         # 브로드캐스트: base_footprint -> base_link (보통 고정)
#         t2 = TransformStamped()
#         t2.header.stamp = current_time.to_msg()
#         t2.header.frame_id = "base_footprint"
#         t2.child_frame_id = "base_link"
#         t2.transform.translation.z = 0.1
#         t2.transform.rotation.w = 1.0
#         self.tf_broadcaster.sendTransform(t2)
        

#         #self.get_logger().info(f"[UPDATE] x={self.x:.4f}, y={self.y:.4f}, Î¸={self.theta:.4f}")

#         #self.get_logger().info(f"[theta] {self.theta:.2f} rad, corrected: {corrected_theta:.2f}")
#         #self.get_logger().info(f"[quat] qx={qx}, qy={qy}, qz={qz}, qw={qw}")
#         self.last_time = current_time


# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# odom_publisher.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Twist
import math
import time


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0

        self.last_time = self.get_clock().now()
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        self.vx = 0.0
        self.vth = 0.0

        self.timer = self.create_timer(0.05, self.update_odom)

    def cmd_callback(self, msg):
        self.get_logger().info(f"[ðŸ“¥ cmd_vel] vx={msg.linear.x}, vth={msg.angular.z}")
        self.vx = msg.linear.x
        self.vth = msg.angular.z
        


    def update_odom(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        SCALE = 2.7  # ì‹¤í—˜ì ìœ¼ë¡œ ì¡°ì •í•  ë³´ì • ê³„ìˆ˜

        # â†“ ì—¬ê¸°ì— ì§ì ‘ SCALE ì ìš©
        delta_x = self.vx * math.cos(self.theta) * dt * SCALE
        delta_y = self.vx * math.sin(self.theta) * dt * SCALE
        if abs(self.vx) < 0.01:
            self.vth = 0.0

        delta_theta = self.vth * dt


        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # â¬‡ï¸ ì—¬ê¸°ë¶€í„° ì¿¼í„°ë‹ˆì–¸ìœ¼ë¡œ ë°”ê¾¼ ë¶€ë¶„
        qx = 0.0
        qy = 0.0
        qz = 0.0
        qw = 1.0
        
        ###############################################################
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        #corrected_theta = self.theta + math.pi
        #qz = math.sin(corrected_theta / 2.0)
        #qw = math.cos(corrected_theta / 2.0)


        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        self.odom_pub.publish(odom)

        # TF ë¸Œë¡œë“œìºìŠ¤íŠ¸
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)
        
        # ë¸Œë¡œë“œìºìŠ¤íŠ¸: base_footprint -> base_link (ë³´í†µ ê³ ì •)
        t2 = TransformStamped()
        t2.header.stamp = current_time.to_msg()
        t2.header.frame_id = "base_footprint"
        t2.child_frame_id = "base_link"
        t2.transform.translation.z = 0.1
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)
        

        #self.get_logger().info(f"[UPDATE] x={self.x:.4f}, y={self.y:.4f}, ÃŽÂ¸={self.theta:.4f}")

        #self.get_logger().info(f"[theta] {self.theta:.2f} rad, corrected: {corrected_theta:.2f}")
        #self.get_logger().info(f"[quat] qx={qx}, qy={qy}, qz={qz}, qw={qw}")
        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
