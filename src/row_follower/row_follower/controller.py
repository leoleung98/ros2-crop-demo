import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Float32MultiArray, '/row_errors', self.cb, 10)
        self.timer = self.create_timer(0.1, self.tick)
        self.last_err = (0.0, 0.0)
        self.k_yaw = 1.2
        self.k_lat = 0.002
        self.forward = 0.2

    def cb(self, msg):
        if len(msg.data) >= 2:
            self.last_err = (msg.data[0], msg.data[1])

    def tick(self):
        lateral_px, heading = self.last_err
        cmd = Twist()
        cmd.linear.x = self.forward
        omega = self.k_yaw * heading + self.k_lat * lateral_px
        omega = max(min(omega, 1.5), -1.5)
        cmd.angular.z = float(omega)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

