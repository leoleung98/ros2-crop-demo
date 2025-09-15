import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, '/row_errors', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/mask', 10)
        self.last_log = 0.0

    def cb(self, msg):
        # Convert incoming (rgb8) to BGR for OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]

        # Use FULL frame (your rows were at the top before)
        roi = frame

        # 1) HSV green
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv,
                               np.array([25, 20, 20], np.uint8),
                               np.array([95, 255, 255], np.uint8))

        # 2) Green dominance in BGR (G much bigger than R/B)
        b, g, r = cv2.split(roi)
        dom = (g.astype(np.int16) - np.maximum(r, b).astype(np.int16))
        dom_mask = (dom > 25).astype(np.uint8) * 255

        # Union
        mask = cv2.bitwise_or(hsv_mask, dom_mask)

        # Lateral offset via centroid
        M = cv2.moments(mask)
        lateral_px = 0.0
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            lateral_px = float(cx - w / 2.0)

        # Heading via mask edges
        edges = cv2.Canny(mask, 50, 150)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180,
                                threshold=40, minLineLength=40, maxLineGap=20)
        heading = 0.0
        if lines is not None:
            angles = []
            for l in lines:
                x1, y1, x2, y2 = l[0]
                dx, dy = float(x2 - x1), float(y2 - y1)
                if abs(dx) > 1:
                    angles.append(np.arctan2(dy, dx))
            if angles:
                heading = float(np.mean(angles))

        # Publish errors
        out = Float32MultiArray()
        out.data = [float(lateral_px), float(heading)]
        self.pub.publish(out)

        # Publish debug mask (mono8) so you can see what we detect
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header
            self.debug_pub.publish(mask_msg)
        except Exception as e:
            pass

        # Throttled log
        now = time.time()
        if now - self.last_log > 0.5:
            green_ratio = float(np.count_nonzero(mask)) / float(mask.size)
            self.get_logger().info(
                f"green_ratio={green_ratio:.3f}, lateral_px={lateral_px:.1f}, heading={heading:.3f}"
            )
            self.last_log = now

def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
