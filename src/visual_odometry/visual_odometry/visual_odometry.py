import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class VisualOdometry(Node):

    def __init__(self):
        super().__init__('airsim_interface')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # Vision setup
        self.cv_bridge = CvBridge()

    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)

    visual_odometry = VisualOdometry()

    rclpy.spin(visual_odometry)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    visual_odometry.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()