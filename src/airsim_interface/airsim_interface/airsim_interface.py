import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import airsim
import numpy as np


class AirSimInterface(Node):

    def __init__(self):
        super().__init__('airsim_interface')
        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # AirSim setup
        self.car_client = airsim.CarClient()
        self.car_client.confirmConnection()
        # Maybe we need to manually control the car instead of using API control
        # If we eventually get to workin on autonomous navigation, we might enable this
        # but for now, use manual control
        #       self.car_client.enableApiControl(True)
        #       self.car_controls = airsim.CarControls()

        # Vision setup
        self.cv_bridge = CvBridge()

        # Create an Image publisher
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

    def timer_callback(self):
        car_state = self.car_client.getCarState()
        self.get_logger().info('Speed: %d, Gear: %d' % (car_state.speed, car_state.gear))
        # Get images from AirSim and publish them
        # Get the image from AirSim
        responses = self.car_client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        ])
        response = responses[0]
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)
        # Convert to ROS Image message
        ros_img = self.cv_bridge.cv2_to_imgmsg(img_rgb, encoding="rgb8")
        # Publish ROS Image message
        self.image_pub.publish(ros_img)


def main(args=None):
    rclpy.init(args=args)

    airsim_interface = AirSimInterface()

    rclpy.spin(airsim_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    airsim_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()