import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import airsim
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros

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
        self.image_pub = self.create_publisher(Image, '/airsim/image_raw', 10)

        # Create a Path publisher
        self.path_pub = self.create_publisher(Path, '/airsim/path', 10)

        # Initialize pose message
        # self.pose_msg = PoseStamped()
        # self.pose_msg.header.frame_id = "map"

        # Initialize path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        # Create a transform broadcaster for /map
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.map_frame = TransformStamped()
        # map_frame.header.stamp = self.get_clock().now().to_msg()
        self.map_frame.header.frame_id = "map"
        self.map_frame.child_frame_id = "odom"
        self.map_frame.transform.translation.x = 0.0
        self.map_frame.transform.translation.y = 0.0
        self.map_frame.transform.translation.z = 0.0
        self.map_frame.transform.rotation.x = 0.0
        self.map_frame.transform.rotation.y = 0.0
        self.map_frame.transform.rotation.z = 0.0
        self.map_frame.transform.rotation.w = 1.0
        # self.tf_broadcaster.sendTransform(map_frame)

        self.counter = 0

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

        self.map_frame.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.map_frame)

        car_pose = self.car_client.simGetVehiclePose()
        car_pose_x = car_pose.position.x_val
        car_pose_y = car_pose.position.y_val
        car_pose_z = car_pose.position.z_val

        self.counter += 1

        if self.counter % 2 == 0:
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            curr_pose = PoseStamped()
            curr_pose.header.stamp = self.get_clock().now().to_msg()
            curr_pose.header.frame_id = "map"
            curr_pose.pose.position.x = -car_pose_x
            curr_pose.pose.position.y = car_pose_y
            curr_pose.pose.position.z = car_pose_z
            """
            self.pose_msg.header.stamp = self.get_clock().now().to_msg()
            self.pose_msg.pose.position.x = car_pose_x
            self.pose_msg.pose.position.y = car_pose_y
            self.pose_msg.pose.position.z = car_pose_z
            """
            self.path_msg.poses.append(curr_pose)
            self.path_pub.publish(self.path_msg)
            self.counter = 0


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