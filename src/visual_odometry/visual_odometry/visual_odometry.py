import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
import tf2_ros


class VisualOdometry(Node):

    def __init__(self):
        super().__init__('airsim_interface')
        self.cv_bridge = CvBridge()
        # create a subsriber to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/airsim/image_raw',
            self.image_callback,
            10)
        self.last_keypoints = None
        self.last_descriptors = None

        self.current_transformation = np.eye(4)

        # Create a Path publisher
        self.path_pub = self.create_publisher(Path, '/odom/path', 10)

        # Initialize path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

    def image_callback(self, msg):
        # convert the image message to an opencv image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # convert the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Step 1: detect keypoints
        kp, des = self.odb_feature_detector(gray_image)
        # draw only keypoints location,not size and orientation
        img_with_keypoints = cv2.drawKeypoints(gray_image, kp, None, color=(0, 255, 0), flags=0)
        
        # Step 2: get matches
        if self.last_keypoints is not None:
            matches = self.get_matches(des, self.last_descriptors)
            # Draw first 10 matches.
            img_with_matches = cv2.drawMatches(gray_image, kp, gray_image, self.last_keypoints, matches[:10], None, 2, flags=2)
            # cv2.imshow("matches", img_with_matches)
            # cv2.waitKey(1)

            # self.get_logger().info("Number of matches: " + str(len(matches)))
            # Step 3: get transformation matrix
            new_transformation = self.get_transformation_matrix(matches, kp)
            self.current_transformation = np.matmul(self.current_transformation, new_transformation)
            self.get_logger().info("Transformation: " + str(self.current_transformation[:3, 3]))
            # Step 4: publish transformation
            self.path_msg.header.stamp = self.get_clock().now().to_msg()
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.current_transformation[0, 3]
            pose.pose.position.y = self.current_transformation[1, 3]
            pose.pose.position.z = self.current_transformation[2, 3]
            self.path_msg.poses.append(pose)
            self.path_pub.publish(self.path_msg)
        
        self.last_keypoints = kp
        self.last_descriptors = des
        
    
    def odb_feature_detector(self, gray_scale_image):
        orb = cv2.ORB_create()
        kp = orb.detect(gray_scale_image, None)
        kp, des = orb.compute(gray_scale_image, kp)
        return kp, des
    
    def get_matches_deprecated(self, des1, des2):
        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        # Match descriptors.
        matches = bf.match(des1, des2)
        # Sort them in the order of their distance.
        matches = sorted(matches, key=lambda x: x.distance)
        return matches
    
    def get_matches(self, des1, des2):
        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING)
        # Match descriptors.
        matches = bf.knnMatch(des1, des2, k=2)

        # Apply ratio test
        good_matches = []
        for m, n in matches:
            if m.distance < 0.7 * n.distance:
                good_matches.append(m)

        return good_matches
    
    def get_transformation_matrix(self, matches, new_keypoints):
        # extract keypoints from matches
        keypoints_array1 = np.zeros((len(matches), 2))
        keypoints_array2 = np.zeros((len(matches), 2))
        for i, match in enumerate(matches):
            if match.queryIdx < len(self.last_keypoints) and match.trainIdx < len(new_keypoints):
                keypoints_array1[i] = self.last_keypoints[match.queryIdx].pt
                keypoints_array2[i] = new_keypoints[match.trainIdx].pt
        essential_matrix, mask = cv2.findEssentialMat(keypoints_array1, keypoints_array2)
        _, R, t, mask = cv2.recoverPose(essential_matrix, keypoints_array1, keypoints_array2)
        transformation = np.eye(4)
        transformation[:3, :3] = R
        transformation[:3, 3] = t.ravel()
        return transformation


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