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
        self.cv_bridge = CvBridge()
        # create a subsriber to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/airsim/image_raw',
            self.image_callback,
            10)
        self.last_keypoints = None
        self.last_descriptors = None

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
        self.last_keypoints = kp
        self.last_descriptors = des

        # Step 3: get transformation matrix
        if self.last_keypoints is not None:
            R, t = self.get_transformation_matrix(matches)
        
        
    
    def odb_feature_detector(self, gray_scale_image):
        orb = cv2.ORB_create()
        kp = orb.detect(gray_scale_image, None)
        kp, des = orb.compute(gray_scale_image, kp)
        return kp, des
    
    def get_matches(self, des1, des2):
        # create BFMatcher object
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        # Match descriptors.
        matches = bf.match(des1, des2)
        # Sort them in the order of their distance.
        matches = sorted(matches, key=lambda x: x.distance)
        return matches
    
    def get_transformation_matrix(self, matches):
        keypoints_array1 = np.zeros((len(matches), 2))
        keypoints_array2 = np.zeros((len(matches), 2))
        for i, match in enumerate(matches):
            keypoints_array1[i] = self.last_keypoints[match.queryIdx].pt
            keypoints_array2[i] = self.last_keypoints[match.trainIdx].pt
        essential_matrix, mask = cv2.findEssentialMat(keypoints_array1, keypoints_array2)
        _, R, t, mask = cv2.recoverPose(essential_matrix, keypoints_array1, keypoints_array2)
        return R, t
        


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