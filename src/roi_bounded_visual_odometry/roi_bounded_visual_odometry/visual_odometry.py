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

class ROI:
    def __init__(self, max_dimension=800, roi_threshold_low=100, roi_threshold_high=500, canny_low=100, canny_high=200) -> None:
        self.rois = []
        self.max_dimension = max_dimension
        self.roi_threshold_low = roi_threshold_low
        self.roi_threshold_high = roi_threshold_high
        self.canny_low = canny_low
        self.canny_high = canny_high
    
    def get_rois(self, img):
        max_dimension = max(img.shape)
        scale = self.max_dimension / max_dimension
        my_img = cv2.resize(img, None, fx=scale, fy=scale)
        edges = cv2.Canny(my_img, self.canny_low, self.canny_high)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rois = []
        for contour in contours:
            # filter contours by area
            area = cv2.contourArea(contour)
            if area > self.roi_threshold_low and area < self.roi_threshold_high:
                x, y, w, h = cv2.boundingRect(contour)
                rois.append([x, y, w, h])
        return rois


class VisualOdometry(Node):

    def __init__(self):
        super().__init__('visual_odometry')
        self.cv_bridge = CvBridge()
        # create a subsriber to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
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

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

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

        self.feature_detector = cv2.SIFT_create()
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        self.flann = cv2.FlannBasedMatcher(indexParams=index_params, searchParams=search_params)

        # ROI related
        self.roi_object = ROI(max_dimension=1200,
                              roi_threshold_low=100,
                              roi_threshold_high=1000)
        self.last_kp = None
        self.last_des = None
        self.match_threshold = 0.5
        self.last_roi = None

        # make up K and P matrix
        self.K = np.array([[525.0, 0.0, 319.5],
                            [0.0, 525.0, 239.5],
                            [0.0, 0.0, 1.0]])
        self.P = np.array([[525.0, 0.0, 319.5, 0.0],
                            [0.0, 525.0, 239.5, 0.0],
                            [0.0, 0.0, 1.0, 0.0]])
        
        self.last_image = None
    
    @staticmethod
    def _form_transf(R, t):
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T
    
    def get_matches(self, new_image):
        kp1, des1 = self.feature_detector.detectAndCompute(self.last_image, None)
        kp2, des2 = self.feature_detector.detectAndCompute(new_image, None)
        matches = self.flann.knnMatch(des1.astype(np.float32), des2.astype(np.float32), k=2)

        good_matches = []
        for m, n in matches:
            if m.distance < self.match_threshold * n.distance:
                good_matches.append(m)
        
        q1 = np.array([kp1[m.queryIdx].pt for m in good_matches])
        q2 = np.array([kp2[m.trainIdx].pt for m in good_matches])

        # visualize matches
        # img3 = cv2.drawMatches(self.last_image, kp1, new_image, kp2, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        # cv2.imshow('matches', img3)
        # cv2.waitKey(1)


        return q1, q2
    
    def get_matches_roi(self, i, roi):
        kp1 = self.last_kp
        des1 = self.last_des
        kp2, des2 = self.get_kp_des(i, roi)
        self.last_kp = kp2
        self.last_des = des2
        matches = self.flann.knnMatch(np.array(des1).astype(np.float32), np.array(des2).astype(np.float32), k=2)
        good_matches = []
        if len(matches) > 0:
            for m, n in matches:
                if m.distance < self.match_threshold * n.distance:
                    good_matches.append(m)
        q1 = np.array([kp1[m.queryIdx].pt for m in good_matches])
        q2 = np.array([kp2[m.trainIdx].pt for m in good_matches])
        return q1, q2, kp2
    
    def get_kp_des(self, image, rois):
        kp_list = []
        des_list = []
        for roi in rois:
            roi_img = image[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]]
            kp, des = self.feature_detector.detectAndCompute(roi_img, None)
            for k in kp:
                k.pt = (k.pt[0] + roi[0], k.pt[1] + roi[1])
            if len(kp) == 0:
                continue
            kp_list.extend(kp)
            des_list.extend(des)
        return kp_list, des_list
    
    def get_pose(self, q1, q2):
        # Essential, mask = cv2.findEssentialMat(q1, q2, self.K, cv2.RANSAC, 0.999, 1.0)
        Essential, _ = cv2.findEssentialMat(q1, q2, self.K)
        R, t = self.decompose(Essential, q1, q2)
        return self._form_transf(R, t)
    
    def decompose(self, Essential, q1, q2):
        R1, R2, t = cv2.decomposeEssentialMat(Essential)
        T1 = self._form_transf(R1,np.ndarray.flatten(t))
        T2 = self._form_transf(R2,np.ndarray.flatten(t))
        T3 = self._form_transf(R1,np.ndarray.flatten(-t))
        T4 = self._form_transf(R2,np.ndarray.flatten(-t))
        transformations = [T1, T2, T3, T4]
        K = np.concatenate(( self.K, np.zeros((3,1)) ), axis = 1)
        projections = [K @ T1, K @ T2, K @ T3, K @ T4]
        np.set_printoptions(suppress=True)
        positives = []
        for P, T in zip(projections, transformations):
            hom_Q1 = cv2.triangulatePoints(self.P, P, q1.T, q2.T)
            hom_Q2 = T @ hom_Q1
            Q1 = hom_Q1[:3, :] / hom_Q1[3, :]
            Q2 = hom_Q2[:3, :] / hom_Q2[3, :]  
            total_sum = sum(Q2[2, :] > 0) + sum(Q1[2, :] > 0)
            relative_scale = np.mean(np.linalg.norm(Q1.T[:-1] - Q1.T[1:], axis=-1) /
                                     (np.linalg.norm(Q2.T[:-1] - Q2.T[1:], axis=-1) + 1e-8))
            positives.append(total_sum + relative_scale)
        max = np.argmax(positives)
        if (max == 2):
            return R1, np.ndarray.flatten(-t)
        elif (max == 3):
            return R2, np.ndarray.flatten(-t)
        elif (max == 0):
            return R1, np.ndarray.flatten(t)
        elif (max == 1):
            return R2, np.ndarray.flatten(t)
    
    def timer_callback(self):
        self.map_frame.header.stamp = self.get_clock().now().to_msg()
        self.tf_broadcaster.sendTransform(self.map_frame)

    def image_callback(self, msg):
        # convert the image message to an opencv image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # convert the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        if self.last_image is None:
            self.last_image = gray_image
            roi = self.roi_object.get_rois(gray_image)
            self.last_kp, self.last_des = self.get_kp_des(gray_image, roi)
            return
        
        roi = self.roi_object.get_rois(gray_image)
        self.get_logger().info("Number of rois: " + str(len(roi)))
        q1, q2, new_keypoints = self.get_matches_roi(gray_image, roi)
        # q1, q2 = self.get_matches(gray_image)

        img = cv_image.copy()
        for r in roi:
            cv2.rectangle(img, (r[0], r[1]), (r[0]+r[2], r[1]+r[3]), (255, 255, 0), 2)
        img = cv2.drawKeypoints(img, new_keypoints, None, color=(0, 255, 0), flags=0)
        # use a cv2 window to display images
        cv2.imshow('ROI & keypoints', img)
        cv2.waitKey(100)

        # q1, q2 = self.get_matches(gray_image)
        estimated_pose = self.get_pose(q1, q2)
        self.current_transformation = np.matmul(self.current_transformation, np.linalg.inv(estimated_pose))
        self.get_logger().info("Transformation: " + str(self.current_transformation[:3, 3]))
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.current_transformation[0, 3]
        pose.pose.position.y = self.current_transformation[1, 3]
        # pose.pose.position.z = self.current_transformation[2, 3]
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
        self.last_image = gray_image


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