import numpy as np
import os
import cv2
from tqdm import tqdm
from matplotlib import pyplot as plt
import time

# class for ROI (region of interest) proposal
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

class ImageTools():
    def __init__(self, image_path):
        self.image_path = image_path
    
    def read_calibration(self, calib_path):
        with open(os.path.join(self.image_path, calib_path), 'r') as f:
            params = np.fromstring(f.readline(), dtype=np.float64, sep=' ')
            P = np.reshape(params, (3, 4))
            K = P[0:3, 0:3]
        return K, P
    
    def read_groundtruth_path(self, gt_path):
        path = []
        with open(os.path.join(self.image_path, gt_path), 'r') as f:
            for line in f.readlines():
                T = np.fromstring(line, dtype=np.float64, sep=' ')
                T = T.reshape(3, 4)
                path.append(T)
        return np.array(path)

    def read_images(self, image_path):
        images = []
        for filename in sorted(os.listdir(os.path.join(self.image_path, image_path))):
            img = cv2.imread(os.path.join(os.path.join(self.image_path, image_path), filename))
            images.append(img)
        return images
    
    def read_all(self):
        K, P = self.read_calibration('calib.txt')
        path = self.read_groundtruth_path('poses.txt')
        images = self.read_images('image_l')
        return K, P, path, images

class PlottingTools():
    def plot_trajectory(self, truth_path, estimated_path):
        truth_path = np.array(truth_path)
        estimated_path = np.array(estimated_path)
        fig = plt.figure()
        # set limit on x axis
        plt.xlim(-10, 10)
        plt.plot(truth_path[:, 0, 3], truth_path[:, 2, 3], label='Ground Truth')
        plt.plot(estimated_path[:, 0, 3], estimated_path[:, 2, 3], label='Estimated')
        plt.legend()
        plt.show()

    def plot_error(self, truth_path, estimated_path):
        truth_path = np.array(truth_path)
        estimated_path = np.array(estimated_path)
        error = np.linalg.norm(truth_path[:, :3, 3] - estimated_path[:, :3, 3], axis=1)
        plt.plot(error)
        plt.show()

