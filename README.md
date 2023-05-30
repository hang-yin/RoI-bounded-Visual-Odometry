# ROI-bounded Visual Odometry

## Test Sequence Demo

https://github.com/hang-yin/RoI-bounded-Visual-Odometry/assets/60046203/d75ac044-fa67-4a98-8839-7e96859f8b00

## Motivation

A conventional approach to monocular visual odometry entails matching low-level local features on successive frames, employing the Essential Matrix and 3D point triangulation to approximate the alteration in pose. Nevertheless, the computation of these low-level features can be considerably time-intensive, particularly with feature-rich algorithms like SIFT (Scale Invariant Feature Transform). Alternatively, one could employ less computationally demanding features like ORB (Oriented FAST and Rotated BRIEF). However, such features typically lead to diminished performance on visual odometry as matching these features between consecutive frames can be challenging. Despite this, SIFT carries the drawback of being markedly more computationally intensive. To illustrate, generating SIFT features for a 600 by 1200 image consumes approximately 0.40s on average on a 12-core Intel i5 PC, whereas ORB requires merely 0.07s per image on average. In the context of compact embedded systems, such as drones, and personal devices equipped with AR/VR capabilities, employing SIFT could significantly compromise real-time performance, a feature critical to these applications.

This project endeavors to harness the accuracy provided by rich features while diminishing the computational resources required for real-time performance. Broadly, my approach exploits the fact that a large proportion of advantageous local features originate from information-dense regions, such as areas populated by objects or regions with a high concentration of edges. This strategy involves the use of region-of-interest proposals as a cost-effective initial step to identify regions likely to yield high-quality features. Subsequently, the computationally demanding feature extraction is performed solely on these selected regions instead of processing the entire image.


## Baseline Visual Odometry

The standard Visual Odometry implementation includes three main stages: local feature extraction, feature matching, and pose estimation. Local features are extracted using OpenCV functions like SIFT_create() and ORB_create(), generating 2D coordinates of local features and their descriptors from an image. Key features are identified, and the K-nearest neighbor method is applied to find two closest matches for each keypoint in the previous frame, using a ratio test to exclude false matches. For pose estimation, the Essential Matrix is calculated using the camera's intrinsic parameters and several feature points. Two plausible rotations and a translation are derived from this matrix, with four possible combinations considered as candidates. These candidates are evaluated using projection matrices and 3D triangulation of the matched 2D points. The optimal candidate is selected based on the quantity of triangulated points in front of the camera and the relative scale factor, which compares the mean distances between successive feature points in the new and previous frames.

## ROI-bounded Visual Odometry

The ROI-bounded Visual Odometry process introduces a method of proposing regions of interest (ROIs) to extract local features using Canny Edge Detection, contour extraction, bounding box placement, and contour box size filtration. The OpenCV Canny method is applied to discover an edge map of an image, and contours in this map are segregated. Bounding boxes encompassing each contour are generated and then filtered based on a low and high area threshold. SIFT features are extracted from these filtered contour boxes, and the rest of the process follows the baseline approach. Cascade-style processing is introduced to reduce run-time by decreasing the resolution of the image for ROI proposal and using the original image for feature extraction within ROIs. Predictive pose estimation is used to calculate a weighted sum of the previous and new transformations to avoid erroneous estimations when there are fewer feature point matches. Lastly, to eliminate overlapping bounding boxes that lead to duplicate feature detection, Intersection over Union (IoU) is used, which calculates the ratio between the intersection and union areas of two bounding boxes.

![image](https://github.com/hang-yin/RoI-bounded-Visual-Odometry/assets/60046203/b485f22f-ec11-4580-b326-798383c5cba8)

## ROS2 package for Real-world Demo

In order to run experiments to assess the proposed methods, I've developed a package for the Robot Operating System (ROS) that reads images from an Intel RealSense D435i, computes odometry paths using the suggested methods, and visualizes the path in RViz, a 3D visualizer for ROS.

![image](https://github.com/hang-yin/RoI-bounded-Visual-Odometry/assets/60046203/244e7af0-74a2-437e-9f03-a5837fa4deff)

## Real-world Demo

https://github.com/hang-yin/RoI-bounded-Visual-Odometry/assets/60046203/3b9544d2-0993-4a2c-8fd6-69019d0a8fe6

## Usage

Check out the pyhon notebooks in the `experiment` directory and the ROS2 package in the `src` directory. 
