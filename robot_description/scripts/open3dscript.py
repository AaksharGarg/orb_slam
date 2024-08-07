#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import open3d as o3d
import numpy as np
import cv2

class StereoReconstruction(Node):
    def __init__(self):
        super().__init__('stereo_reconstruction')

        self.bridge = CvBridge()

        #subscriber
        self.camera_L_sub = self.create_subscription(Image, '/camera_L', self.callback_left, 10)
        self.camera_R_sub = self.create_subscription(Image, '/camera_R', self.callback_right, 10)

        self.left_image = None
        self.right_image = None

    def callback_left(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def callback_right(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.process_images()

    def process_images(self):
        if self.left_image is None or self.right_image is None:
            return

        #convert img to gray
        left_gray = cv2.cvtColor(self.left_image, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(self.right_image, cv2.COLOR_BGR2GRAY)

        #stereo matching
        stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        
        #disparity map
        disparity = stereo.compute(left_gray, right_gray)

        #normalisation
        disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        disparity_normalized = np.uint8(disparity_normalized)

        cv2.imshow('Disparity', disparity_normalized)
        cv2.waitKey(1)

        #convert disparity mao to point cloud
        h,w = disparity.shape #returns a tuple of the number of rows, columns, and channels(rgbd img)
        focal_len = 1.0
        baseline = 0.1
        pt_cloud = o3d.geometry.PointCloud()
        points= []
        for y in range(h):
            for x in range(w):
                d=disparity[y,x] / 16.0
                if d>0 :
                    x3 = (x - w / 2) * d / focal_len
                    y3 = (y - h / 2) * d / focal_len
                    z3 = baseline * focal_len / d
                    points.append([x3, y3, z3])
        
        pt_cloud.points = o3d.utility.Vector3dVector(points)
        #visualise 
        o3d.visualization.draw_geometries([pt_cloud])


if __name__ == "__main__":
    rclpy.init(args=None)
    stereo_node = StereoReconstruction()
    rclpy.spin(stereo_node)
    rclpy.shutdown()
