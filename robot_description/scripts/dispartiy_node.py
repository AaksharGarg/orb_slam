#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import open3d as o3d
import cv2
import numpy as np

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

        #open3d obj from array
        left_o3d = o3d.geometry.Image(self.left_image)
        right_o3d = o3d.geometry.Image(self.right_image)

        #stereo matching 
        stereo = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            o3d.geometry.PointCloud.create_from_stereo(left_o3d, right_o3d)
        )

        # stereo img to array - better visualisation
        disparity = np.asarray(stereo)
        
        # normalisation
        disparity_normalized = cv2.normalize(disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
        disparity_normalized = np.uint8(disparity_normalized)

        cv2.imshow('Disparity', disparity_normalized)
        cv2.waitKey(1)

if __name__ == "__main__":
    rclpy.init(args=None)
    stereo_node = StereoReconstruction()
    rclpy.spin(stereo_node)
    rclpy.shutdown()
