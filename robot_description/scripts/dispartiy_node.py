#! /usr/bin/env python3

from message_filters import Subscriber, TimeSynchronizer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import cv2
import numpy as np

class StereoOut(Node):
    """Node to compute and display disparity image from stereo camera feeds."""
    def __init__(self):
        super().__init__('stereo_out')

        # StereoSGBM parameters
        block_size = 17  # Window size for the stereo block matching
        min_disp = 16    # Minimum possible disparity value
        num_disp = 128 - min_disp  # Maximum disparity minus minimum disparity

        self.stereo = cv2.StereoSGBM_create(
            minDisparity=min_disp,
            numDisparities=num_disp,
            blockSize=block_size,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
            disp12MaxDiff=0,
            P1=8 * 1 * block_size * block_size,
            P2=32 * 1 * block_size * block_size,
        )

        self.bridge = CvBridge()

        self.camera_R_sub = Subscriber(self, Image, "/camera_R")
        self.camera_L_sub = Subscriber(self, Image, "/camera_L")

        sync = TimeSynchronizer([self.camera_L_sub, self.camera_R_sub], queue_size=10)
        sync.registerCallback(self.sync_callback)
    
    def sync_callback(self, left_feed, right_feed):
        # Convert ROS Image messages to OpenCV format
        cv_image_L = self.bridge.imgmsg_to_cv2(left_feed, desired_encoding='passthrough')
        cv_image_R = self.bridge.imgmsg_to_cv2(right_feed, desired_encoding='passthrough')

        # Convert images to grayscale
        gray_L = cv2.cvtColor(cv_image_L, cv2.COLOR_BGR2GRAY)
        gray_R = cv2.cvtColor(cv_image_R, cv2.COLOR_BGR2GRAY)

        # Compute disparity map using StereoSGBM
        disparity = self.stereo.compute(gray_L, gray_R).astype(np.float32) / 16.0

        # Normalize the disparity map for better visualization
        disparity = (disparity - min_disp) / num_disp
        disparity = np.clip(disparity, 0, 1)

        # Convert to 8-bit for display
        disparity_display = (disparity * 255).astype(np.uint8)

        # Display the grayscale images and disparity map
        cv2.imshow("Left Image", gray_L)
        cv2.imshow("Disparity", disparity_display)
        cv2.waitKey(1)

if __name__ == "__main__":
    rclpy.init(args=None)
    stereo_node = StereoOut()
    rclpy.spin(stereo_node)
    stereo_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
    exit()
