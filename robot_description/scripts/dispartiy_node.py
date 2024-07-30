#! /usr/bin/env python3

from message_filters import Subscriber, TimeSynchronizer
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from matplotlib import pyplot as plt
import numpy as np

class Sterio_out(Node):
    """docstring for swift"""
    def __init__(self):
        super().__init__('sterio_out')

        block_size = 11
        min_disp = -128
        max_disp = 128
        num_disp = max_disp - min_disp
        uniquenessRatio = 5
        speckleWindowSize = 200
        speckleRange = 2
        disp12MaxDiff = -1

        self.stereo = cv2.StereoSGBM_create(
    minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=block_size,
    uniquenessRatio=uniquenessRatio,
    speckleWindowSize=speckleWindowSize,
    speckleRange=speckleRange,
    disp12MaxDiff=disp12MaxDiff,
    P1=8 * 1 * block_size * block_size,
    P2=32 * 1 * block_size * block_size,
)

        self.bridge = CvBridge()

        self.camera_R_sub = Subscriber(self, Image, "/camera_L" )
        self.camera_L_sub = Subscriber(self, Image, "/camera_R")


        sync = TimeSynchronizer([self.camera_L_sub, self.camera_R_sub], queue_size = 10)
        sync.registerCallback(self.SyncCallback)
    
    def callback_left(self, msg):
        pass
    
    def callback_right(self, msg):
        pass

    def SyncCallback(self, Left_Feed, Right_Feed): 
        #To obtain both feeds, convert them to cv2 data, then to BGR

        #converting to CV2 format
        cv_image_L = self.bridge.imgmsg_to_cv2(Left_Feed, desired_encoding='passthrough')
        cv_image_R = self.bridge.imgmsg_to_cv2(Right_Feed, desired_encoding='passthrough')

        Limage_rgb = cv2.cvtColor(cv_image_L, cv2.COLOR_BGR2GRAY) 
        Rimage_rgb = cv2.cvtColor(cv_image_R, cv2.COLOR_BGR2GRAY) 
        # stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
        # disparity = stereo.compute(Limage_rgb,Rimage_rgb)
        # plt.imshow(disparity,'gray')
        # # plt.show()
        # print("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")
        cv2.imshow("cam_l", Limage_rgb)
        # # cv2.imshow("cam_r", Rimage_rgb)
        # cv2.waitKey(1)
        disparity_SGBM = self.stereo.compute(Limage_rgb, Rimage_rgb)

        # Normalize the values to a range from 0..255 for a grayscale image
        disparity_SGBM = cv2.normalize(disparity_SGBM, disparity_SGBM, alpha=255,
                                    beta=0, norm_type=cv2.NORM_MINMAX)
        disparity_SGBM = np.uint8(disparity_SGBM)

        cv2.imshow("Disparity", disparity_SGBM)
        cv2.waitKey(1)
if __name__ == "__main__":
    rclpy.init(args=None)
    swift_drone = Sterio_out()
    cv2.destroyAllWindows()
    rclpy.spin(swift_drone)
    swift_drone.destroy_node()
    rclpy.shutdown()
    exit()