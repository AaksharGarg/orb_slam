#! /usr/bin/env python3

import rclpy
import time
import numpy as np
import cv2
import glob
import argparse

from message_filters import Subscriber, TimeSynchronizer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.node import Node

from geometry_msgs.msg import PoseArray



class Sterio_out(Node):
    """docstring for swift"""
    def __init__(self):
        super().__init__('sterio_out')
        self.bridge = CvBridge()

        #calibration params
        self.criteria = (cv2.TERM_CRITERIA_EPS +
                         cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.criteria_cal = (cv2.TERM_CRITERIA_EPS +
                             cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.objp = np.zeros((9*6, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

        # Arrays to store object points and image points from all the images.
        self.objpoints = []  # 3d point in real world space
        self.imgpoints_l = []  # 2d points in image plane.
        self.imgpoints_r = []  # 2d points in image plane.


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

        Limage_rgb = cv2.cvtColor(cv_image_L, cv2.COLOR_BGR2RGB) 
        Rimage_rgb = cv2.cvtColor(cv_image_R, cv2.COLOR_BGR2RGB) 
        cv2.imshow("cam_l", Limage_rgb)
        cv2.imshow("cam_r", Rimage_rgb)
        cv2.waitKey(1)

if __name__ == "__main__":
    rclpy.init(args=None)
    swift_drone = Sterio_out()
    cv2.destroyAllWindows()
    rclpy.spin(swift_drone)
    swift_drone.destroy_node()
    rclpy.shutdown()
    exit()