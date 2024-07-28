#! /usr/bin/env python3

from message_filters import Subscriber, TimeSynchronizer
from geometry_msgs.msg import PoseArray
import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



class Sterio_out(Node):
    """docstring for swift"""
    def __init__(self):
        super().__init__('sterio_out')
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
        cv_image = self.bridge.imgmsg_to_cv2(Left_Feed, desired_encoding='passthrough')
        
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) 
        cv2.imshow("cam_l", image_rgb)
        cv2.waitKey(1)

if __name__ == "__main__":
    rclpy.init(args=None)
    swift_drone = Sterio_out()
    cv2.destroyAllWindows()
    rclpy.spin(swift_drone)
    swift_drone.destroy_node()
    rclpy.shutdown()
    exit()