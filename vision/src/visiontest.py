#!/usr/bin/env python


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
#from colours import Colours
from cv_bridge import CvBridge, CvBridgeError

class visiontest:
    def __init__(self):
        
        self.subscriber_colour_image = rospy.Subscriber('camera/color/image_raw', Image, self.callback_colour_image)
        self.subscriber_camera_info = rospy.Subscriber('camera/color/camera_info', CameraInfo, self.callback_camerainfo)
        self.pub_color_image = rospy.Publisher("/visp_auto_tracker/camera/color/image_raw", Image, queue_size=1)
        self.pub_image = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
        self.pub_image2 = rospy.Publisher("/image_raw", Image, queue_size=1)
        self.pub_rect_image = rospy.Publisher("/camera/image_rect", Image, queue_size=1)
        self.pub_rect_image2 = rospy.Publisher("/image_rect", Image, queue_size=1)
        self.pub_cam_info = rospy.Publisher("/visp_auto_tracker/camera/color/camera_info", CameraInfo, queue_size=1)
        self.pub_cam_info2 = rospy.Publisher("/camera/camera_info", CameraInfo, queue_size=1)
        self.pub_cam_info3 = rospy.Publisher("/camera_info", CameraInfo, queue_size=1)

    def callback_colour_image(self,data):
        self.pub_color_image.publish(data)
        #self.pub_rect_image.publish(data)
        #self.pub_rect_image2.publish(data)
        #self.pub_image.publish(data)
        #self.pub_image2.publish(data)

    def callback_camerainfo (self, data):
        self.pub_cam_info.publish(data)
        #self.pub_cam_info2.publish(data)
        #self.pub_cam_info3.publish(data)
        """
        self.bridge = CvBridge()
        self.subscriber_colour_image = rospy.Subscriber('camera/color/image_raw/compressed', CompressedImage, self.callback_colour_image)
        self.subscriber_depth = rospy.Subscriber('camera/aligned_depth_to_color/image_raw/', Image, self.callback_depth)

    # get the image for the robot 
    def callback_colour_image(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def find_circles(self, cvMat):
        print "not implemented "

    

    def callback_depth(self, data):
        print "not implemented"
"""

if __name__ == '__main__':
    print "Starting ROS Beacon Detector module"
    rospy.init_node('visiontest', anonymous=True)
    vt = visiontest()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Beacon Detector module"
