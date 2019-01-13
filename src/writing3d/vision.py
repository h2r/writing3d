#!/usr/bin/env python
#
# Handles the computer-vision side of things.
# - takes picture with kinect
# - a UI to let user specify the "origin" of the character image
#   and helps user align the table to the robot.
# - extract character image from the region;
# - saves the image, which may correspond to the result of writing
#   one stroke.
#   - possibility: get the difference between strokes to extract
#     the stroke by itself.
#
# Note: At the time of writing, the cv2 version used is '3.3.1-dev'.
# The opencv version returned by 'pkg-config --modversion opencv' is
# 2.4.9.
#
# Utilizes cv_bridge ros package to convert ros image message to
# opencv image. Reference:
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rospy
import numpy as np
import sensor_msgs.msg
import cv2
import copy
import writing3d.util as util
import argparse
from cv_bridge import CvBridge, CvBridgeError

class MovoKinectInterface:

    def __init__(self):
        self._image_taken = None
        self._cv_bridge = CvBridge()
        rospy.init_node('movo_kinect_interface', anonymous=True)
    
    def take_picture(self):
        """
        Returns an image taken from the kinect as an opencv image.
        Blocking call; Returns only when an image is obtained.
        """
        
        def get_picture(msg):
            # The image taken should have encoding "bgr8" which corresponds
            # to CV_8UC3 according to
            self._image_taken = self._cv_bridge.imgmsg_to_cv2(msg, msg.encoding)

        util.info("Taking picture with Kinect")
        rospy.Subscriber("movo_camera/color/image_color_rect",
                         sensor_msgs.msg.Image, get_picture)
        while self._image_taken is None:
            rospy.sleep(0.5)
        img = copy.deepcopy(self._image_taken)
        self._image_taken = None
        return img
            

class CharacterExtractor:
    pass

def main():
    kinect = MovoKinectInterface()
    img = kinect.take_picture()
    cv2.imshow('image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
