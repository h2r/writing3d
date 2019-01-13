#!/usr/bin/env python
#
# Handles the computer-vision side of things.
# - takes picture with kinect
# - a UI to let user specify the "origin" of the character image
#   among other things.
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
from writing3d.cv_util import TkGui
import writing3d.common as common
import argparse
from cv_bridge import CvBridge, CvBridgeError

common.DEBUG_LEVEL = 2

class WritingGui(TkGui):

    def __init__(self):
        super(WritingGui, self).__init__()

    def _cond_choose_origin(self, event, x, y):
        return self.last_n_keys(1) == "o"
            
        

    def init(self):
        super(WritingGui, self).init()
        self.register_mouse_click_circle(self._cond_choose_origin,
                                         radius=5, color=(64, 179, 239))


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
            # The taken image is BGR but we need RGB for Tk.
            self._image_taken = cv2.cvtColor(self._cv_bridge.imgmsg_to_cv2(msg, msg.encoding),
                                             cv2.COLOR_BGR2RGB)

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
    gui = WritingGui()
    gui.init()
    kinect = MovoKinectInterface()
    img = kinect.take_picture()
    gui.show_image(img)
    gui.spin()


if __name__ == "__main__":
    main()
