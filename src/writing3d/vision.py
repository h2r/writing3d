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
        self._origin = None  # (x, y) for origin
        self._x_axis = None  # direction vector for pos x
        self._y_axis = None  # direction vector for pos y

    def _cond_set_origin(self, event, x, y):
        ok = self._current_img is not None \
             and self.last_n_keys(1) == "o"
        if ok:
            self._origin = (x, y)
        return ok

    def _update_axis_coords(self, name, x, y):
        axis = getattr(self, "_%s_axis" % name)
        if axis is not None:
            if len(axis) == 1:
                # this is the second check
                axis.append((x,y))
            elif len(axis) == 2:
                # this means we are clearing the previous line
                # and drawing a new one
                axis = [(x,y)]
                import pdb; pdb.set_trace()
            else:
                raise ValueError("Unexpected state. x_axis: %s" % axis)
        else:
            # We just clicked for the first time.
            axis = [(x,y)]
            
    def _cond_set_x_axis(self, event, x, y):
        ok = self._current_img is not None\
             and self.last_n_keys(1) == "x"
        if ok:
            self._update_axis_coords("x", x, y)
        return ok

    def _cond_set_y_axis(self, event, x, y):
        ok = self._current_img is not None\
             and self.last_n_keys(1) == "y"
        if ok:
            self._update_axis_coords("y", x, y)
        return ok
    

    def init(self):
        super(WritingGui, self).init()
        self.register_mouse_click_circle("set_origin",
                                         self._cond_set_origin,
                                         radius=5, color=(64, 179, 239),
                                         clear_previous=True)

        self.register_mouse_click_line_segment("set_x_axis",
                                               self._cond_set_x_axis,
                                               width=5, color=(255, 0, 0),
                                               clear_previous=True)
        
        self.register_mouse_click_line_segment("set_y_axis",
                                               self._cond_set_y_axis,
                                               width=5, color=(10, 245, 10),
                                               clear_previous=True)


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
