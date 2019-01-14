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

    ORIGIN_CIRCLE_RADIUS = 5

    def __init__(self):
        super(WritingGui, self).__init__()
        self._origin = None  # (x, y) for origin
        self._x_axis = None  # direction vector for pos x
        self._y_axis = None  # direction vector for pos y
        self._items = {}

    def _cond_set_origin(self, event, x, y):
        return self._current_img is not None \
            and self.last_n_keys(1) == "o"
            
    def _cond_set_x_axis(self, event, x, y):
        return self._current_img is not None\
            and self.last_n_keys(1) == "x"

    def _cond_set_y_axis(self, event, x, y):
        return self._current_img is not None\
            and self.last_n_keys(1) == "y"

    def _store_origin(self, item):
        x, y, _, _ = self._canvas.coords(item)
        self._origin = (x + WritingGui.ORIGIN_CIRCLE_RADIUS,
                        y + WritingGui.ORIGIN_CIRCLE_RADIUS)
        
    def _show_axis_dashed_line(self, axis, item, color=(128, 128, 128)):
        if not hasattr(self, "_origin"):
            util.warning("Origin not created yet. X axis won't be considered!")
            return

        x0, y0, x1, y1 = self._canvas.coords(item)
        setattr(self, "_%s_axis" % axis, util.unit_vector([np.array([x0,y0]),
                                                           np.array([x1,y1])]))

        # Draw a dashed line from origin to very far away (positive x)
        if "%s_dash" % axis in self._items:
            self._canvas.delete(self._items['%s_dash' % axis])
        axis_vec = getattr(self, "_%s_axis" % axis)
            
        start = self._origin
        pos_end = util.point_along(start, axis_vec, t=300)
        neg_end = util.point_along(start, axis_vec, t=-300)
        self._items['%s_dash' % axis] = self._canvas.create_line(neg_end[0],
                                                                 neg_end[1],
                                                                 pos_end[0],
                                                                 pos_end[1],
                                                                 fill=util.rgb_to_hex(color),
                                                                 width=2.0,
                                                                 dash=(4,4))

    def _show_x_axis_dashed_line(self, item):
        self._show_axis_dashed_line("x", item, color=(255, 30, 30))
        
    def _show_y_axis_dashed_line(self, item):
        self._show_axis_dashed_line("y", item, color=(30, 255, 30))
    

    def init(self):
        super(WritingGui, self).init()
        self.register_mouse_click_circle("set_origin",
                                         self._cond_set_origin,
                                         radius=WritingGui.ORIGIN_CIRCLE_RADIUS,
                                         color=(64, 179, 239),
                                         clear_previous=True, done_cb=self._store_origin)

        self.register_mouse_click_line_segment("set_x_axis",
                                               self._cond_set_x_axis,
                                               width=5, color=(255, 0, 0),
                                               clear_previous=True, done_cb=self._show_x_axis_dashed_line)
        
        self.register_mouse_click_line_segment("set_y_axis",
                                               self._cond_set_y_axis,
                                               width=5, color=(10, 245, 10),
                                               clear_previous=True, done_cb=self._show_y_axis_dashed_line)


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
    # kinect = MovoKinectInterface()
    img = np.load("kinect.npy") #kinect.take_picture()
    gui.show_image(img)
    gui.spin()


if __name__ == "__main__":
    main()
