#!/usr/bin/env python
#
# If ImageTk not found:
#   sudo apt-get install python-imaging-tk
#
# References: https://stackoverflow.com/questions/19030579/tkinter-not-found
# From https://docs.python.org/2/library/tkinter.html:
#   the add in bind() is optional, either '' or '+'. Passing an empty string
#   denotes that this binding is to replace any other bindings that this event
#   is associated with. Passing a '+' means that this function is to be added
#   to the list of functions bound to this event type.
#
# On Python 2, import Tkinter. On python 3, import tkinter
#
# The process of determination of the character window
# 1. set 4 points
# 2. remove perspective

import cv2
import numpy as np
import copy
import argparse
import sensor_msgs.msg
import writing3d.common as common
import writing3d.util as util
from writing3d.robot.movo_vision import MovoKinectInterface
import Tkinter as tk
from PIL import Image, ImageTk


common.DEBUG_LEVEL = 2


class TkGui(object):

    def __init__(self):
        self._root = None
        self._canvas = None
        self._images = {}  # maps name to [np.array, int]
        self._last_10_keys = []
        self._shapes = {}
        self._key_cb = {}
    
    def init(self):
        self._root = tk.Tk()
        self._width = 0
        self._height = 0
        self._canvas = tk.Canvas(self._root, width=self._width,
                                 height=self._height)
        self._canvas.config(bg='black')
        # pack the canvas into a frame/form
        self._canvas.pack(fill=tk.BOTH)
        self._root.bind("<Key>", self._key_pressed)
    
    def show_image(self, name, img, loc=(0,0), anchor='nw',
                   scale=1, background=False, interpolation=cv2.INTER_LINEAR):
        """
        Given an image `img` as np.array (RGB), display the image on Tk window.
        If name already exists, will override the old one. Returns the size of
        the drawn image.
        """
        if name in self._images:
            self._canvas.delete(self._images[name][0])
        self._images[name] = [img, None, -1]  # image (original size), img_tk (shown), item_id
        img = cv2.resize(img, (int(round(img.shape[1]*scale)),
                               int(round(img.shape[0]*scale))), interpolation=interpolation)
        self._images[name][1] = ImageTk.PhotoImage(image=Image.fromarray(img))
        self._images[name][2] = self._canvas.create_image(loc[0], loc[1],
                                                          anchor=anchor, image=self._images[name][1])
        if background:
            self._width = img.shape[1]
            self._height = img.shape[0]
            self._canvas.config(width=self._width,
                                height=self._height)
        return self._images[name][1].width(), self._images[name][1].height()

    def remove_image(self, name):
        if name not in self._images:
            raise ValueError("Image named %s does not exist!" % name)
        self._canvas.delete(self._images[name][2])
        del self._images[name]

    def spin(self):
        if self._root:
            self._root.mainloop()
        else:
            raise ValueError("tk root does not exist. Did you init GUI?")


    def last_n_keys(self, n=1):
        """Return a list of n keys that were most recently pressed. Most-recent first."""
        if n <= 0 or n > len(self._last_10_keys):
            util.warning("Do not know key! (got: %d; max_length: %d)"
                         % (n, len(self._last_10_keys)))
            return None
        last_n = list(reversed(self._last_10_keys[-n:]))
        if n == 1:
            return last_n[0]
        else:
            return last_n
    
    def _key_pressed(self, event):
        if len(self._last_10_keys) == 10:
            self._last_10_keys.pop(0)
        util.info("Pressed %s" % repr(event.char), debug_level=3)
        self._last_10_keys.append(event.char)
        util.info2(str(self._last_10_keys), debug_level=3)

    def register_key_press_event(self, event_name, key, callback):
        """Generic function to register callback when pressed key"""
        if key in self._key_cb:
            raise ValueError("Key %s is already bound to another event." % key)
        self._key_cb[key] = event_name
        self._root.bind("<%s>" % key, callback)
            

    """For all mouse event callback functions,  it will be called if
    `cond_func(event, x, y)` evaluates to True. The shapes (or intermediate
    objects necessary) drawn as a result of the callback will be stored in
    self._shapes[event_name]. `done_cb` is called when the shape is drawn,
    with the drawn shape item id as the argument."""
    def register_mouse_click_circle(self, event_name, cond_func, button_num='1',
                                    radius=50, color=(255, 0, 0), loc_func=None,
                                    clear_previous=False, done_cb=None):
        """Draw a circle at the location of mouse click, or some other
        specified location, determined by `loc_func` (if not None):
            loc_func(mouse_x, mouse_y)  --> (x, y)

        button_num can be '1', '2', or '3' (left, middle, right buttons). 
        """
        util.info2("Registering mouse click circle", debug_level=1)
        param = {
            'event_name': event_name,
            'radius': radius,
            'color': util.rgb_to_hex(color),
            'loc_func': loc_func,
            'cond_func': cond_func,
            'clear_previous': clear_previous,
            'done_cb': done_cb
        }
        if event_name in self._shapes:
            raise ValueError("Event %s already exists" % event_name)
        self._shapes[event_name] = []
        self._canvas.bind("<Button-%s>" % button_num,
                          lambda event: self._mouse_click_circle(event, param),
                          add="+")

    def _mouse_click_circle(self, event, param):
        x, y = event.x, event.y
        if param['cond_func'](event, x, y):
            if param['loc_func'] is not None:
                x, y = param['loc_func'](x, y)

            r = param['radius']
            if param['clear_previous'] and len(self._shapes[param['event_name']]) > 0:
                self._canvas.delete(self._shapes[param['event_name']][-1])
                self._shapes[param['event_name']].pop(-1)
            self._shapes[param['event_name']].append(self._canvas.create_oval(x-r, y-r, x+r, y+r, fill=param['color'],
                                                                              outline=""))
            if param['done_cb'] is not None:
                param['done_cb'](self._shapes[param['event_name']][-1])

            
            
    def register_mouse_click_line_segment(self, event_name, cond_func, button_num='1',
                                          width=1.0, dash=None, loc_func=None, color=(255, 0, 0),
                                          clear_previous=False, done_cb=None):
        """
        Draw a line segment with two mouse clicks. The underlying event callback
        is for mouse click. 

        `loc_func` is similar to that in register_mouse_click_circle:
            loc_func(x, y) --> x, y
        """
        util.info2("Registering mouse click line segment", debug_level=1)
        param = {
            'event_name': event_name,
            'color': util.rgb_to_hex(color),
            'dash': dash,
            'width': width,
            'loc_func': loc_func,
            'cond_func': cond_func,
            'clear_previous': clear_previous,
            'done_cb': done_cb
        }
        if event_name in self._shapes:
            raise ValueError("Event %s already exists" % event_name)
        self._shapes[event_name] = []
        self._canvas.bind("<Button-%s>" % button_num,
                          lambda event: self._mouse_click_line_segment(event, param),
                          add="+")
        

    def _mouse_click_line_segment(self, event, param):
        x, y = event.x, event.y

        if not param['cond_func'](event, x, y):
            return
        # See if we want to clear the previous line
        if len(self._shapes[param['event_name']]) > 0 \
           and self._canvas.type(self._shapes[param['event_name']][-1]) == "line" \
           and param['clear_previous']:
            self._canvas.delete(self._shapes[param['event_name']][-1])
            self._shapes[param['event_name']].pop(-1)

        if len(self._shapes[param['event_name']]) == 0 \
           or self._canvas.type(self._shapes[param['event_name']][-1]) == "line":
            self._shapes[param['event_name']].append(None)

        if param['loc_func'] is not None:
            x, y = param['loc_func'](x, y)
        if self._shapes[param['event_name']][-1] is None:
            # First click
            # Draw a circle first
            r = param['width'] / 2
            self._shapes[param['event_name']][-1] = self._canvas.create_oval(x-r, y-r, x+r, y+r,
                                                                             fill=param['color'],
                                                                             outline="")
        elif self._canvas.type(self._shapes[param['event_name']][-1]) == "oval":
            # Second click
            # first click pos
            circ_coords = self._canvas.coords(self._shapes[param['event_name']][-1])
            x0 = circ_coords[0] + param['width']/2
            y0 = circ_coords[1] + param['width']/2
            # Delete the circle drawn previousy
            self._canvas.delete(self._shapes[param['event_name']][-1])
            self._shapes[param['event_name']][-1] = self._canvas.create_line(x0, y0, x, y,
                                                                             fill=param['color'],
                                                                             dash=param['dash'],
                                                                             width=param['width'])
            if param['done_cb'] is not None:
                param['done_cb'](self._shapes[param['event_name']][-1])
#--- End TkGui ---#


class WritingGui(TkGui):

    POINT_CIRCLE_RADIUS = 5
    KINECT_IMAGE_NAME = "kinect_view"

    def __init__(self, character_dim=500, hd=True):
        super(WritingGui, self).__init__()
        self._top_left = None  # (x, y) for top_left
        self._bottom_right = None  # (x, y) for bottom_right
        self._top_right = None
        self._bottom_left = None
        self._items = {}
        self._cdim = character_dim
        self._writing_character = None
        self._writing_character_img = None
        self._stroke_images = []
        # counter clock wise
        self._corners_persp = None
        # Scaling
        if hd:
            self._bg_scale = 0.7
            self._box_scale = 0.5
        else:
            self._bg_scale = 1.0
            self._box_scale = 0.3

    def set_writing_character(self, char):
        """Set the writing character. A char is a list of strokes which
        is a list of waypoints x,y,z,z2,al,az. The dimension of the
        character should be equal to self._cdim"""
        self._writing_character = char

    def show_kinect_image(self, img):
        self.show_image(WritingGui.KINECT_IMAGE_NAME, img, background=True,
                        loc=(0,0), scale=self._bg_scale)

    def show_stroke_image(self, img):
        """All stroke images will be shown one next to each other on the top
        of the screen. The `img` should be of size (self._cdim, self._cdim).
        And since `img` is a stroke image, it is binary, where 0 = background
        and 1 = ink."""
        self._stroke_images.append(img)
        img_display = np.copy(img)
        img_display[img_display==1] = 255
        self.show_image("stroke-%d" % len(self._stroke_images),
                        img_display, loc=(img_display.shape[0]*(self._box_scale/2.0)*(len(self._stroke_images)-1), 0),
                        scale=self._box_scale/2.0, interpolation=cv2.INTER_NEAREST)

    def kinect_image_shown(self):
        return WritingGui.KINECT_IMAGE_NAME in self._images

    def _cond_set_top_left(self, event, x, y):
        return self.kinect_image_shown() is not None \
            and self.last_n_keys(1) == "q"
    def _cond_set_top_right(self, event, x, y):
        return self.kinect_image_shown() is not None \
            and self.last_n_keys(1) == "w"
    def _cond_set_bottom_right(self, event, x, y):
        return self.kinect_image_shown() is not None \
            and self.last_n_keys(1) == "s"
    def _cond_set_bottom_left(self, event, x, y):
        return self.kinect_image_shown() is not None \
            and self.last_n_keys(1) == "a"

    def _store_top_left(self, item):
        x, y, _, _ = self._canvas.coords(item)
        self._top_left = np.array([x/self._bg_scale + WritingGui.POINT_CIRCLE_RADIUS,
                                   y/self._bg_scale + WritingGui.POINT_CIRCLE_RADIUS])
        self._check_and_draw()
    def _store_top_right(self, item):
        x, y, _, _ = self._canvas.coords(item)
        self._top_right = np.array([x/self._bg_scale + WritingGui.POINT_CIRCLE_RADIUS,
                                    y/self._bg_scale + WritingGui.POINT_CIRCLE_RADIUS])
        self._check_and_draw()
    def _store_bottom_left(self, item):
        x, y, _, _ = self._canvas.coords(item)
        self._bottom_left = np.array([x/self._bg_scale + WritingGui.POINT_CIRCLE_RADIUS,
                                      y/self._bg_scale + WritingGui.POINT_CIRCLE_RADIUS])
        self._check_and_draw()
    def _store_bottom_right(self, item):
        x, y, _, _ = self._canvas.coords(item)
        self._bottom_right = np.array([x/self._bg_scale + WritingGui.POINT_CIRCLE_RADIUS,
                                       y/self._bg_scale + WritingGui.POINT_CIRCLE_RADIUS])
        self._check_and_draw()
        
    def _draw_dash(self, name, start, end, color=(102, 204, 255)):
        """`start` and `end` are points on the original image, which may have been scaled
        for display. So we need to adjust here too when displaying the dashed line"""
        if name in self._items:
            self._canvas.delete(self._items[name])
        self._items[name] = self._canvas.create_line(start[0]*self._bg_scale,
                                                     start[1]*self._bg_scale,
                                                     end[0]*self._bg_scale,
                                                     end[1]*self._bg_scale,
                                                     fill=util.rgb_to_hex(color),
                                                     width=2.0,
                                                     dash=(4,4))

    def _check_and_draw(self):
        if self._top_left is not None and self._top_right is not None:
            self._draw_dash("top_line", self._top_left, self._top_right)
        if self._top_left is not None and self._bottom_left is not None:
            self._draw_dash("left_line", self._top_left, self._bottom_left)
        if self._bottom_left is not None and self._bottom_right is not None:
            self._draw_dash("bottom_line", self._bottom_left, self._bottom_right)
        if self._bottom_right is not None and self._top_right is not None:
            self._draw_dash("right_line", self._bottom_right, self._top_right)
        if self._top_left is not None and self._top_right is not None\
           and self._bottom_left is not None and self._bottom_right is not None:
            img_np, drawn_size = self._remove_perspective_show_result()
            img_char, _ = self._extract_character_show_result(img_np, drawn_size)


    def _remove_perspective_show_result(self):
        h, status = cv2.findHomography(np.array([self._top_left,
                                                 self._top_right,
                                                 self._bottom_right,
                                                 self._bottom_left]),
                                       np.array([[0, 0],
                                                 [self._cdim, 0],
                                                 [self._cdim, self._cdim],
                                                 [0, self._cdim]]))
        img_src = self._images[WritingGui.KINECT_IMAGE_NAME][0]
        img_dst = cv2.warpPerspective(img_src, h, (self._cdim, self._cdim))
        size = self.show_image(WritingGui.KINECT_IMAGE_NAME + "_np", img_dst,
                               loc=(self._width, 0), anchor='ne', scale=self._box_scale)
        return img_dst, size


    def _extract_character_show_result(self, img, size):
        """`img` should have no perspective. `size` is the size that `img` is shown
        on the screen (it's probably different from `img`'s dimensions, which is
        equal to (self._cdim, self._cdim). ) 

        Reference: https://docs.opencv.org/3.2.0/d7/d4d/tutorial_py_thresholding.html.

        The result will be a binary image with 0 = background and 1 = ink."""
        # Perform Otsu thresholding after Gaussian filtering
        img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(img,(5,5),0)
        _, img_th = cv2.threshold(blur, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # Remove corners and invert
        corner_width = int(img_th.shape[0] * 0.15)
        corner_height = int(img_th.shape[1] * 0.15)
        img_th[0:corner_width,
            0:corner_height].fill(1)
        img_th[img_th.shape[0]-corner_width:img_th.shape[0],
            0:corner_height].fill(1)
        img_th[img_th.shape[0]-corner_width:img_th.shape[0],
            img_th.shape[1]-corner_height:img_th.shape[1]].fill(1)
        img_th[0:corner_width,
            img_th.shape[1]-corner_height:img_th.shape[1]].fill(1)
        img_th = 1 - img_th  # invert

        # Display
        img_th_display = np.copy(img_th)
        img_th_display[img_th_display==1] = 255
        img_th_display[img_th_display==0] = 100
        
        if self._writing_character is not None:
            # show the character on top of the currently extracted image. Resize if necessary
            self._writing_character_img = np.zeros((self._cdim, self._cdim))
            for stroke in self._writing_character:
                for p in stroke:
                    x, y = p[0], p[1]
                    # Add 'double' thickness to this stroke (based on z)
                    z = p[2] * 2
                    img_th_display[int(round(y-z)):int(round(y+z)),
                                   int(round(x-z)):int(round(x+z))] = 0
                    
        size = self.show_image("char_extract", img_th_display,
                               loc=(self._width, size[1]), anchor='ne', scale=self._box_scale,
                               interpolation=cv2.INTER_NEAREST)
        return img_th, size

    

    def init(self):
        super(WritingGui, self).init()
        self.register_mouse_click_circle("set_top_left",
                                         self._cond_set_top_left,
                                         radius=WritingGui.POINT_CIRCLE_RADIUS,
                                         color=(255, 80, 80),
                                         clear_previous=True, done_cb=self._store_top_left)

        self.register_mouse_click_circle("set_top_right",
                                         self._cond_set_top_right,
                                         radius=WritingGui.POINT_CIRCLE_RADIUS,
                                         color=(0, 204, 102),
                                         clear_previous=True, done_cb=self._store_top_right)

        self.register_mouse_click_circle("set_bottom_left",
                                         self._cond_set_bottom_left,
                                         radius=WritingGui.POINT_CIRCLE_RADIUS,
                                         color=(255, 255, 20),
                                         clear_previous=True, done_cb=self._store_bottom_left)

        self.register_mouse_click_circle("set_bottom_right",
                                         self._cond_set_bottom_right,
                                         radius=WritingGui.POINT_CIRCLE_RADIUS,
                                         color=(0, 102, 255),
                                         clear_previous=True, done_cb=self._store_bottom_right)


def main():
    FILE = "../../../data/stroke.npy"
    characters = np.load(FILE)
    gui = WritingGui(hd=True)
    gui.init()
    gui.set_writing_character(characters[0])
    kinect = MovoKinectInterface()
    img = kinect.take_picture(hd=True)
    gui.show_kinect_image(img)
    gui.spin()


if __name__ == "__main__":
    main()
