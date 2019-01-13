# computer vision utilities
#
# If ImageTk not found:
#   sudo apt-get install python-imaging-tk
#
# References: https://stackoverflow.com/questions/19030579/tkinter-not-found
#
# On Python 2, import Tkinter. On python 3, import tkinter


import cv2
import writing3d.util as util
import Tkinter as tk
import Image, ImageTk


class TkGui(object):

    def __init__(self):
        self._root = None
        self._canvas = None
        self._current_img = None  # current image (np.array)
        self._current_img_tk = None  # current image (PhotoImage)
        self._last_10_keys = []

    
    def init(self):
        self._root = tk.Tk()
        self._canvas = tk.Canvas(self._root, width=20,
                                 height=20)
        self._canvas.config(bg='black')
        # pack the canvas into a frame/form
        self._canvas.pack(fill=tk.BOTH)
        self._root.bind("<Key>", self._key_pressed)

    def show_image(self, img):
        """
        Given an image `img` as np.array (RGB), display the image on Tk window
        """
        self._current_img = img
        self._current_img_tk = ImageTk.PhotoImage(image=Image.fromarray(self._current_img))
        self._canvas.create_image(0, 0, anchor='nw', image=self._current_img_tk)
        self._canvas.config(width=self._current_img.shape[1],
                            height=self._current_img.shape[0])

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
        util.info("Pressed %s" % repr(event.char), debug_level=1)
        self._last_10_keys.append(event.char)
        util.info2(str(self._last_10_keys), debug_level=2)
            

    """For all register callback functions,  it will be called if
    `cond_func(event, x, y)` evaluates to True"""
    def register_mouse_click_circle(self, cond_func, button_num='1',
                                    radius=50, color=(255, 0, 0),
                                    thickness=-1, loc_func=None):
        """Draw a circle at the location of mouse click, or some other
        specified location, determined by `loc_func` (if not None):
            loc_func(mouse_x, mouse_y)  --> (x, y)

        button_num can be '1', '2', or '3' (left, middle, right buttons). 
        """
        util.info2("Registering mouse click circle", debug_level=1)
        param = {
            'radius': radius,
            'color': util.rgb_to_hex(color),
            'thickness': thickness,
            'loc_func': loc_func,
            'cond_func': cond_func
        }
        self._canvas.bind("<Button-%s>" % button_num,
                          lambda event: self._mouse_click_circle(event, param))

    def _mouse_click_circle(self, event, param):
        if self._current_img is None:
            return
        x, y = event.x, event.y
        if param['cond_func'](event, x, y):
            if param['loc_func'] is not None:
                x, y = param['loc_func'](x, y)

            r = param['radius']
            self._canvas.create_oval(x-r, y-r, x+r, y+r, fill=param['color'],
                                     outline="")
            





# class GUI:

#     """GUI based on TKinter. Drawing is done using OpenCV."""

#     def __init__(self):
#         self._current_img = None  # This image is a
#         self._last_10_keys = []

#         self._config = {}

#     def set_image(self, img):
#         """`img` is an opencv image (i.e. numpy array)"""
#         self._current_img = img

#     @property
#     def current_image(self):
#         return self._current_img

#     """For all register callback functions,  it will be called if
#     `cond_func(event, x, y)` evaluates to True"""
#     def register_mouse_click_circle(self, winname, cond_func,
#                                     radius=50, color=(255, 0, 0),
#                                     thickness=-1, loc_func=None):
#         """Draw a circle at the location of mouse click, or some other
#         specified location, determined by `loc_func` (if not None):
#             loc_func(mouse_x, mouse_y)  --> (x, y)
#         """
#         util.info2("Registering mouse click circle", debug_level=1)
#         cv2.setMouseCallback(winname, self._mouse_click_circle,
#                              {
#                                  'radius': radius,
#                                  'color': color,
#                                  'thickness': thickness,
#                                  'loc_func': loc_func,
#                                  'cond_func': cond_func
                                 
#                              })

#     def _mouse_click_circle(self, event, x, y, flags, param):
#         if self._current_img is None:
#             return
#         if param['cond_func'](event, x, y):
#             if param['loc_func'] is not None:
#                 x, y = param['loc_func'](x, y)
#             cv2.circle(self._current_img, (x, y),
#                        param['radius'], param['color'], param['thickness'])


#     def last_n_keys(self, n=1):
#         """Return a list of n keys that were most recently pressed. Most-recent first."""
#         if n <= 0 or n > len(self._last_10_keys):
#             util.warning("Do not know key! (got: %d; max_length: %d)"
#                          % (n, len(self._last_10_keys)))
#             return None
#         return list(reversed(self._last_10_keys[-n:-1]))

