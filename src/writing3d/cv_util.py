# computer vision utilities
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
# author: Kaiyu Zheng


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
        self._shapes = {}

    
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
    `cond_func(event, x, y)` evaluates to True. The shapes (or intermediate
    objects necessary) drawn as a result of the callback will be stored in
    self._shapes[event_name]. `done_cb` is called when the shape is drawn."""
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
                param['done_cb']()

            
            
    def register_mouse_click_line_segment(self, event_name, cond_func, button_num='1',
                                          width=1.0, dash=None, loc_func=None, color=(255, 0, 0),
                                          clear_previous=False, done_cb=None):
        """
        Draw a line segment with two mouse clicks. The underlying event callback
        is for mouse click. So it is up to the user to handle the "first point"
        and "second point" clicks (the cond_func will be called twice, on each
        click, as a signal).

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
                param['done_cb']()
