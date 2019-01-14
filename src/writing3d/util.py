import math
import writing3d.common as common
import numpy as np
import copy

# Colors on terminal https://stackoverflow.com/a/287944/2893053
class bcolors:
    # source: https://godoc.org/github.com/whitedevops/colors

    ResetAll = "\033[0m"

    Bold       = "\033[1m"
    Dim        = "\033[2m"
    Underlined = "\033[4m"
    Blink      = "\033[5m"
    Reverse    = "\033[7m"
    Hidden     = "\033[8m"

    ResetBold       = "\033[21m"
    ResetDim        = "\033[22m"
    ResetUnderlined = "\033[24m"
    ResetBlink      = "\033[25m"
    ResetReverse    = "\033[27m"
    ResetHidden     = "\033[28m"

    Default      = "\033[39m"
    Black        = "\033[30m"
    Red          = "\033[31m"
    Green        = "\033[32m"
    Yellow       = "\033[33m"
    Blue         = "\033[34m"
    Magenta      = "\033[35m"
    Cyan         = "\033[36m"
    LightGray    = "\033[37m"
    DarkGray     = "\033[90m"
    LightRed     = "\033[91m"
    LightGreen   = "\033[92m"
    LightYellow  = "\033[93m"
    LightBlue    = "\033[94m"
    LightMagenta = "\033[95m"
    LightCyan    = "\033[96m"
    White        = "\033[97m"

    BackgroundDefault      = "\033[49m"
    BackgroundBlack        = "\033[40m"
    BackgroundRed          = "\033[41m"
    BackgroundGreen        = "\033[42m"
    BackgroundYellow       = "\033[43m"
    BackgroundBlue         = "\033[44m"
    BackgroundMagenta      = "\033[45m"
    BackgroundCyan         = "\033[46m"
    BackgroundLightGray    = "\033[47m"
    BackgroundDarkGray     = "\033[100m"
    BackgroundLightRed     = "\033[101m"
    BackgroundLightGreen   = "\033[102m"
    BackgroundLightYellow  = "\033[103m"
    BackgroundLightBlue    = "\033[104m"
    BackgroundLightMagenta = "\033[105m"
    BackgroundLightCyan    = "\033[106m"
    BackgroundWhite        = "\033[107m"

    DISABLED = False
        
    @staticmethod
    def s(color, content, bold=False):
        """Returns a string with color when shown on terminal.
        `color` is a constant in `bcolors` class."""
        if bcolors.DISABLED:
            return content
        else:
            bold = bcolors.Bold if bold else ""
            return bold + color + content + bcolors.ResetAll

# String with colors
def info(text, debug_level=0, bold=False):
    if common.DEBUG_LEVEL >= debug_level:
        print(bcolors.s(bcolors.Cyan, text, bold=bold))

def info2(text, debug_level=1):
    # Used by default for debugging.
    if common.DEBUG_LEVEL >= debug_level:
        print(bcolors.s(bcolors.LightMagenta, text))

def error(text, debug_level=0):
    if common.DEBUG_LEVEL >= debug_level:
        print(bcolors.s(bcolors.Red, text))

def warning(text, debug_level=0):
    if common.DEBUG_LEVEL >= debug_level:
        print(bcolors.s(bcolors.Yellow, text))

def success(text, debug_level=0):
    if common.DEBUG_LEVEL >= debug_level:
        print(bcolors.s(bcolors.Green, text))

def rgb_to_hex(rgb):
    r,g,b = rgb
    return '#%02x%02x%02x' % (int(r), int(g), int(b))

def hex_to_rgb(hx):
    """hx is a string, begins with #. ASSUME len(hx)=7."""
    if len(hx) != 7:
        raise ValueError("Hex must be #------")
    hx = hx[1:]  # omit the '#'
    r = int('0x'+hx[:2], 16)
    g = int('0x'+hx[2:4], 16)
    b = int('0x'+hx[4:6], 16)
    return (r,g,b)

# Printing
def print_banner(text, ch='=', length=78, color=None):
    """Source: http://code.activestate.com/recipes/306863-printing-a-bannertitle-line/"""
    spaced_text = ' %s ' % text
    banner = spaced_text.center(length, ch)
    if color is None:
        print(banner)
    else:
        print(bcolors.s(color, text))


def print_in_box(msgs, ho="=", vr="||", color=None):
    max_len = 0
    for msg in msgs:
        max_len = max(len(msg), max_len)
    if color is None:
        print(ho*(max_len+2*(len(vr)+1)))
    else:
        print(bcolors.s(color, ho*(max_len+2*(len(vr)+1))))
    for msg in msgs:
        if color is None:
            print(vr + " " + msg + " " + vr)
        else:
            print(bcolors.s(color, vr + " " + msg + " " + vr))
    if color is None:
        print(ho*(max_len+2*(len(vr)+1)))
    else:
        print(bcolors.s(color, ho*(max_len+2*(len(vr)+1))))
    
# Data
def downsample(arr1d, final_len):
    if len(arr1d) < final_len:
        return arr1d
    result = [arr1d[i] for i in range(len(arr1d))
              if i % (len(arr1d) / final_len) == 0]
    return result

# Geometry
def pose_close(p1, p2, r=0.005):
    """
    p1 and p2 are geometry_msgs/Pose objects. Returns true
    if their positions are within the given radius.
    """
    dist = math.sqrt((p1.position.x - p2.position.x)**2
                     + (p1.position.y - p2.position.y)**2
                     + (p1.position.z - p2.position.z)**2)
    return dist <= 0.005

# A vector is a tuple or array of two points (start, end), each
# is N-dimensional np.array.
def unit_vector(v):
    """Return a vector that is a unit vector in the direction
    of `v`. Does not change origin."""
    return np.array([v[0], v[0] + (v[1] - v[0]) / np.linalg.norm(v[1] - v[0])])

def point_along(p, v2, t=1):
    """returns the end point of moving from p along the direction
    of v2 for t times the magnitutde of v2."""
    return p + (v2[1] - v2[0]) * t
