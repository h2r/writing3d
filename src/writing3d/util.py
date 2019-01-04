import math

# Colors on terminal https://stackoverflow.com/a/287944/2893053
class bcolors:
    WHITE = '\033[97m'
    CYAN = '\033[96m'
    MAGENTA = '\033[95m'
    BLUE = '\033[94m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BOLD = '\033[1m'
    ENDC = '\033[0m'

    @staticmethod
    def disable():
        bcolors.WHITE   = ''
        bcolors.CYAN    = ''
        bcolors.MAGENTA = ''
        bcolors.BLUE    = ''
        bcolors.GREEN   = ''
        bcolors.YELLOW  = ''
        bcolors.RED     = ''
        bcolors.ENDC    = ''
        
    @staticmethod
    def s(color, content):
        """Returns a string with color when shown on terminal.
        `color` is a constant in `bcolors` class."""
        return color + content + bcolors.ENDC

# String with colors
def info(text):
    print(bcolors.s(bcolors.CYAN, text))

def error(text):
    print(bcolors.s(bcolors.RED, text))

def warning(text):
    print(bcolors.s(bcolors.YELLOW, text))

def success(text):
    print(bcolors.s(bcolors.GREEN, text))
    

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
    # result may be off-by-1
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
