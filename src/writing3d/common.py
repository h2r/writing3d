import rospkg
import os

class ActionType:
    EXECUTE = 0
    CANCEL = 1
    # PAUSE = 2

_rospack = rospkg.RosPack()
CFG_PATH = _rospack.get_path('writing3d')

def goal_file(name):
    return os.path.join(CFG_PATH, "cfg", "%s.yml" % name)
