# Defines parameters for various kinds of pens.
#
# Remember: where the pen is attached to the end effector is very important.
# Make sure that when the end effector rotates to reach a altitude angle, the
# gripper does not collide with the surface and that the pen tip can touch the
# surface.
#
# TODO: It is possible to calculate the ee's movement in z direction when the
# pen is rotated but that would require some fine-tuned parameters such as
# the distance from the center of rotation to the surface, which is time-consuming
# to obtain and easy to break.
import math
from pprint import pprint
import writing3d.common as common

# Generic pen
class Pen:
    CONFIG = {
        "RESOLUTION": 0.0002,
        "Z_RESOLUTION": 0.0002,
        "Z_MIN": -0.05,  # This depends on the size of the pen
        "Z_MAX": 0.05,
        "Z_LIFT": 0.05,
    }

    @classmethod
    def name(cls):
        return "pen"

    @classmethod
    def touch_pose(cls):
        return None

    @classmethod
    def retract_pose(cls):
        return None
    
    @classmethod
    def param(cls, name):
        return cls.CONFIG.get(name, None)
    
    @classmethod
    def get_config(cls):
        return cls.CONFIG

    @classmethod
    def print_config(cls):
        print("%s:" % cls.__name__)
        pprint(cls.CONFIG)

class SmallBrush(Pen):
    CONFIG = {
        "RESOLUTION": 0.0002,
        "Z_RESOLUTION": 0.0003,
        "Z_MIN": -0.03,  # This depends on the size of the pen
        "Z_MAX": 0.03,
        "Z_LIFT": 0.03,
    }
    
    @classmethod
    def name(cls):
        return "small_brush"

    @classmethod
    def touch_pose(cls):
        return common.goal_file("touch_joint_pose_brush_small")

    @classmethod
    def retract_pose(cls):
        return common.goal_file("retract_joint_pose_brush_small")

class Sharpe:
    CONFIG = {
        "RESOLUTION": 0.0002,
        "Z_LIFT": 0.03,
    }
    
    @classmethod
    def name(cls):
        return "sharpe"

    @classmethod
    def touch_pose(cls):
        return common.goal_file("touch_joint_pose_sharpe")

    @classmethod
    def retract_pose(cls):
        return common.goal_file("retract_joint_pose_sharpe")


ALL_PENS = [Pen, SmallBrush, Sharpe]

def str_to_pen(string):
    for pen in ALL_PENS:
        if string.lower() == pen.name():
            return pen
    raise ValueError("Pen %s is currently not supported.")
        

