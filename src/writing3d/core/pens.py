#!/usr/bin/env python
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
import rospy
import tf
import math
import argparse
from pprint import pprint
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import writing3d.common as common
import writing3d.util as util

# Generic pen
class Pen:
    CONFIG = {
        "RESOLUTION": 0.0002,
        "Z_RESOLUTION": 0.0002,
        "Z_MIN": -0.05,  # This depends on the size of the pen
        "Z_MAX": 0.05,
        "Z_LIFT": 0.05,
        "PEN_TIP_TF": [(0.0, 0.0, 0.0),
                       tuple(quaternion_from_euler(math.radians(0),
                                                   math.radians(0),
                                                   math.radians(0)))]
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

    @classmethod
    def uses_orientation(cls):
        return False

    @classmethod
    def needs_dip(cls):
        return False

class SmallBrush(Pen):
    CONFIG = {
        "RESOLUTION": 0.0004,
        "Z_RESOLUTION": 0.003,
        "Z_MIN": -0.15,  # This depends on the size of the pen
        "Z_MAX": -0.004,
        "Z_LIFT": 0.05,
        "PEN_TIP_TF": [(0.0, 0.0, 0.0),
                       tuple(quaternion_from_euler(math.radians(0),
                                                   math.radians(0),
                                                   math.radians(0)))]                
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

    @classmethod
    def uses_orientation(cls):
        return False

    @classmethod
    def needs_dip(cls):
        return True



class Sharpe(Pen):
    # Move torso to 0.1115, head tilt to -0.6
    
    CONFIG = {
        "RESOLUTION": 0.0002,
        "Z_RESOLUTION": 0.008,
        "Z_MIN": -0.012,  # This depends on the size of the pen
        "Z_MAX": -0.007,
        "Z_LIFT": 0.03,
        "O_INI": (0.0, 90.0, 45.0), # when the pen is prependicular to the paper
        "O_REST": (38.0, 44.6, 45.0), # when holding the pen at resting pose
        "AZ_FACTOR": -1,   # direction the angle should be applied
        "AL_FACTOR": -1,   # to the robot.
        "AZ_I": 0,  # index for az
        "AL_I": 1,  # index for al
        "PEN_TIP_TF": [(0.15, 0.0, 0.0),
                       tuple(quaternion_from_euler(math.radians(0),
                                                   math.radians(0),
                                                   math.radians(0)))]        
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

    @classmethod
    def uses_orientation(cls):
        return True
    
    @classmethod
    def needs_dip(cls):
        return False


ALL_PENS = [Pen, SmallBrush, Sharpe]

def str_to_pen(string):
    for pen in ALL_PENS:
        if string.lower() == pen.name():
            return pen
    raise ValueError("Pen %s is currently not supported.")


####### TF publisher ########        
class PubPenTipTf:
    def __init__(self, pen, ee_frame):
        self._ee_frame = ee_frame
        self._pen = pen

    def publish_transform(self, ee_frame):
        br = tf.TransformBroadcaster()
        try:
            rate = rospy.Rate(60)
            while not rospy.is_shutdown():
                br.sendTransform(self._pen.CONFIG['PEN_TIP_TF'][0],
                                 self._pen.CONFIG['PEN_TIP_TF'][1],
                                 rospy.Time.now(),
                                 "pen_tip",
                                 ee_frame)
                rate.sleep()
        except KeyboardInterrupt:
            return

    def run(self):
        util.warning("Publishing tf transform for pen tip (from %s to %s)"\
                     % (self._ee_frame, "pen_tip"))
        self.publish_transform(self._ee_frame)

def main():
    parser = argparse.ArgumentParser(description='Movo Moveit Planner.')
    parser.add_argument("pen", type=str, help="Type of pen to use. See pens.py")
    parser.add_argument("ee_frame", type=str, help="EE link which holds the pen (e.g. right_ee_link)")
    args = parser.parse_args()

    rospy.init_node("pen_tip_tf_pub",
                    anonymous=True, disable_signals=True)
    
    pen = str_to_pen(args.pen)
    ptf = PubPenTipTf(pen, args.ee_frame)
    ptf.run()

        
if __name__ == "__main__":
    main()
