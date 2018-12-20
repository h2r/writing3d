#!/usr/bin/env python
# Uses Moveit! to control MOVO's parts.
#
# Docs:
# http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial.html
#
# See also:
# https://github.com/h2r/ros_reality_bridge/blob/holocontrol_movo/scripts/moveit_movo.py

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import util

class Planner:

    def __init__(self, group_name, visualize_plan=True):

        # Initializing node
        util.info("Initializing moveit commander...")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_%s_planner" % group_name,
                        anonymous=True)

        # interface to the robot
        self._robot = moveit_commander.RobotCommander()

        # interface to the world
        self._scene = moveit_commander.PlanningSceneInterface()

        # interface to the joint group
        self._joint_group = moveit_commander.MoveGroupCommander(group_name)

        # starts an action server

        if visualize_plan:
            self._display_trajectory_publisher = rospy.Publisher(
                '/move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory)
            
            util.warning("Waiting 10 seconds for Rviz...")
            rospy.sleep(10)
                
        self._print_info()
        self.plan()


    def _print_info(self):
        print("============ Reference frame: %s" % self._joint_group.get_end_effector_link())
        print("============ Robot Groups:")
        print(self._robot.get_group_names())
        print("============ Printing robot state")
        print(self._robot.get_current_state())
        print("============")

    def plan(self):
        print "============ Generating plan 1"
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = 1.0
        pose_target.position.x = 0.7
        pose_target.position.y = -0.05
        pose_target.position.z = 1.1
        self._joint_group.set_pose_target(pose_target)

        plan1 = self._joint_group.plan()
        print "============ Waiting while RVIZ displays plan1..."
        rospy.sleep(5)
        
        


if __name__ == "__main__":
    Planner("right_arm")

    rospy.spin()
