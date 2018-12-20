#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import util

from writing3d.msg import PlanMoveEEAction, PlanMoveEEGoal, PlanMoveEEResult, PlanMoveEEFeedback, \
    ExecMoveEEAction, ExecMoveEEGoal, ExecMoveEEResult, ExecMoveEEFeedback


class MoveitClient:

    """SimpleActionClient that feeds goal to the Planner,
    and react accordingly based on the feedback and result."""

    def __init__(self, robot_name="movo"):
        rospy.init_node("moveit_%s_client" % robot_name,
                        anonymous=True)
        self._plan_client = actionlib.SimpleActionClient("moveit_%s_plan" % robot_name,
                                                         PlanMoveEEAction)
        self._exec_client = actionlib.SimpleActionClient("moveit_%s_exec" % robot_name,
                                                         ExecMoveEEAction)

        util.info("Waiting for moveit planner server...")
        up = self._plan_client.wait_for_server(timeout=rospy.Duration(10))
        if not up:
            rospy.logerr("Timed out waiting for Moveit Planner")
            sys.exit(1)
        up = self._exec_client.wait_for_server(timeout=rospy.Duration(10))
        if not up:
            rospy.logerr("Timed out waiting for Moveit Executer")
            sys.exit(1)

    def send_goal(self, group_name, pose):
        """"`pose` can either be a Pose() object, or a 4-tuple (x,y,z,w)."""
        if type(pose) == tuple:
            x, y, z, w = pose
            pose_target = geometry_msgs.msg.Pose()
            pose_target.position.x = x
            pose_target.position.y = y
            pose_target.position.z = z
            pose_target.orientation.w = w
        else:
            pose_target = pose
        goal = PlanMoveEEGoal()
        goal.group_name = group_name
        goal.pose = pose_target
        util.info("Client sending goal [%s, %s]" % (group_name, pose))
        self._plan_client.send_goal(goal)
        self._plan_client.wait_for_result(rospy.Duration.from_sec(5.0))


if __name__ == "__main__":
    client = MoveitClient()
    client.send_goal("right_arm", (0.7, -0.05, 1.1, 1.0))
    
