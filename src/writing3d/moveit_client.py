#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import util
import yaml

from writing3d.msg import PlanMoveEEAction, PlanMoveEEGoal, PlanMoveEEResult, PlanMoveEEFeedback, \
    ExecMoveitPlanAction, ExecMoveitPlanGoal, ExecMoveitPlanResult, ExecMoveitPlanFeedback, \
    PlanJointSpaceAction, PlanJointSpaceGoal, PlanJointSpaceResult, PlanJointSpaceFeedback
from writing3d.common import ActionType

import argparse


class MoveitClient:

    """SimpleActionClient that feeds goal to the Planner,
    and react accordingly based on the feedback and result."""

    def __init__(self, robot_name="movo"):
        rospy.init_node("moveit_%s_client" % robot_name,
                        anonymous=True)
        self._plan_client = actionlib.SimpleActionClient("moveit_%s_plan" % robot_name,
                                                         PlanMoveEEAction)
        self._js_plan_client = actionlib.SimpleActionClient("moveit_%s_joint_space_plan" % robot_name,
                                                         PlanJointSpaceAction)
        self._exec_client = actionlib.SimpleActionClient("moveit_%s_exec" % robot_name,
                                                         ExecMoveitPlanAction)

        util.info("Waiting for moveit planner server...")
        up = self._plan_client.wait_for_server(timeout=rospy.Duration(10))
        if not up:
            rospy.logerr("Timed out waiting for Moveit Planner")
            sys.exit(1)
        up = self._js_plan_client.wait_for_server(timeout=rospy.Duration(10))
        if not up:
            rospy.logerr("Timed out waiting for Moveit Joint Space Planner")
            sys.exit(1)
        up = self._exec_client.wait_for_server(timeout=rospy.Duration(10))
        if not up:
            rospy.logerr("Timed out waiting for Moveit Executer")
            sys.exit(1)

    def send_goal(self, group_name, pose):
        """"`pose` can either be a Pose() object, or a 4-tuple (x,y,z,w),
        or a list of joint values"""
        joint_space = False
        if type(pose) == tuple:
            x, y, z, w = pose
            pose_target = geometry_msgs.msg.Pose()
            pose_target.position.x = x
            pose_target.position.y = y
            pose_target.position.z = z
            pose_target.orientation.w = w
        elif type(pose) == list:
            joint_space = True
        else:
            pose_target = pose
        if not joint_space:
            goal = PlanMoveEEGoal()
            goal.group_name = group_name
            goal.pose = pose_target
            util.info("Client sending goal [%s, %s]" % (group_name, pose))
            self._plan_client.send_goal(goal)
            self._plan_client.wait_for_result(rospy.Duration.from_sec(5.0))
        else:
            goal = PlanJointSpaceGoal()
            goal.joint_values = pose
            goal.group_name = group_name
            util.info("Client sending goal [%s, %s]" % (group_name, pose))
            self._js_plan_client.send_goal(goal)
            self._js_plan_client.wait_for_result(rospy.Duration.from_sec(5.0))
            

    def execute_plan(self, group_name, wait=True):
        util.info("Executing plan for %s" % group_name)
        goal = ExecMoveitPlanGoal()
        goal.wait = wait
        goal.action = ActionType.EXECUTE
        goal.group_name = group_name
        self._exec_client.send_goal(goal)
        self._exec_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def cancel_plan(self, group_name, wait=True):
        util.warning("Canceling plan for %s" % group_name)
        goal = ExecMoveitPlanGoal()
        goal.wait = wait
        goal.action = ActionType.CANCEL
        goal.group_name = group_name
        self._exec_client.send_goal(goal)
        self._exec_client.wait_for_result(rospy.Duration.from_sec(5.0))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Movo Moveit Client. Priority (-g > -f > -e > -k)')
    parser.add_argument('group_name', type=str, help="Group name that the client wants to talk to")
    parser.add_argument('-g', '--goal', type=float, nargs='+',
                        help='Plans goal, specified as a list of floats x y z w, or a list of joint values'\
                        '(more than 4 elements).')
    parser.add_argument('-f', '--goal-file', type=str,
                        help='Path to a yaml file that contains a goal, specified as a list of floats x y z w'\
                        ', or a list of joint values (more than 4 elements).')
    parser.add_argument('-e', '--exe', help='Execute the plan.', action="store_true")
    parser.add_argument('-k', '--cancel', help='Cancel the plan.', action="store_true")
    args = parser.parse_args()

    client = MoveitClient()

    if args.goal:
        goal = args.goal
        if len(goal) == 4:
            goal = tuple(goal)
        client.send_goal(args.group_name, goal)

    if args.goal_file:
        with open(args.goal_file) as f:
            goal = yaml.load(f)
        if len(goal) == 4:
            goal = tuple(goal)
        client.send_goal(args.group_name, goal)
    
    elif args.exe:
        client.execute_plan(args.group_name)

    elif args.cancel:
        client.cancel_plan(args.group_name)

    # Example run:
    # ./moveit_client right_arm -g 0.7 -0.05 1.1 1.0
