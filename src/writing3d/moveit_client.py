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
    PlanJointSpaceAction, PlanJointSpaceGoal, PlanJointSpaceResult, PlanJointSpaceFeedback, \
    PlanWaypointsAction, PlanWaypointsGoal, PlanWaypointsResult, PlanWaypointsFeedback,\
    GetStateAction, GetStateGoal, GetStateResult, GetStateFeedback
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
        self._wayp_plan_client = actionlib.SimpleActionClient("moveit_%s_wayp_plan" % robot_name,
                                                              PlanWaypointsAction)
        self._get_state_client = actionlib.SimpleActionClient("moveit_%s_get_state" % robot_name,
                                                              GetStateAction)
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
        up = self._wayp_plan_client.wait_for_server(timeout=rospy.Duration(10))
        if not up:
            rospy.logerr("Timed out waiting for Moveit Waypoints Planner")
            sys.exit(1)
        up = self._exec_client.wait_for_server(timeout=rospy.Duration(10))
        if not up:
            rospy.logerr("Timed out waiting for Moveit Executer")
            sys.exit(1)
        up = self._get_state_client.wait_for_server(timeout=rospy.Duration(10))
        if not up:
            rospy.logerr("Timed out waiting for Moveit GetState server")
            sys.exit(1)

    def send_goal(self, group_name, pose, done_cb=None):
        """"`pose` can either be a Pose, a list of coordinates for
        end effector pose or a list of joint values"""
        #### PlanMoveEE
        if isinstance(pose, geometry_msgs.msg.Pose):
            goal = PlanMoveEEGoal()
            goal.group_name = group_name
            goal.pose = pose
            goal.trans_only = False
            util.info("Client sending goal [%s, %s]" % (group_name, pose))
            self._plan_client.send_goal(goal, done_cb=done_cb)
            self._plan_client.wait_for_result(rospy.Duration.from_sec(5.0))
            
        if type(pose) == tuple:
            trans_only = True
            pose_target = geometry_msgs.msg.Pose()
            pose_target.position.x = pose[0]
            pose_target.position.y = pose[1]
            pose_target.position.z = pose[2]
            if len(pose) > 3:
                pose_target.orientation.x = pose[3]
                pose_target.orientation.y = pose[4]
                pose_target.orientation.z = pose[5]
                pose_target.orientation.w = pose[6]
                trans_only = False
                
            goal = PlanMoveEEGoal()
            goal.group_name = group_name
            goal.pose = pose_target
            goal.trans_only = trans_only
            util.info("Client sending goal [%s, %s]" % (group_name, pose))
            self._plan_client.send_goal(goal, done_cb=done_cb)
            self._plan_client.wait_for_result(rospy.Duration.from_sec(5.0))

        #### PlanWaypoints and PlanJointSpace
        elif type(pose) == list:
            if isinstance(pose[0], geometry_msgs.msg.Point) \
               or isinstance(pose[0], geometry_msgs.msg.Pose):
                goal = PlanWaypointsGoal()
                goal.waypoints = pose
                goal.group_name = group_name
                util.info("Client sending goal [%s, %s]" % (group_name, pose))
                self._wayp_plan_client.send_goal(goal, done_cb=done_cb)
                self._wayp_plan_client.wait_for_result(rospy.Duration.from_sec(5.0))
            else:
                goal = PlanJointSpaceGoal()
                goal.joint_values = pose
                goal.group_name = group_name
                util.info("Client sending goal [%s, %s]" % (group_name, pose))
                self._js_plan_client.send_goal(goal, done_cb=done_cb)
                self._js_plan_client.wait_for_result(rospy.Duration.from_sec(5.0))

        else:
            util.error("pose type not understood. Goal unsent.")
            

    def execute_plan(self, group_name, wait=True, done_cb=None):
        util.info("Executing plan for %s" % group_name)
        goal = ExecMoveitPlanGoal()
        goal.wait = wait
        goal.action = ActionType.EXECUTE
        goal.group_name = group_name
        self._exec_client.send_goal(goal, done_cb=None)
        self._exec_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def cancel_plan(self, group_name, wait=True):
        util.warning("Canceling plan for %s" % group_name)
        goal = ExecMoveitPlanGoal()
        goal.wait = wait
        goal.action = ActionType.CANCEL
        goal.group_name = group_name
        self._exec_client.send_goal(goal)
        self._exec_client.wait_for_result(rospy.Duration.from_sec(5.0))

    def get_state(self, group_name, done_cb, wait_time=10.0):
        goal = GetStateGoal()
        goal.group_name = group_name
        self._get_state_client.send_goal(goal, done_cb=done_cb)
        finished = self._get_state_client.wait_for_result(rospy.Duration.from_sec(wait_time))
        if not finished:
            util.error("Client didn't hear from Server in %s seconds." % str(wait_time))

def parse_waypoints_diffs(points):
    """points is a list of relative differences in x, y, z directions."""
    diffs = []
    for p in points:
        dp = geometry_msgs.msg.Point()
        dp.x = p[0]
        dp.y = p[1]
        dp.z = p[2]
        diffs.append(dp)
    return diffs


def main():
    parser = argparse.ArgumentParser(description='Movo Moveit Client. Priority (-g > -f > -e > -k > --state)')
    parser.add_argument('group_name', type=str, help="Group name that the client wants to talk to")
    parser.add_argument('-g', '--goal', type=float, nargs='+',
                        help='Plans goal, specified as a list of floats (either means end-effector pose,'\
                        ', or a list of joint values')
    parser.add_argument('-f', '--goal-file', type=str,
                        help='Path to a yaml file that contains a goal, specified as a list of floats x y z w'\
                        ', or a list of joint values (more than 4 elements). If it contains multiple points,'\
                        'then they are interpreted as waypoints.')
    parser.add_argument('-e', '--exe', help='Execute the plan.', action="store_true")
    parser.add_argument('-k', '--cancel', help='Cancel the plan.', action="store_true")
    parser.add_argument('--ee', help="goal is end effector pose", action="store_true")
    parser.add_argument('--state', help='Get robot state (joint values and pose)', action="store_true")
    args = parser.parse_args()

    client = MoveitClient()

    if args.goal:
        goal = args.goal
        if args.ee:
            goal = tuple(goal)
        client.send_goal(args.group_name, goal)

    elif args.goal_file:
        with open(args.goal_file) as f:
            goal = yaml.load(f)
        if type(goal[0]) != list:
            # Single goal
            if args.ee:
                goal = tuple(goal)
        else:
            # Multiple goals. We need a list of waypoints
            goal = parse_waypoints_diffs(goal)
        client.send_goal(args.group_name, goal)

    elif args.exe:
        client.execute_plan(args.group_name)

    elif args.cancel:
        client.cancel_plan(args.group_name)

    elif args.state:
        goal = GetStateGoal()
        



if __name__ == "__main__":
    main()
