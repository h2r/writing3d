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
import shape_msgs.msg
import actionlib
import util

from writing3d.msg import PlanMoveEEAction, PlanMoveEEGoal, PlanMoveEEResult, PlanMoveEEFeedback, \
    ExecMoveitPlanAction, ExecMoveitPlanGoal, ExecMoveitPlanResult, ExecMoveitPlanFeedback, \
    PlanJointSpaceAction, PlanJointSpaceGoal, PlanJointSpaceResult, PlanJointSpaceFeedback

from writing3d.common import ActionType

import argparse
    

class MoveitPlanner:

    """SimpleActionServer that takes care of planning."""

    # Unimplemented: it is possible to use MoveIt to specify a list of waypoints for the end effector
    #                to go through. This way you can visualize the entire plan and see if there is any
    #                weird motion. The usefulness of this depends on whether the writing plan is decided
    #                altogether before actually writing.

    def __init__(self, group_names, visualize_plan=True, robot_name="movo"):

        # Initializing node
        util.info("Initializing moveit commander...")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("moveit_%s_planner" % robot_name,
                        anonymous=True)

        # interface to the robot, world, and joint group
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._joint_groups = {n:moveit_commander.MoveGroupCommander(n)
                              for n in group_names}

        # Set the planner to be used. Reference: https://github.com/ros-planning/moveit/issues/236
        for n in self._joint_groups:
            self._joint_groups[n].set_planner_id("RRTConnectkConfigDefault")

        # starts an action server
        util.info("Starting moveit_planner_server...")
        self._plan_server = actionlib.SimpleActionServer("moveit_%s_plan" % robot_name,
                                                         PlanMoveEEAction, self.plan, auto_start=False)
        self._js_plan_server = actionlib.SimpleActionServer("moveit_%s_joint_space_plan" % robot_name,
                                                            PlanJointSpaceAction, self.plan_joint_space, auto_start=False)
        self._exec_server = actionlib.SimpleActionServer("moveit_%s_exec" % robot_name,
                                                         ExecMoveitPlanAction, self.execute, auto_start=False)
        self._plan_server.start()
        self._js_plan_server.start()
        self._exec_server.start()

        self._current_plan = None
        self._current_goal = None

        # Print current joint positions
        for n in self._joint_groups:
            util.info("Joint values for %s" % n)
            util.info("    " + str(self._joint_groups[n].get_current_joint_values()))

        if visualize_plan:
            self._display_trajectory_publisher = rospy.Publisher(
                '/move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory)


    def __del__(self):
        moveit_commander.roscpp_shutdown()

    
    def plan(self, goal):
        if self._current_goal is not None:
            rospy.logwarn("Previous goal exists. Clear it first.")
            return
        self._current_goal = goal
        group_name = goal.group_name
        pose_target = goal.pose
        util.info("Generating plan for goal [%s to %s]" % (group_name, pose_target))

        result = PlanMoveEEResult()
        self._joint_groups[group_name].set_pose_target(pose_target)
        self._current_plan = self._joint_groups[group_name].plan()
        if len(self._current_plan.joint_trajectory.points) > 0:
            util.success("A plan has been made. See it in RViz [check Show Trail and Show Collisions]")
            result.status = 0
        else:
            result.status = 1
        self._plan_server.set_succeeded(result)

    def plan_joint_space(self, goal):
        if self._current_goal is not None:
            rospy.logwarn("Previous goal exists. Clear it first.")
            return
        self._current_goal = goal
        group_name = goal.group_name
        util.info("Generating joint space plan [%s to %s]" % (group_name, goal.joint_values))
        
        result = PlanMoveEEResult()
        self._joint_groups[group_name].set_joint_value_target(goal.joint_values)
        self._current_plan = self._joint_groups[group_name].plan()
        if len(self._current_plan.joint_trajectory.points) > 0:
            util.success("A plan has been made. See it in RViz [check Show Trail and Show Collisions]")
            result.status = 0
        else:
            result.status = 1
        self._plan_server.set_succeeded(result)

    def execute(self, goal):
        group_name = goal.group_name
        util.info("Received executive action from client [type = %d]" % goal.action)

        result = ExecMoveitPlanResult()
        if goal.action == ActionType.EXECUTE:
            success = self._joint_groups[group_name].go(wait=goal.wait)
            if success:
                util.success("Plan for %s will execute." % group_name)
            else:
                util.error("Plan for %s will NOT execute. Is there a collision?" % group_name)
            result.status = 0
                
        elif goal.action == ActionType.CANCEL:
            self._joint_groups[group_name].clear_pose_targets()
            util.success("Plan for %s has been canceled" % group_name)
            self._current_plan = None
            self._current_goal = None
            result.status = 0

        else:
            result.status = 1
            self._exec_server.set_succeeded(result)
            raise ValueError("Unrecognized action type %d" % goal.action)
        
        self._exec_server.set_succeeded(result)
        


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Movo Moveit Client. Priority (-g > -e > -k)')
    parser.add_argument('group_names', type=str, nargs="+", help="Group name(s) that the client wants to talk to")
    args = parser.parse_args()
    
    MoveitPlanner(args.group_names)
    rospy.spin()
