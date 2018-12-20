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

from writing3d.msg import PlanMoveEEAction, PlanMoveEEGoal, PlanMoveEEResult, PlanMoveEEFeedback, \
    ExecMoveEEAction, ExecMoveEEGoal, ExecMoveEEResult, ExecMoveEEFeedback
    

class MoveitPlanner:

    """SimpleActionServer that takes care of planning."""

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

        # starts an action server
        util.info("Starting moveit_planner_server...")
        self._plan_server = actionlib.SimpleActionServer("moveit_%s_plan" % robot_name,
                                                         PlanMoveEEAction, self.plan, auto_start=False)
        self._exec_server = actionlib.SimpleActionServer("moveit_%s_exec" % robot_name,
                                                         ExecMoveEEAction, self.execute, auto_start=False)
        self._plan_server.start()
        self._exec_server.start()

        self._current_plan = None
        self._current_goal = None

        if visualize_plan:
            self._display_trajectory_publisher = rospy.Publisher(
                '/move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory)

    
    def plan(self, goal):
        self._current_goal = goal
        group_name = goal.group_name
        pose_target = goal.pose
        util.info("Generating plan for goal [%s to %s]" % (group_name, pose_target))

        result = PlanMoveEEResult()
        self._joint_groups[group_name].set_pose_target(pose_target)
        self._current_plan = self._joint_groups[group_name].plan()
        util.success("A plan has been made. See it in RViz [check Show Trail]")
        result.status = 0
        self._plan_server.set_succeeded(result)

    def execute(self, goal):
        pass
        
        
        


if __name__ == "__main__":
    MoveitPlanner(["right_arm"])
    rospy.spin()
