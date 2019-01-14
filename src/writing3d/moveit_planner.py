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
import copy
import argparse

from writing3d.msg import PlanMoveEEAction, PlanMoveEEGoal, PlanMoveEEResult, PlanMoveEEFeedback, \
    ExecMoveitPlanAction, ExecMoveitPlanGoal, ExecMoveitPlanResult, ExecMoveitPlanFeedback, \
    PlanJointSpaceAction, PlanJointSpaceGoal, PlanJointSpaceResult, PlanJointSpaceFeedback,\
    PlanWaypointsAction, PlanWaypointsGoal, PlanWaypointsResult, PlanWaypointsFeedback, \
    GetStateAction, GetStateGoal, GetStateResult, GetStateFeedback

import writing3d.common as common


common.DEBUG_LEVEL = 1
    

class MoveitPlanner:

    """SimpleActionServer that takes care of planning."""

    class Status:
        ABORTED = 0
        SUCCESS = 1

    class PlanType:
        CARTESIAN = 1
        JOINT_SPACE = 2
        WAYPOINTS = 3

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
            self._joint_groups[n].set_planner_id("RRTstarkConfigDefault")

        # starts an action server
        util.info("Starting moveit_planner_server...")
        self._plan_server = actionlib.SimpleActionServer("moveit_%s_plan" % robot_name,
                                                         PlanMoveEEAction, self.plan, auto_start=False)
        self._js_plan_server = actionlib.SimpleActionServer("moveit_%s_joint_space_plan" % robot_name,
                                                            PlanJointSpaceAction, self.plan_joint_space, auto_start=False)
        self._wayp_plan_server = actionlib.SimpleActionServer("moveit_%s_wayp_plan" % robot_name,
                                                              PlanWaypointsAction, self.plan_waypoints, auto_start=False)
        self._exec_server = actionlib.SimpleActionServer("moveit_%s_exec" % robot_name,
                                                         ExecMoveitPlanAction, self.execute, auto_start=False)
        self._get_state_server = actionlib.SimpleActionServer("moveit_%s_get_state" % robot_name,
                                                              GetStateAction, self.get_state, auto_start=False)
                
        self._plan_server.start()
        self._js_plan_server.start()
        self._wayp_plan_server.start()
        self._exec_server.start()
        self._get_state_server.start()

        self._current_plan = {n:None for n in group_names}
        self._current_goal = {n:None for n in group_names}
        self._plan_type = None

        # Print current joint positions
        for n in self._joint_groups:
            util.info("Joint values for %s" % n)
            util.info("    " + str(self._joint_groups[n].get_current_joint_values()))
            util.info("Current pose for %s" % n)
            util.info("    " + str(self._joint_groups[n].get_current_pose().pose))

        if visualize_plan:
            self._display_trajectory_publisher = rospy.Publisher(
                '/move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory)

    def __del__(self):
        moveit_commander.roscpp_shutdown()

    def plan(self, goal):
        group_name = goal.group_name
        if self._current_goal[group_name] is not None:
            rospy.logwarn("Previous goal exists. Clear it first.")
            return
        self._plan_type = MoveitPlanner.PlanType.CARTESIAN
        self._current_goal[group_name] = self._joint_groups[group_name].get_current_pose().pose
        self._current_goal[group_name].position = goal.pose.position
        if not goal.trans_only:
            self._current_goal[group_name].orientation = goal.pose.orientation
        util.info("Generating plan for goal [%s to %s]" % (group_name, self._current_goal[group_name]))

        self._joint_groups[group_name].set_pose_target(self._current_goal[group_name])
        self._current_plan[group_name] = self._joint_groups[group_name].plan()
        result = PlanWaypointsResult()
        if len(self._current_plan[group_name].joint_trajectory.points) > 0:
            util.success("A plan has been made. See it in RViz [check Show Trail and Show Collisions]")
            result.status = MoveitPlanner.Status.SUCCESS
            self._plan_server.set_succeeded(result)
        else:
            util.error("No plan found.")
            result.status = MoveitPlanner.Status.ABORTED
            self._plan_server.set_aborted(result)
            

    def plan_joint_space(self, goal):
        util.info2("plan_joint_space")
        group_name = goal.group_name
        if self._current_goal[group_name] is not None:
            rospy.logwarn("Previous goal exists. Clear it first.")
            return
        self._plan_type = MoveitPlanner.PlanType.JOINT_SPACE
        self._current_goal[group_name] = goal
        util.info("Generating joint space plan [%s to %s]" % (group_name, goal.joint_values))

        self._joint_groups[group_name].set_joint_value_target(goal.joint_values)
        self._joint_groups[group_name].set_planning_time(1.0)
        self._current_plan[group_name] = self._joint_groups[group_name].plan()
        self._joint_groups[group_name].set_planning_time(5.0)  # set back to default value
        result = PlanJointSpaceResult()
        if len(self._current_plan[group_name].joint_trajectory.points) > 0:
            util.success("A plan has been made. See it in RViz [check Show Trail and Show Collisions]")
            result.status = MoveitPlanner.Status.SUCCESS
            self._js_plan_server.set_succeeded(result)
        else:
            util.error("No plan found.")
            result.status = MoveitPlanner.Status.ABORTED
            self._js_plan_server.set_aborted(result)

            
    def plan_waypoints(self, goal):
        util.info2("plan_waypoints")
        group_name = goal.group_name
        if self._current_goal[group_name] is not None:
            rospy.logwarn("Previous goal exists. Clear it first.")
            return
        self._plan_type = MoveitPlanner.PlanType.WAYPOINTS
        self._current_goal[group_name] = goal
        util.info("Generating waypoints plan for %s" % (group_name))
        
        current_pose = self._joint_groups[group_name].get_current_pose().pose
        waypoints = goal.waypoints #[current_pose] + 
        self._current_plan[group_name], fraction = self._joint_groups[group_name].compute_cartesian_path(waypoints, 0.01, 0.0)
        result = PlanWaypointsResult()
        if len(self._current_plan[group_name].joint_trajectory.points) > 0:
            util.success("A plan has been made (%d points). See it in RViz [check Show Trail and Show Collisions]"
                         % len(self._current_plan[group_name].joint_trajectory.points))
            result.status = MoveitPlanner.Status.SUCCESS
            self._wayp_plan_server.set_succeeded(result)
        else:
            util.error("No plan found.")
            result.status = MoveitPlanner.Status.ABORTED
            self._wayp_plan_server.set_aborted(result)
        

    def execute(self, goal):
        util.info2("execute")
        group_name = goal.group_name
        util.info("Received executive action from client [type = %d]" % goal.action)

        result = ExecMoveitPlanResult()
        if goal.action == common.ActionType.EXECUTE:
            if self._plan_type == MoveitPlanner.PlanType.WAYPOINTS:
                success = self._joint_groups[group_name].execute(self._current_plan[group_name])
            else:
                success = self._joint_groups[group_name].go(wait=goal.wait)
            if success:
                util.success("Plan for %s will execute." % group_name)
                result.status = MoveitPlanner.Status.SUCCESS
                rospy.sleep(1)
            else:
                util.error("Plan for %s will NOT execute. Is there a collision?" % group_name)
                result.status = MoveitPlanner.Status.ABORTED
            self.cancel_goal(group_name)  # get rid of this goal since we have completed it
            util.info("Now %s is at pose:\n%s" % (group_name,
                                                  self._joint_groups[group_name].get_current_pose().pose))
            self._exec_server.set_succeeded(result)
                
        elif goal.action == common.ActionType.CANCEL:
            self.cancel_goal(group_name)
            result.status = MoveitPlanner.Status.SUCCESS 
            self._exec_server.set_succeeded(result)
            
        else:
            util.error("Unrecognized action type %d" % goal.action)
            result.status = MoveitPlanner.Status.ABORTED
            self._exec_server.set_aborted()

    def get_state(self, goal):
        util.info2("get_state")
        group_name = goal.group_name
        result = GetStateResult()
        # Fill in state attributes
        result.pose = self._joint_groups[group_name].get_current_pose().pose
        result.joint_values = self._joint_groups[group_name].get_current_joint_values()
        result.has_goal = self._current_goal[group_name] is not None
        result.has_plan = self._current_plan[group_name] is not None
        self._get_state_server.set_succeeded(result)

    def cancel_goal(self, group_name):
        self._joint_groups[group_name].clear_pose_targets()
        util.success("Goal for %s has been cleared" % group_name)
        self._current_plan[group_name] = None
        self._current_goal[group_name] = None
            

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Movo Moveit Planner.')
    parser.add_argument('group_names', type=str, nargs="+", help="Group name(s) that the client wants to talk to")
    args = parser.parse_args()
    
    MoveitPlanner(args.group_names)
    rospy.spin()
