#!/usr/bin/env python

import sys
import rospy
import util
import copy
import numpy as np
from writing3d.moveit_client import MoveitClient
from writing3d.moveit_planner import MoveitPlanner
from actionlib import SimpleGoalState

class StrokeWriter:

    """Writes one stroke"""

    class Status:
        NOT_STARTED = 0
        DRAWING = 1
        COMPLETED = 2

    def __init__(self, stroke, dimension=500, resolution=0.000001,
                 robot_name="movo", arm="right_arm"):
        """
        stroke (np.array) array of waypoints (x, y, z, z2, altitude, azimuth)
        dimension (int) dimension of a character's image (default. 500 pixles)
        resolution (float) metric length for one pixel (default. 1cm)
        """
        self._stroke = stroke
        self._dimension = dimension
        self._resolution = resolution
        self._waypoints = None
        self._draw_indx = 0  # for drawing method "separate"
        self._arm = arm
        self._client = MoveitClient(robot_name)
        self._client.get_state(arm, self._prepare_waypoints)
        self._status = StrokeWriter.Status.NOT_STARTED
        try:
            sys.stdout.write("Preparing wapoints...")
            while self._waypoints is None:
                sys.stdout.write(".")
                rospy.sleep(1.0)
        except rospy.ROSInternalException:
            print("Interrupt...")
            return

    def _prepare_waypoints(self, goal_status, result):
        """
        Treat the current pose as corresponding to the "origin" of the character
        image. map stroke from image space to world space.
        """
        # TODO: Now it's only 2D
        util.success("Got state!")
        if goal_status >= SimpleGoalState.DONE:
            state = result
            current_pose = copy.deepcopy(state.pose)
            waypoints = []  # list of poses
            for x, y, z, z2, al, az in self._stroke:
                # Map from image space to world space
                wx = -y * self._resolution
                wy = -x * self._resolution
                print(wx, wy)
                current_pose.position.x += wx
                current_pose.position.y += wy
                waypoints.append(copy.deepcopy(current_pose))
            self._waypoints = waypoints
            util.info(str(self._waypoints[0].position))
            util.info(str(self._waypoints[-1].position))

    def _draw(self, method="together", done_cb=None, wait_time=10.0):
        """
        method can be "together" or "separate".

        If "together", the parameter "done_cb" is called when stroke is finished.
        If "separate", done_cb won't be called. You need to call "done" to check
           if the drawing is finished.
        """
        def plan(status, result):
            if result.status == MoveitPlanner.Status.SUCCESS:
                if self._draw_indx >= len(self._waypoints):
                    return
                else:
                    util.info("Sending goal [%d]" % (self._draw_indx))
                    self._client.send_goal(self._arm, self._waypoints[self._draw_indx],
                                           done_cb=go)
                    self._draw_indx += 1
            else:
                util.error("Oops. Something went wrong :(")
                return SimpleGoalState.DONE

        
        def go(status, result):
            if result.status == MoveitPlanner.Status.SUCCESS:
                self._client.execute_plan(self._arm, done_cb=plan)
            else:
                util.error("Oops. Something went wrong :(")
                return SimpleGoalState.DONE

        # start planning
        if self._waypoints is None:
            return

        self._status = StrokeWriter.Status.DRAWING
        if method == "together":
            self._waypoints = [self._waypoints[0], self._waypoints[-1]]
            self._client.send_goal(self._arm, self._waypoints, done_cb=done_cb)
        elif method == "separate":
            self._waypoints = [self._waypoints[0], self._waypoints[-1]]
            self._client.send_goal(self._arm, self._waypoints[0], done_cb=go)
            self._draw_indx = 1
        util.info("Waiting to draw...")
        rospy.sleep(wait_time)

    def _execute_waypoints_plan(self, status, result):
        if result.status == MoveitPlanner.Status.SUCCESS:
            print("great! Execute now...")
            self._client.execute_plan(self._arm)
            rospy.sleep(3)
        else:
            print("Oops. Something went wrong :(")

    def done(self):
        """Returns true if done drawing"""
        return self._draw_indx >= len(self._waypoints)

    def draw_by_waypoints(self, wait_time=10.0):
        self._draw(method="together", done_cb=self._execute_waypoints_plan,
                   wait_time=wait_time)

    def draw_incrementally(self, wait_time=10.0):
        self._draw(method="separate", done_cb=None,
                   wait_time=wait_time)



if __name__ == "__main__":
    # Ad-hoc
    FILE = "../../data/stroke.npy"
    stroke = np.load(FILE)[0][0]
    print(stroke)
    try:
        util.info("Starting stroke writer...")
        writer = StrokeWriter(stroke)

        # writer.draw_incrementally(wait_time=10.0)
        writer.draw_by_waypoints(wait_time=10.0)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
