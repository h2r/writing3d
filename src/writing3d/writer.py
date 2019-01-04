#!/usr/bin/env python

import matplotlib.pyplot as plt
import sys
import rospy
import util
import copy
import numpy as np
from writing3d.moveit_client import MoveitClient
from writing3d.moveit_planner import MoveitPlanner
from actionlib import SimpleGoalState

RESOLUTION = 0.0001

class StrokeWriter:

    """Writes one stroke"""

    class Status:
        NOT_STARTED = 0
        DRAWING = 1
        COMPLETED = 2

    def __init__(self, stroke, client,
                 dimension=500, resolution=RESOLUTION,
                 robot_name="movo", arm="right_arm",
                 origin_pose=None, num_waypoints=5):
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
        self._origin_pose = origin_pose
        self._num_waypoints = num_waypoints
        self._status = StrokeWriter.Status.NOT_STARTED
        
        self._client = client
        self._client.get_state(arm, self._prepare_waypoints)
        try:
            sys.stdout.write("Preparing wapoints...")
            while self._waypoints is None:
                sys.stdout.write(".")
                rospy.sleep(1.0)
            sys.stdout.write("\n")
        except rospy.ROSInternalException:
            print("Interrupt...")
            return
        
    @property
    def waypoints(self):
        return self._waypoints

    @property
    def origin_pose(self):
        return self._origin_pose

    def visualize(self):
        xvals = []
        yvals = []
        for pose in self._waypoints:
            xvals.append(pose.position.x)
            yvals.append(pose.position.y)
        plt.plot(xvals, yvals)

    def _print_waypoint_stats(self):
        print("-----waypoint stats-----")
        xvals = [p.position.x for p in self._waypoints]
        yvals = [p.position.y for p in self._waypoints]
        zvals = [p.position.z for p in self._waypoints]
        print("Total waypoints: %d" % len(self._waypoints))
        print("x range: %.3f (%.3f ~ %.3f)" % (np.ptp(xvals), np.min(xvals), np.max(xvals)))
        print("y range: %.3f (%.3f ~ %.3f)" % (np.ptp(yvals), np.min(yvals), np.max(yvals)))
        print("z range: %.3f (%.3f ~ %.3f)" % (np.ptp(zvals), np.min(zvals), np.max(zvals)))

    def _prepare_waypoints(self, goal_status, result):
        """
        Treat the current pose as corresponding to the "origin" of the character
        image. map stroke from image space to world space.
        """
        # TODO: Now it's only 2D
        util.success("Got state!")
        if goal_status >= SimpleGoalState.DONE:
            state = result
            if self._origin_pose is None:
                self._origin_pose = copy.deepcopy(state.pose)

            waypoints = []  # list of poses
            for x, y, z, z2, al, az in self._stroke:
                # Map from image space to world space
                current_pose = copy.deepcopy(self._origin_pose)
                wx = -y * self._resolution
                wy = -x * self._resolution
                current_pose.position.x += wx
                current_pose.position.y += wy
                waypoints.append(current_pose)

            # filter waypoints. There are too many
            self._waypoints = util.downsample(waypoints, self._num_waypoints)
            
            # At the end of the stroke, lift the pen.
            last_pose = copy.deepcopy(self._waypoints[-1])
            last_pose.position.z += 0.03
            self._waypoints.append(last_pose)

            # Print stats
            self._print_waypoint_stats()
            

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
                    self._status = StrokeWriter.Status.COMPLETED
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
            self._client.send_goal(self._arm, self._waypoints, done_cb=done_cb)
        elif method == "separate":
            self._client.send_goal(self._arm, self._waypoints[0], done_cb=go)
            self._draw_indx = 1
        util.info("Waiting to draw...")
        rospy.sleep(wait_time)

    def _execute_waypoints_plan(self, status, result):

        def executing(status, result):
            self._client.get_state(self._arm, completed)
        
        def completed(status, result):
            # Mark as completed when the arm actually moved near target pose.
            # Otherwise, keep getting state (sleep for 1 sec).
            if util.pose_close(self._waypoints[-1], result.pose, r=0.005):
                self._status = StrokeWriter.Status.COMPLETED
            else:
                util.info("Expecting arm to move to")
                util.info(str(self._waypoints[-1]))
                util.info("Now at")
                util.info(str(result.pose))
                rospy.sleep(1)
                self._client.get_state(self._arm, completed)
            
        if result.status == MoveitPlanner.Status.SUCCESS:
            print("great! Execute now...")
            self._client.execute_plan(self._arm,
                                      done_cb=executing)
            rospy.sleep(3)
        else:
            print("Oops. Something went wrong :(")

    def done(self):
        """Returns true if done drawing"""
        return self._status == StrokeWriter.Status.COMPLETED

    def draw_by_waypoints(self, wait_time=10.0):
        self._draw(method="together", done_cb=self._execute_waypoints_plan,
                   wait_time=wait_time)

    def draw_incrementally(self, wait_time=10.0):
        self._draw(method="separate", done_cb=None,
                   wait_time=wait_time)


class CharacterWriter:

    def __init__(self, strokes, dimension=500, resolution=RESOLUTION,
                 robot_name="movo", arm="right_arm", num_waypoints=5):
        self._client = MoveitClient(robot_name)
        self._dimension = dimension
        self._resolution = resolution
        self._strokes = strokes
        self._arm = arm
        self._robot_name = robot_name
        self._origin_pose = None
        self._client = MoveitClient(robot_name)

        # Construct writers
        self._writers = []
        for i in range(len(self._strokes)):
            util.info("Initializing writer for stroke %d" % i)
            self._writers.append(StrokeWriter(self._strokes[i], self._client,
                                              dimension=self._dimension, resolution=self._resolution,
                                              robot_name=self._robot_name, arm=self._arm,
                                              num_waypoints=num_waypoints, origin_pose=self._origin_pose))
            self._origin_pose = self._writers[i].origin_pose

        # Print stats
        self._print_waypoint_stats()

    def visualize(self):
        for i in range(len(self._strokes)):
            self._writers[i].visualize()
        plt.show()
                                 
    def _print_waypoint_stats(self):
        print("=========Character waypoint stats========")
        xvals = [p.position.x
                 for w in self._writers
                 for p in w.waypoints]
        yvals = [p.position.y
                 for w in self._writers
                 for p in w.waypoints]
        zvals = [p.position.z
                 for w in self._writers
                 for p in w.waypoints]
        print("Total waypoints: %d" % len(xvals))
        print("x range: %.3f (%.3f ~ %.3f)" % (np.ptp(xvals), np.min(xvals), np.max(xvals)))
        print("y range: %.3f (%.3f ~ %.3f)" % (np.ptp(yvals), np.min(yvals), np.max(yvals)))
        print("z range: %.3f (%.3f ~ %.3f)" % (np.ptp(zvals), np.min(zvals), np.max(zvals)))
        

    def write(self, index=-1, visualize=True, method="together"):
        if index < 0:
            # Draw entire character
            for i in range(len(self._strokes)):
                self.write(i)
        else:
            if index > 0 and self._origin_pose is None:
                util.warning("Origin pose unknown but not writing the first stroke."\
                             "Will treat current pose as origin.")
            # Wait till finish
            sys.stdout.write("Drawing stroke %d..." % index)
            if method == "together":
                self._writers[index].draw_by_waypoints(wait_time=10.0)
            # elif method == "separate":
            #     writer.draw_incrementally(wait_time=10.0)
            while not self._writers[index].done():
                sys.stdout.write(".")
                rospy.sleep(1.0)
            util.success("Done!")


if __name__ == "__main__":
    # Ad-hoc
    FILE = "../../data/stroke.npy"
    characters = np.load(FILE)
    util.info("Starting character writer...")
    writer = CharacterWriter(characters[0])
    util.warning("Begin writing...")
    rospy.sleep(2)
    writer.write()

