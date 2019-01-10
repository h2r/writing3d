#!/usr/bin/env python
#
# Convention: If a public function is named with CamelCase, then it can possibly lead
# to robot motion

import matplotlib.pyplot as plt
import sys
import rospy
import util
import copy
import numpy as np
import yaml
import math
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from writing3d.moveit_client import MoveitClient
from writing3d.moveit_planner import MoveitPlanner
import writing3d.common as common
import writing3d.pens as pens
from actionlib import SimpleGoalState


class StrokeWriter:

    """Writes one stroke"""

    class Status:
        NOT_STARTED = 0
        DRAWING = 1
        COMPLETED = 2

    def __init__(self, stroke, client, dimension=500, pen=pens.BrushSmall,
                 robot_name="movo", arm="right_arm", origin_pose=None, num_waypoints=5,
                 resolution=None, z_resolution=None, z_min=None, z_max=None):
                 
        """
        stroke (np.array) array of waypoints (x, y, z, z2, altitude, azimuth)
        dimension (int) dimension of a character's image (default. 500 pixles)
        pen (pens.Pen) a type of pen. Its configuration overrides the other
                       provided parameters (e.g. resolution, z_min, etc.)
        resolution (float) metric length for one pixel (default. 1cm)
        origin_pose (Pose) Cartesian pose for robot arm that corresponds to
                           the origin of the character's image (not just stroke).
        z_resolution (float) resolution in the vertical direction (in meters). If
                             set to None, the stroke writer will only write in x, y.
        z_min (float) the minimum displacement in z direction the end effector can move,
                      so as to avoid bumping into the surface.
        z_max (float) the maximum relative z-coordinate the end 
        """
        self._stroke = stroke
        self._dimension = dimension
        if pen is not None:
            pen_config = pen.get_config()
            self._resolution = pen_config.get('RESOLUTION', None)
            self._z_resolution = pen_config.get('Z_RESOLUTION', None)
            self._z_min = pen_config.get('Z_MIN', None)
            self._z_max = pen_config.get('Z_MAX', None)
            self._z_lift = pen_config.get('Z_LIFT', None)
        else:
            self._resolution = resolution
            self._z_resolution = z_resolution
            self._z_min = z_min
            self._z_max = z_max
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
        except (KeyboardInterrupt, rospy.ROSInternalException):
            print("Interrupt...")
            self._client.go_fail()
            return
        
    @property
    def waypoints(self):
        return self._waypoints

    @property
    def origin_pose(self):
        return self._origin_pose

    def set_origin_pose(self, pose):
        self._origin_pose = pose

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
                
                if self._z_resolution is not None:
                    wz = z * self._z_resolution
                    if wz > self._z_max or wz < self._z_min:
                        util.warning("Movement in z %.3f is out of range (%.3f ~ %.3f)"
                                     % (wz, self._z_min, self._z_max))
                        wz = max(min(wz, self._z_max), self._z_min)
                    current_pose.position.z += wz
                    print(euler_from_quaternion([
                        current_pose.orientation.x,
                        current_pose.orientation.y,
                        current_pose.orientation.z,
                        current_pose.orientation.w,
                    ]))
                    print((math.radians(az), math.radians(al), math.radians(0)))
                    # current_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(math.radians(al),
                    #                                                                                math.radians(az),
                    #                                                                                0.0)) # roll, pitch, yaw
                waypoints.append(current_pose)

            # filter waypoints. There are too many
            self._waypoints = util.downsample(waypoints, self._num_waypoints)
            
            # At the end of the stroke, lift the pen.
            if len(waypoints) > 0:
                last_pose = copy.deepcopy(self._waypoints[-1])
            else:
                last_pose = copy.deepcopy(self._origin_pose)
            last_pose.position.z += self._z_lift
            self._waypoints.append(last_pose)

            # Print stats
            self._print_waypoint_stats()
            

    def Draw(self, method="together"):
        """
        method can be "together" or "separate".
        """
        # start planning
        if self._waypoints is None:
            return

        self._status = StrokeWriter.Status.DRAWING
        if method == "together":
            self._client.send_and_execute_goals(self._arm, [self._waypoints], wait=True)
        elif method == "separate":
            self._client.send_and_execute_goals(self._arm, self._waypoints, wait=True)
        self._status = StrokeWriter.Status.COMPLETED

    def done(self):
        """Returns true if done drawing"""
        return self._status == StrokeWriter.Status.COMPLETED


class CharacterWriter:

    
    def __init__(self, strokes, dimension=500, pen=pens.BrushSmall,
                 robot_name="movo", arm="right_arm", num_waypoints=5,
                 blank_stroke_first=True):
        """
        with `blank_stroke_first` set to True, the robot writes an empty
        stroke, which effectively lifts the arm before writing the actual
        first stroke.
        """
        self._client = MoveitClient(robot_name)
        self._dimension = dimension
        self._pen = pen
        self._num_waypoints = num_waypoints
        self._strokes = strokes.tolist()
        self._blank_stroke_first = blank_stroke_first
        if self._blank_stroke_first:
            self._strokes.insert(0, [])
        self._origin_pose = None
        self._arm = arm
        self._robot_name = robot_name
        self._writers = []
        self._client = MoveitClient(robot_name)

        # Print pen config
        self._pen.print_config()

        # Print stroke stats
        self._print_stroke_stats()

    def init_writers(self):
        """Initialize stroke writers. When this function is called, the robot arm should be in the
        desired 'origin_pose' (see definition in StrokeWriter.__init__(). For example, this function
        can be called after 'GetReady' which is supposed to move the robot arm to a pose suitable
        to begin writing for a particular type of pen."""
        for i in range(len(self._strokes)):
            util.info("Initializing writer for stroke %d" % i)
            self._writers.append(StrokeWriter(self._strokes[i], self._client, pen=self._pen,
                                              dimension=self._dimension,
                                              robot_name=self._robot_name, arm=self._arm,
                                              num_waypoints=self._num_waypoints, origin_pose=self._origin_pose))
            self._origin_pose = self._writers[i].origin_pose

        # Print stats
        self._print_waypoint_stats()

    def visualize(self):
        for i in range(len(self._strokes)):
            self._writers[i].visualize()
        plt.show()

    def _print_stroke_stats(self):
        first_stroke_indx = 1 if self._blank_stroke_first else 0
        allmax = np.max(self._strokes[first_stroke_indx], axis=0)
        allmin = np.min(self._strokes[first_stroke_indx], axis=0)
        for stroke in self._strokes[first_stroke_indx+1:]:
            allmax = np.max(np.vstack((allmax, stroke)), axis=0)
            allmin = np.min(np.vstack((allmin, stroke)), axis=0)
        
        print("========= Ranges (image-space) ========")
        print("x range: %.3f (%.3f ~ %.3f)" % (allmax[0] - allmin[0], allmin[0], allmax[0]))
        print("y range: %.3f (%.3f ~ %.3f)" % (allmax[1] - allmin[1], allmin[1], allmax[1]))
        print("z range: %.3f (%.3f ~ %.3f)" % (allmax[2] - allmin[2], allmin[2], allmax[2]))
        print("al range: %.3f (%.3f ~ %.3f)" % (allmax[3] - allmin[3], allmin[3], allmax[3]))
        print("az range: %.3f (%.3f ~ %.3f)" % (allmax[4] - allmin[4], allmin[4], allmax[4]))

        print("========= Ranges (world-space) ========")
        resolution = self._pen.param("RESOLUTION")
        z_resolution = self._pen.param("Z_RESOLUTION")
        print("x range: %.3f (%.3f ~ %.3f)" % ((allmax[0] - allmin[0]) * resolution,
                                               allmin[0] * resolution,
                                               allmax[0] * resolution))
        print("y range: %.3f (%.3f ~ %.3f)" % ((allmax[1] - allmin[1]) * resolution,
                                               allmin[1] * resolution,
                                               allmax[1] * resolution))
        if z_resolution is not None:
            print("z range: %.3f (%.3f ~ %.3f)" % ((allmax[2] - allmin[2]) * z_resolution,
                                                   allmin[2] * z_resolution,
                                                   allmax[2] * z_resolution))
            print("al range: %.3f (%.3f ~ %.3f)" % (math.radians(allmax[3] - allmin[3]),
                                                    math.radians(allmin[3]),
                                                    math.radians(allmax[3])))
            print("az range: %.3f (%.3f ~ %.3f)" % (math.radians(allmax[4] - allmin[4]),
                                                    math.radians(allmin[4]),
                                                    math.radians(allmax[4])))


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
                    
    ### Actions that will lead to robot motion ###
    def DipPen(self):
        dip_high = common.goal_file("dip_high")
        dip_low = common.goal_file("dip_low")
        dip_move = common.goal_file("dip_move")
        dip_retract = common.goal_file("dip_retract")
        goal_files = [
            dip_high, dip_low, dip_move, dip_high, dip_retract
        ]
        self._client.send_and_execute_joint_space_goals_from_files(self._arm, goal_files)

    def GetReady(self, pen_type="brush_small"):
        ready = common.goal_file("touch_joint_pose_%s" % pen_type)
        self._client.send_and_execute_joint_space_goals_from_files(self._arm, [ready])

    def Write(self, index=-1, visualize=True, method="together"):
        if len(self._strokes) != len(self._writers):
            raise Value("Incorrect number of stroke writers. Needs %d but got %d"
                        % (len(self._strokes), len(self._writers)))
        if index < 0:
            # Draw entire character
            for i in range(len(self._strokes)):
                if self._client.is_healthy():
                    self.Write(i)
        else:
            if index > 0 and self._origin_pose is None:
                util.warning("Origin pose unknown but not writing the first stroke."\
                             "Will treat current pose as origin.")
            # Wait till finish
            if len(self._strokes[index]) == 0:
                print("Empty stroke. Lifting arm...")
            else:
                print("Drawing stroke %d..." % index)
            self._writers[index].Draw(method=method)
            try:
                while self._client.is_healthy() \
                      and not self._writers[index].done():
                    rospy.sleep(1.0)
                util.success("Done. Client is %s" % ("up"
                                                     if self._client.is_healthy()
                                                     else "down!"))
            except (KeyboardInterrupt, rospy.ROSInternalException):
                print("Interrupt...")
                self._client.go_fail()
                return
#-- End of CharacterWriter --#
        

def main():
    # Ad-hoc
    FILE = "../../data/stroke.npy"
    characters = np.load(FILE)

    util.info("Starting character writer...")
    try:
        writer = CharacterWriter(characters[0], num_waypoints=10)
        # util.warning("Dipping pen...")
        # writer.DipPen()
        util.warning("Getting ready...")
        writer.GetReady()
        writer.init_writers()
        util.warning("Begin writing...")
        rospy.sleep(2)
        # writer.Write()
    except KeyboardInterrupt:
        print("Terminating...")
    except Exception as ex:
        print("Exception! %s" % ex)
        import traceback
        traceback.print_exc()

        
if __name__ == "__main__":
    main()
