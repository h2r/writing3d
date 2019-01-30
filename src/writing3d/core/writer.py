#!/usr/bin/env python
#
# Convention: If a public function is named with CamelCase, then it can possibly lead
# to robot motion

import matplotlib.pyplot as plt
import os, sys
import rospy
import copy
import numpy as np
import yaml
import math
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from writing3d.robot.moveit_client import MoveitClient
from writing3d.robot.moveit_planner import MoveitPlanner
import writing3d.common as common
import writing3d.core.pens as pens
import writing3d.util as util
import writing3d.robot.movo_pose_publisher as movo_pp
from actionlib import SimpleGoalState
import argparse

common.DEBUG_LEVEL = 2

class StrokeWriter:

    """Writes one stroke"""

    class Status:
        NOT_STARTED = 0
        DRAWING = 1
        COMPLETED = 2

    def __init__(self, stroke, stroke_index, client, dimension=500, pen=pens.SmallBrush,
                 robot_name="movo", arm="right_arm", origin_pose=None, num_waypoints=5,
                 resolution=None, z_resolution=None, z_min=None, z_max=None):
                 
        """
        stroke (np.array) array of waypoints (x, y, z, z2, altitude, azimuth)
        strok_index (int) index of the stroke in the character that it belongs to.
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
        self._stroke_index = stroke_index
        self._dimension = dimension
        if pen is not None:
            self._pen = pen
            pen_config = self._pen.get_config()
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

            # Filter stroke points
            stroke_points = self._stroke
            
            # filter stroke points. There are too many
            if self._num_waypoints > 0:
                stroke_points = util.downsample(stroke_points, every=self._num_waypoints)
                print("Downsampling stroke points from %d to %d waypoints..." % (len(self._stroke), len(stroke_points)))

            if len(stroke_points) > 0:
                print("Starting x,y location: %s" % (stroke_points[0][:2]))

            # x, y, z are in meters; al and az are degrees
            for x, y, z, z2, al, az in stroke_points:
                # Map from image space to world space
                current_pose = copy.deepcopy(self._origin_pose)
                wx = -y * self._resolution
                wy = -x * self._resolution
                current_pose.position.x += wx
                current_pose.position.y += wy

                # # For debugging
                # dwx = current_pose.position.x - self._origin_pose.position.x
                # dwy = current_pose.position.y - self._origin_pose.position.y
                # dx = -dwy / self._resolution
                # dy = -dwx / self._resolution

                if self._pen.uses_z():
                    if self._z_resolution is not None:
                        # Force; larger z means downward more, so reverse.
                        #wz = -z * self._z_resolution
                        wz = util.translate(-z, -11.06, -0.175, self._pen.CONFIG['Z_MIN'], self._pen.CONFIG['Z_MAX'])
                        # For security concerns:
                        # if wz > self._z_max or wz < self._z_min:
                        #     util.warning("Movement in z %.4f is out of range (%.4f ~ %.4f)"
                        #                  % (wz, self._z_min, self._z_max))
                        #     wz = max(min(wz, self._z_max), self._z_min)
                        current_pose.position.z += wz

                        # Orientation;
                        if self._pen.uses_orientation():
                            if self._pen.fix_az():
                                az = stroke_points[0][-1]  # the azimuth of the first point
                            if self._pen.fix_al():
                                al = stroke_points[0][-2]  # the altitude of the first point
                            al = self._pen.map_value(al, val_type="al")
                            print(al)
                            
                            euler = list(self._pen.CONFIG["O_INI"])
                            euler[self._pen.CONFIG["AZ_I"]] += az * self._pen.CONFIG["AZ_FACTOR"]
                            euler[self._pen.CONFIG["AL_I"]] += (90-al) * self._pen.CONFIG["AL_FACTOR"]
                            current_pose.orientation = geometry_msgs.msg.Quaternion(*quaternion_from_euler(math.radians(euler[0]),
                                                                                                           math.radians(euler[1]),
                                                                                                           math.radians(euler[2])))
                else:
                    # current_pose.position.z
                    pass
                            
                waypoints.append(current_pose)

            self._waypoints = waypoints

            # Before the beginning of the stroke, lift the pen
            if len(waypoints) > 0:
                first_pose = copy.deepcopy(self._waypoints[0])
            else:
                first_pose = copy.deepcopy(self._origin_pose)
            first_pose.position.z += self._z_lift
            self._waypoints.insert(0, first_pose)
            
            # At the end of the stroke, lift the pen.
            if len(waypoints) > 0:
                last_pose = copy.deepcopy(self._waypoints[-1])
            else:
                last_pose = copy.deepcopy(self._origin_pose)
            last_pose.position.z += self._z_lift
            self._waypoints.append(last_pose)

            # Print stats
            if common.DEBUG_LEVEL > 3:
                self._print_waypoint_stats()
            

    def Draw(self, method="together"):
        """
        method can be "together" or "separate".
        """
        # start planning
        if self._waypoints is None:
            return

        self._status = StrokeWriter.Status.DRAWING
        exec_args = {
            "stroke_index": self._stroke_index
        }
        if len(self._waypoints) <= 4:
            method = "together"

        print(method)
        
        if method == "separate":
            util.info2("first.")
            self._client.send_and_execute_goals(self._arm, [[self._waypoints[0]]], wait=True,
                                                exec_args=exec_args)
            util.info2("middle.")
            self._client.send_and_execute_goals(self._arm, [self._waypoints[1:-1]], wait=True,
                                                exec_args=exec_args)
            util.info2("last.")
            self._client.send_and_execute_goals(self._arm, [[self._waypoints[-1]]], wait=True,
                                                exec_args=exec_args)
        elif method == "together":
            self._client.send_and_execute_goals(self._arm, [self._waypoints], wait=True,
                                                exec_args=exec_args)
        self._status = StrokeWriter.Status.COMPLETED


    def done(self):
        """Returns true if done drawing"""
        return self._status == StrokeWriter.Status.COMPLETED


class CharacterWriter:
    
    def __init__(self, strokes, dimension=500, pen=pens.Pen,
                 robot_name="movo", arm="right_arm", num_waypoints=5,
                 blank_stroke_first=True, retract_after_stroke=False,
                 retract_scale=1):
        """
        with `blank_stroke_first` set to True, the robot writes an empty
        stroke, which effectively lifts the arm before writing the actual
        first stroke.

        with `retract_after_stroke` set to True, the robot retracts its arm
        after writing every stroke (moves the arm away from)
        """
        self._client = MoveitClient(robot_name)
        self._dimension = dimension
        self._pen = pen
        self._num_waypoints = num_waypoints
        self._strokes = strokes.tolist()
        self._blank_stroke_first = blank_stroke_first
        self._retract_after_stroke = retract_after_stroke
        self._retract_scale = retract_scale
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
        can be called after 'ReadyPose' which is supposed to move the robot arm to a pose suitable
        to begin writing for a particular type of pen."""
        for i in range(len(self._strokes)):
            util.info("Initializing writer for stroke %d" % i)
            stroke_index = (i-1) if self._blank_stroke_first else i
            self._writers.append(StrokeWriter(self._strokes[i], stroke_index,
                                              self._client, pen=self._pen,
                                              dimension=self._dimension,
                                              robot_name=self._robot_name, arm=self._arm,
                                              num_waypoints=self._num_waypoints, origin_pose=self._origin_pose))
            self._origin_pose = self._writers[i].origin_pose

        # Print stats
        if common.DEBUG_LEVEL > 0:
            self._print_waypoint_stats()


    @property
    def arm(self):
        return self._arm
            
    @property
    def arm_side(self):
        return self._arm.split("_")[0]

    @property
    def client(self):
        return self._client

    
    def visualize_strokes(self):
        """Visualize each stroke"""
        for i in range(len(self._strokes)):
            self._writers[i].visualize()
        plt.show()

    def print_character(self, res=50):
        """res means the side width of the text area you want to print. It shouldn't be large (above 50)
        since that would not be readable. This is of course not accurate."""
        img = np.zeros((res, res), dtype=np.int32)
        ratio = float(self._dimension) / float(res)
        for stroke in self._strokes:
            for p in stroke:
                x, y = int(p[0]/ratio), int(p[1]/ratio)
                img[x,y] = 1
        last_point = (int(self._strokes[-1][-1][0] / ratio),
                      int(self._strokes[-1][-1][1] / ratio))
        img = img.transpose()
        started_content = False
        for r in range(img.shape[0]):
            if not started_content:
                if 1 not in img[r]:
                    continue # skip blank lines at first
            if r > last_point[0]:
                continue # skip blank lines at the end
            for c in range(img.shape[1]):
                if img[r,c] == 0:
                    sys.stdout.write(" ")
                else:
                    started_content = True
                    sys.stdout.write("-")
            sys.stdout.write("\n")

    def _get_stroke_ranges(self):
        first_stroke_indx = 1 if self._blank_stroke_first else 0
        allmax = np.max(self._strokes[first_stroke_indx], axis=0)
        allmin = np.min(self._strokes[first_stroke_indx], axis=0)
        for stroke in self._strokes[first_stroke_indx+1:]:
            allmax = np.max(np.vstack((allmax, stroke)), axis=0)
            allmin = np.min(np.vstack((allmin, stroke)), axis=0)
        return allmin, allmax

    def _print_stroke_stats(self):
        allmin, allmax = self._get_stroke_ranges()
        print("========= Ranges (image-space) ========")
        print("x range: %.3f (%.3f ~ %.3f)" % (allmax[0] - allmin[0], allmin[0], allmax[0]))
        print("y range: %.3f (%.3f ~ %.3f)" % (allmax[1] - allmin[1], allmin[1], allmax[1]))
        print("z range: %.3f (%.3f ~ %.3f)" % (allmax[2] - allmin[2], allmin[2], allmax[2]))

        print("========= Ranges (world-space) ========")
        print("------ Note: Influenced by pen! -------")
        resolution = self._pen.param("RESOLUTION")
        z_resolution = self._pen.param("Z_RESOLUTION")
        print("x range: %.3f (%.3f ~ %.3f)" % ((allmax[0] - allmin[0]) * resolution,
                                               allmin[0] * resolution,
                                               allmax[0] * resolution))
        print("y range: %.3f (%.3f ~ %.3f)" % ((allmax[1] - allmin[1]) * resolution,
                                               allmin[1] * resolution,
                                               allmax[1] * resolution))
        print("z range: %.5f (%.5f ~ %.5f)" % ((allmax[2] - allmin[2]) * z_resolution,
                                               allmin[2] * z_resolution,
                                               allmax[2] * z_resolution))
        if self._pen.uses_orientation():
            print("al range: %.3f (%.3f ~ %.3f)" % (allmax[4] - allmin[4], allmin[4], allmax[4]))
            print("az range: %.3f (%.3f ~ %.3f)" % (allmax[5] - allmin[5], allmin[5], allmax[5]))


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

    def save_origin_pose(self, character_save_directory):
        if os.path.exists(character_save_directory):
            with open(os.path.join(character_save_directory, "origin_pose.yml"), "w") as f:
                yaml.dump(self._origin_pose, f)
                util.success("Saved origin pose")
        else:
            util.warning("%s does not exist. origin pose is not saved." % character_save_directory)
                    
    ### Actions that will lead to robot motion ###
    def DipPen(self):
        dip_high = common.goal_file("dip_high")
        dip_low = common.goal_file("dip_low")
        dip_retract = common.goal_file("dip_retract")
        goal_files = [
            dip_high, dip_low, dip_high, dip_retract
        ]
        self._client.send_and_execute_joint_space_goals_from_files(self._arm, goal_files)

    def DipRetract(self):
        dip_retract = common.goal_file("dip_retract")
        self._client.send_and_execute_joint_space_goals_from_files(self._arm, [dip_retract])

    def ReadyPose(self):
        ready = self._pen.touch_pose()
        self._client.send_and_execute_joint_space_goals_from_files(self._arm, [ready])

    def _Retract(self):
        util.info2("Retracting...")
        retract = self._pen.retract_pose()
        self._client.send_and_execute_joint_space_goals_from_files(self._arm, [retract])
        rospy.sleep(1)

    def Write(self, index=-1, method="together", stroke_complete_cb=None, cb_args=None):
        if len(self._strokes) != len(self._writers):
            raise Value("Incorrect number of stroke writers. Needs %d but got %d"
                        % (len(self._strokes), len(self._writers)))
        if index < 0:
            # Draw entire character
            for i in range(len(self._strokes)):
                if self._client.is_healthy():
                    self.Write(i)
                    if len(self._strokes[i]) > 0:
                        if self._retract_after_stroke:
                            self._Retract()
                        if stroke_complete_cb is not None:
                            if self._blank_stroke_first:
                                stroke_complete_cb(i-1, **cb_args)  # -1 to account for the empty stroke
                            else:
                                stroke_complete_cb(i, **cb_args)
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

def write_characters(characters, retract_after_stroke=True, retract_scale=1, pen=pens.SmallBrush):
    """Each character is an array of strokes, which is an array of waypoints (x,y,z,z2,al,az)
    Currently, the characters are all written with respect to the same origin pose. So they
    will overlap. A human assistant should replace the paper once a character is written"""

    for i, character in enumerate(characters):
        util.info("Starting character writer...")
        try:
            writer = CharacterWriter(character, pen=pen, num_waypoints=10,
                                     retract_after_stroke=retract_after_stroke,
                                     retract_scale=retract_scale)
            # Print character
            writer.print_character(res=40)
            util.warning("Dipping pen...")
            if pen.needs_dip():
                writer.DipPen()
            util.warning("Getting ready...")
            writer.ReadyPose()
            writer.init_writers()
            util.warning("Begin writing...")
            rospy.sleep(2)
            writer.Write()
            util.warning("Finished writing. Repositioning...")
            if pen.needs_dip():
                writer.DipRetract()
        except KeyboardInterrupt:
            print("Terminating...")
        except Exception as ex:
            print("Exception! %s" % ex)
            import traceback
            traceback.print_exc()

def main():
    parser = argparse.ArgumentParser(description='Let MOVO write characters')
    parser.add_argument("path", type=str, help="Path to a .npy file that contains characters"\
                        "data (each character is list of strokes, each of which is a list of waypoints).")
    parser.add_argument("index", type=int, help="index of the character you want to write."\
                        "If negative, all characters will be written.")
    parser.add_argument("--continuous", help="The robot will not retract after writing every stroke.",
                        action="store_true")
    parser.add_argument("--retract-scale", type=float, help="How much you want to retract the arm after writing"\
                        "every stroke. Acceptable values: float between 1 to 2", default=1.0)
    args = parser.parse_args()
    characters = np.load(args.path)    
    if args.index < 0:
        write_characters(characters, retract_after_stroke=(not args.continuous),
                         retract_scale=args.retract_scale)
    else:
        if args.index >= len(characters):
            raise ValueError("Index out of bound. Valid range: 0 ~ %d" % (len(characters)))
        write_characters([characters[args.index]], retract_after_stroke=(not args.continuous),
                         retract_scale=args.retract_scale)
        
if __name__ == "__main__":
    main()
