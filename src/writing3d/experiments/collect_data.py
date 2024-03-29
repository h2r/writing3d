#!/usr/bin/env python
#
# Pipeline for collecting data.
#
# 1. Ready -> Dip ink
# 2. Dot on 4 corners
# 3. Start UI; the same time, start writing a
#    testing character. Wait for user to specify
#    bounding box.
# 4. If the user signals "start", start collecting
#    data. After completing each stroke, the
#    camera will take a picture. The picture will
#    be saved, as well as the robot arm's x,y positions
#    when drawing the stroke.
# 5. After finish writing a character, wait for humans
#    to change paper.
#
# Manual steps before starting:
# - Start a planner
#   ./moveit_planner.py right_arm left_arm right_pen_tip_link left_ee_link
# - Adjust kinect pan to 0.0 and tilt to -0.6 by
#   ./movo_pose_publisher.py head -p 0.0 0.2 0.1 -t -0.6 0.2 0.1
# - Move the left arm away by
#   ./moveit_client.py left_arm -f ../../../cfg/left_clearway.yml
#   ./moveit_client.py left_arm -e
# - Let the arm be at dip_retract pose
#   ./moveit_client.py right_arm -f ../../../cfg/dip_retract.yml
#   ./moveit_client.py right_arm -e
# - Place a table (with two layered foams) in front of the
#   robot.
#
# Try:
# ./collect_data.py ../../../data/stroke.npy ../../../data/characters/ -g ../../../cfg/gui_config.yml
import math
import cv2
import os, sys, signal
import rospy
import subprocess
from multiprocessing import Process
import numpy as np
from scipy.ndimage.measurements import label
import writing3d.common as common
import writing3d.util as util
from writing3d.robot.movo_vision import MovoKinectInterface
from writing3d.core.writer import CharacterWriter, write_characters
from writing3d.core.gui import WritingGui
import writing3d.core.pens as pens

import argparse


class CollectData():

    def __init__(self, characters, sorted_cindx, dimension, gui, save_dir,
                 pen=pens.SmallBrush, test_first=False, num_waypoints=-1):
        self._characters = characters
        self._sorted_cindx = sorted_cindx
        self._current_character = None
        self._dimension = dimension
        self._num_waypoints = num_waypoints
        self._gui = gui
        self._pen = pen
        self._save_dir = save_dir
        self._done = False
        self._test_first = test_first


    def _produce_stroke_image_from_difference(self,
                                              pen,
                                              prev_stroke_img,
                                              cur_stroke_img,
                                              stroke):
        """This is not as easy as it seems, because there could
        be arbitrarily random differences from the previous image
        to the current image due to light reflection.

        The way to produce the stroke image to use is that:
        
          Take the difference
           -> Get the positive values
           -> Cluster (connected components)
           -> Eliminate small clusters and keep big clusters.
        
        This is not a particularly efficient procedure but it will
        benefit the network.
        """
        # The ink that was there, will still be there.
        # The ink that were just added may or may not be there.
        if pen == pens.SmallBrush:
            min_coverage = 0.001
        elif pen == pens.Sharpe or pen == pens.StraightSharpe:
            min_coverage = 0.0005
            
        points_on_stroke = {(p[0],p[1]) for p in stroke}

        diff = cur_stroke_img - prev_stroke_img
        pos_diff = np.where(diff!=1, diff, 0)  # because img is of type uint8,
                                              # rather than having -255, we will have 1.
        structure = np.ones((3, 3), dtype=np.int)
        labeled, ncomponents = label(pos_diff, structure)
        indices = np.flip(np.indices(reversed(labeled.shape)).T[:,:,[0, 1]], axis=len(labeled.shape))
        comps_kept = []
        for i in range(ncomponents):
            # Get the center of mass (based on indices) for this component
            indices_for_component = indices[labeled==(i+1)]

            # If the component is bigger than 1% image, definitely keep it.
            if len(indices_for_component) > self._dimension*self._dimension * 0.001:  # only consider big ones
                comps_kept.append(i)
        stroke_img = np.copy(prev_stroke_img)
        for i in comps_kept:
            stroke_img[labeled==i+1] = 255
        return stroke_img, labeled, comps_kept
        

    def stroke_complete_cb(self, stroke_indx, **kwargs):
        # should be path to the directory for the character.
        save_dir = kwargs.get("save_dir", None)
        # if true, the image for stroke i will be image for i-1 plus
        # their difference.
        take_difference = kwargs.get("take_difference", False)

        util.info("Saving information after writing stroke %d" % (stroke_indx+1))
        img = self._kinect.take_picture(hd=True)
        img_char = self._gui.extract_character_image(img_src=img, show_result=False)
        
        # This image may be off. We may need to shift it according to the robot's path.
        if take_difference:
            if len(self._gui.stroke_images) > 0:
                img_char, labeled, comps_kept = self._produce_stroke_image_from_difference(
                    self._pen, self._gui.stroke_images[-1], img_char, self._current_character[stroke_indx])
                
        self._gui.add_stroke_image(img_char)
        self._gui.save_stroke_image(img_char, os.path.join(save_dir, "stroke-%d.bmp" % stroke_indx))


    def run(self):
        if self._done:
            return        
        # write a test character
        rospy.init_node("collect_writing_data", anonymous=True, disable_signals=True)

        self._kinect = MovoKinectInterface()
        
        if self._test_first:
            # 1. Dip ink & 2. Dot on 4 corners
            dot_four_corners(self._dimension, self._pen)

            util.info2("Writing a character (below). Please specify bounding box in GUI.",
                       bold=True)
            self._gui.set_writing_character(self._characters[0], 0)
            write_characters([self._characters[0]], retract_after_stroke=False, pen=self._pen)
            util.info("Sleeping for 20 seconds. Confirm bounding box within this time.")
            rospy.sleep(20)
        rospy.sleep(2)

        # 4. start collecting data
        for i in self._sorted_cindx:
            character = self._characters[i]
            util.info("Starting character writer...")
            save_dir = os.path.join(self._save_dir, "Character-%d" % i)
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            self._current_character = character
            rospy.set_param("current_writing_character_index", i)
            rospy.set_param("current_writing_character_save_dir", save_dir)
            try:
                writer = CharacterWriter(character, pen=self._pen,
                                         num_waypoints=self._num_waypoints,
                                         retract_after_stroke=False,
                                         retract_scale=0.5)
                writer.print_character(res=40)
                self._gui.set_writing_character(character, i, char_dir=save_dir)
                self._gui.save_writing_character_image(os.path.join(save_dir, "image.bmp"))
                if self._pen.needs_dip():
                    dip_pen(writer)
                get_ready(writer)
                rospy.sleep(5) # use up some ink to lighten first strokes.
                writer.init_writers()
                writer.save_origin_pose(save_dir)
                util.warning("Begin writing...")
                rospy.sleep(2)
                writer.Write(stroke_complete_cb=self.stroke_complete_cb,
                             cb_args={
                                 'save_dir': save_dir,
                                 'take_difference': True
                             })
                util.warning("Finished writing. Repositioning...")
                if self._pen.needs_dip():
                    writer.DipRetract()
                util.success("Character writing finished! Pause for 4 seconds before"\
                             "starting the next one.", bold=True)
                rospy.sleep(4)

            except Exception as ex:
                print("Exception! %s" % ex)
                import traceback
                traceback.print_exc()
        self._done = True

class FakeGuiWrapper():
    def __init__(self, gui_config_file=None, pen=pens.SmallBrush):
        self._gui = WritingGui(hd=True, is_fake=True,
                               character_res=pen.CONFIG['RESOLUTION'],
                               character_zres=pen.CONFIG['Z_RESOLUTION'])
        self._gui.init()
        logo_img = cv2.cvtColor(cv2.imread("logo.jpg", cv2.IMREAD_UNCHANGED),
                                cv2.COLOR_BGR2RGB)
        self._gui.show_image("logo", logo_img, background=True)
        if gui_config_file is not None:
            self._gui.set_config_file(gui_config_file)
            if os.path.exists(gui_config_file):
                self._gui.load_config(gui_config_file)

    @property
    def gui(self):
        return self._gui

    def run(self):
        try:
            self._gui.spin()
        except KeyboardInterrupt:
            print("Terminating...")

        

def dip_pen(writer):
    util.warning("Dipping pen...")
    writer.DipPen()

def get_ready(writer):
    util.warning("Getting ready...")
    writer.ReadyPose()

def dip_retract(writer):
    util.warning("Resetting pose to dip retract...")
    writer.DipRetract()

def dot_four_corners(dim, pen):
    # strokes is an array of waypoints (x,y,z,z2,al,az)
    angles = [None, None, 0.0]
    if pen.uses_orientation():
        angles[pen.CONFIG["AL_I"]] = pen.CONFIG["O_REST"][pen.CONFIG["AL_I"]]
        angles[pen.CONFIG["AZ_I"]] = pen.CONFIG["O_REST"][pen.CONFIG["AZ_I"]]
    strokes = np.array([
        [[0, 0, 0.5] + angles],
        [[dim-1, 0, 0.5] + angles],
        [[dim-1, dim-1, 0.5] + angles],
        [[0, dim-1, 0.5] + angles]
    ])
    writer = CharacterWriter(strokes, pen=pen,
                             num_waypoints=-1,
                             retract_after_stroke=False)
    if pen.needs_dip():
        dip_pen(writer)
    get_ready(writer)
    writer.init_writers()
    util.warning("Dipping corners...")
    rospy.sleep(2)
    writer.Write()           
    util.warning("Finished writing. Repositioning...")
    if pen.needs_dip():
        writer.DipRetract()


def begin_procedure(characters, sorted_cindx, pen, dimension, save_dir,
                    test_first=False, gui_config_file=None,
                    num_waypoints=-1):

    # 3. Start UI  -- this gui will not display anything; it
    # just uses the WritingGui API and kinect to take images.
    fg = FakeGuiWrapper(gui_config_file, pen=pen)
    task = CollectData(characters, sorted_cindx, dimension, fg.gui, save_dir,
                       pen=pen, test_first=test_first,
                       num_waypoints=num_waypoints)    
    p = Process(target=fg.run, args=())
    p.daemon = True
    p.start()
    task.run()
    p.terminate()
    p.join()


def main():
    parser = argparse.ArgumentParser(description='Let MOVO collect training data.')
    parser.add_argument("chars_path", type=str, help="Path to a .npy file that contains characters"\
                        "data (each character is list of strokes, each of which is a list of waypoints).")
    parser.add_argument("save_dirpath", type=str, help="Directory to save the collected data")
    parser.add_argument("-n", "--num-chars", type=int, help="Number of characters to write."\
                        "If negative, all characters will be written.", default=-1)
    parser.add_argument("-i", "--index", type=int, help="Write character at specific index of the file."\
                        "If negative, all characters will be written.", default=-1)
    parser.add_argument("-I", "--indices", type=int, nargs="+", help="Write character at specific index of the file."\
                        "If negative, all characters will be written.")
    parser.add_argument("-p", "--pen", type=str, help="Type of pen to use. See pens.py",
                        default=pens.SmallBrush.name())
    parser.add_argument("-d", "--dim", type=int, help="Dimension of the character image. Default 500.",
                        default=500)
    parser.add_argument("--test-first", help="Write a test character first to help setting the bounding"\
                        "box", action="store_true")
    parser.add_argument("-g", "--gui-config-file", type=str, help="Path to a file that stores gui config."
                        "If the file does not exist, when gui saves config, it will use this path.",
                        default=None)
    parser.add_argument("--num-waypoints", type=int, help="Number of waypoints per stroke. Negative if NO downsample."\
                        "default is -1", default=-1)
    parser.add_argument("--only-one", help="The file in `chars_path` has only one character",
                        action="store_true")
    args, _ = parser.parse_known_args()
    characters = np.load(args.chars_path)

    if args.only_one:
        characters = np.array([characters])
    
    if args.num_chars >= len(characters):
        raise ValueError("Index out of bound. Valid range: 0 ~ %d" % (len(characters)))

    if args.indices is not None and len(args.indices) > 0:
        used_characters = []
        for i in args.indices:
            used_characters.append(characters[i])
        characters = used_characters
    else:
        if args.index >= len(characters):
            raise ValueError("Character index %d out of bound. Total %d characters."
                             % (args.index, len(characters)))

        elif args.index > 0:
            characters = np.array([characters[args.index]])


    # Write simplest characters first
    stroke_lengths = [len(c) for c in characters]
    # sorted_cindx = util.argsort(stroke_lengths)
    sorted_cindx = [i for i in range(0, len(stroke_lengths))]

    if args.num_chars > 0:
        sorted_cindx = sorted_cindx[:args.num_chars]
        # characters = characters[:args.num_chars]

    gui_config_file_arg = ['-g', args.gui_config_file] \
                          if args.gui_config_file is not None else []

    only_one_arg = ['--only-one'] \
              if args.only_one else []
    

    # confirm pen. very important
    resp = raw_input("Using pen \"%s\". Confirm? [Y/n] " % args.pen)
    if not resp.upper().startswith("Y"):
        util.info2("Please make sure you have the pen you want.", bold=True)
        return
    
    # p_ext_gui = subprocess.Popen(['rosrun',
    #                               'writing3d',
    #                               'start_gui.py',
    #                               args.save_dirpath,
    #                               args.chars_path,
    #                               '-p', args.pen] + gui_config_file_arg + only_one_arg)
    try:
        begin_procedure(characters, sorted_cindx, pens.str_to_pen(args.pen),
                        args.dim, args.save_dirpath,
                        test_first=args.test_first, gui_config_file=args.gui_config_file,
                        num_waypoints=args.num_waypoints)
        # p_ext_gui.wait()
    except KeyboardInterrupt:
        # p_ext_gui.kill()
        pass

if __name__ == "__main__":
    main()
