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
#   ./moveit_planner right_arm left_arm
# - Adjust kinect pan to 0.0 and tilt to -0.6 by
#   ./movo_pose_publisher head -p 0.0 0.2 0.1 -t -0.6 0.2 0.1
# - Move the left arm away by
#   ./moveit_client left_arm -f ../../../cfg/left_clearway.yml
#   ./moveit_client left_arm -e
# - Let the arm be at dip_retract pose
#   ./moveit_client right_arm -f ../../../cfg/dip_retract.yml
#   ./moveit_client right_arm -e
# - Place a table (with two layered foams) in front of the
#   robot.
#
# Try:
# ./collect_data.py ../../../data/stroke.npy ../../../data/characters/ -g ../../../cfg/gui_config.yml
import os
import rospy
import subprocess
import numpy as np
import writing3d.common as common
import writing3d.util as util
from writing3d.robot.movo_vision import MovoKinectInterface
from writing3d.core.writer import CharacterWriter, write_characters
from writing3d.core.gui import WritingGui
import writing3d.core.pens as pens
import threading

import argparse


class CollectData(threading.Thread):

    def __init__(self, characters, gui, kinect, save_dir, test_first=False):
        threading.Thread.__init__(self)
        self._characters = characters
        self._gui = gui
        self._kinect = kinect
        self._save_dir = save_dir
        self._done = False
        self._test_first = test_first

    def stroke_complete_cb(self, stroke_indx, **kwargs):
        gui = kwargs.get("gui", None)
        kinect = kwargs.get("kinect", None)
        # should be path to the directory for the character.
        save_dir = kwargs.get("save_dir", None)

        util.info("Saving information after writing stroke %d" % stroke_indx)
        img = kinect.take_picture(hd=True)
        gui.show_kinect_image(img)
        img_char = gui.extract_character_image(img_src=img, show_result=False)
        gui.show_stroke_image(img_char)
        np.save(os.path.join(save_dir, "stroke-%d.npy" % stroke_indx),
                img_char)


    def run(self):
        if self._done:
            return
        # write a test character
        if self._test_first:
            util.info2("Writing a character (below). Please specify bounding box in GUI.",
                       bold=True)
            self._gui.set_writing_character(self._characters[0])
            write_characters([self._characters[0]], retract_after_stroke=False)
            util.info("Sleeping for 3 seconds. Confirm bounding box within this time.")
            ropspy.sleep(3)
        rospy.sleep(2)

        # 4. start collecting data
        for i, character in enumerate(self._characters):
            util.info("Starting character writer...")
            save_dir = os.path.join(self._save_dir, "Character-%d" % i)
            if not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)
            try:
                writer = CharacterWriter(character, pen=pens.SmallBrush,
                                         num_waypoints=10,  # should change to -1
                                         retract_after_stroke=True,
                                         retract_scale=1)
                writer.print_character(res=40)
                self._gui.set_writing_character(character)
                self._gui.save_writing_character_image(os.path.join(save_dir, "image.bmp"))
                dip_pen(writer)
                get_ready(writer)
                writer.init_writers()
                util.warning("Begin writing...")
                rospy.sleep(2)
                writer.Write(stroke_complete_cb=self.stroke_complete_cb,
                             cb_args={
                                 'gui': self._gui,
                                 'kinect': self._kinect,
                                 'save_dir': save_dir
                             })
                util.warning("Finished writing. Repositioning...")
                writer.DipRetract()
                util.success("Character writing finished! Pause for 4 seconds before"\
                             "starting the next one.", bold=True)
                rospy.sleep(4)

            except Exception as ex:
                print("Exception! %s" % ex)
                import traceback
                traceback.print_exc()
        self._done = True

        

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
    strokes = np.array([
        [[0, 0, 1.0, 0.0, 0.0, 0.0]],
        [[dim-1, 0, 1.0, 0.0, 0.0, 0.0]],
        [[dim-1, dim-1, 1.0, 0.0, 0.0, 0.0]],
        [[0, dim-1, 1.0, 0.0, 0.0, 0.0]]
    ])
    writer = CharacterWriter(strokes, pen=pen,
                             num_waypoints=-1,
                             retract_after_stroke=False)
    dip_pen(writer)
    get_ready(writer)
    writer.init_writers()
    util.warning("Dipping corners...")
    rospy.sleep(2)
    writer.Write()           
    util.warning("Finished writing. Repositioning...")
    writer.DipRetract()


def begin_procedure(characters, pen, dimension, save_dir,
                    test_first=False, gui_config_file=None):

    # 1. Dip ink & 2. Dot on 4 corners
    # dot_four_corners(dimension, pen)

    # 3. Start UI
    gui = WritingGui(hd=True)
    gui.init()
    kinect = MovoKinectInterface()
    gui.set_kinect(kinect, take_picture_every=0.5)
    gui.update_kinect_image_periodically()

    if gui_config_file is not None:
        gui.set_config_file(gui_config_file)
        if os.path.exists(gui_config_file):
            gui.load_config(gui_config_file)

    task = CollectData(characters, gui, kinect, save_dir, test_first=test_first)
    task.daemon = True
    task.start()
    gui.spin()



def main():
    parser = argparse.ArgumentParser(description='Let MOVO collect training data.')
    parser.add_argument("chars_path", type=str, help="Path to a .npy file that contains characters"\
                        "data (each character is list of strokes, each of which is a list of waypoints).")
    parser.add_argument("save_dirpath", type=str, help="Directory to save the collected data")
    parser.add_argument("-n", "--num-chars", type=int, help="Number of characters to write."\
                        "If negative, all characters will be written.", default=-1)
    parser.add_argument("-p", "--pen", type=str, help="Type of pen to use. See pens.py",
                        default=pens.SmallBrush.name())
    parser.add_argument("-d", "--dim", type=int, help="Dimension of the character image. Default 500.",
                        default=500)
    parser.add_argument("--test-first", help="Write a test character first to help setting the bounding"\
                        "box", action="store_true")
    parser.add_argument("-g", "--gui-config-file", type=str, help="Path to a file that stores gui config."
                        "If the file does not exist, when gui saves config, it will use this path.",
                        default=None)
    args = parser.parse_args()
    characters = np.load(args.chars_path)
    if args.num_chars >= len(characters):
        raise ValueError("Index out of bound. Valid range: 0 ~ %d" % (len(characters)))

    rospy.init_node("collect_writing_data", anonymous=True)

    if args.num_chars > 0:
        characters = characters[:args.num_chars]
    begin_procedure(characters[:args.num_chars], pens.str_to_pen(args.pen),
                    args.dim, args.save_dirpath,
                    test_first=args.test_first, gui_config_file=args.gui_config_file)


if __name__ == "__main__":
    main()
