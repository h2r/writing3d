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
import os
import rospy
import subprocess
import writing3d.common as common
import writing3d.util as util
from writing3d.robot.movo_vision import MovoKinectInterface
from writing3d.core.writer import CharacterWriter, write_characters
from writing3d.core.gui import WritingGui
import writing3d.core.pens as pens

import argparse

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

def stroke_complete_cb(stroke_indx, **kwargs):
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
    

def begin_procedure(characters, pen, dimension, save_dir):

    # 1. Dip ink & 2. Dot on 4 corners
    dot_four_corners(dimension, pen)

    # 3. Start UI
    gui = WritingGui(hd=True)
    kinect = MovoKinectInterface()
    gui.set_kinect(kinect, take_picture_every=0.5)
    gui.update_kinect_image_periodically()
    #   write a test character
    util.info("Writing a character (below). Please specify bounding box in GUI.")
    gui.set_writing_character(characters[0])
    write_characters([characters[0]], retract_after_stroke=False)
    confirm = input("Confirm bounding box? [Y]")
    while confirm != "Y":
        confirm = input("Please enter Y if yes. Otherwise, adjust the bonding box.")

    # 4. start collecting data
    confirm = input("Start collecting data for %d characters? [Y]" % len(character))
    if confirm != "Y":
        util.warning("Quit.")
        return # quit
    
    for i, character in enumerate(characters):
        util.info("Starting character writer...")
        try:
            writer = CharacterWriter(character, pen=pens.SmallBrush,
                                     num_waypoints=10,  # should change to -1
                                     retract_after_stroke=True,
                                     retract_scale=1)
            writer.print_character(res=40)
            dip_pen(writer)
            get_ready(writer)
            writer.init_writers()
            util.warning("Begin writing...")
            rospy.sleep(2)
            writer.Write(stroke_complete_cb=stroke_complete_cb,
                         cb_args={
                             'gui': gui,
                             'kinect': kinect,
                             'save_dir': save_dir
                         })
            confirm = input("Continue? [Y]")
            while confirm != "Y":
                confirm = input("Please enter Y if yes.")

        except Exception as ex:
            print("Exception! %s" % ex)
            import traceback
            traceback.print_exc()

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
    args = parser.parse_args()
    characters = np.load(args.chars_path)
    if args.index >= len(characters):
        raise ValueError("Index out of bound. Valid range: 0 ~ %d" % (len(characters)))

    begin_procedure(characters, pen, args.dim, args.save_dirpath)


if __name__ == "__main__":
    pass
