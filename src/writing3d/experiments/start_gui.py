#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import argparse
import os
from writing3d.core.gui import WritingGui
from writing3d.robot.movo_vision import MovoKinectInterface

CURRENT_CHAR_INDX = ""

def update_stroke_images(gui, characters=None, chars_path=None):
    global CURRENT_CHAR_INDX
    """Check the characters path. Display the stroke images of the
    latest character."""
    if rospy.has_param('current_writing_character_index'):
        char_indx = rospy.get_param('current_writing_character_index')
        char_dir = "Character-%d" % char_indx
    else:
        char_dirs = os.listdir(chars_path)
        if len(char_dirs) > 0:
            char_dir = sorted(char_dirs)[-1]
            char_indx = int(char_dir.split("-")[-1])
        else:
            return
    if char_indx != CURRENT_CHAR_INDX:
        CURRENT_CHAR_INDX = char_indx
        gui.set_writing_character(characters[char_indx], char_indx, char_dir=char_dir)
    stroke_img_files = []
    for f in sorted(os.listdir(os.path.join(chars_path, char_dir))):
        if f.startswith("stroke"):
            stroke_img_files.append(f)
    if len(gui.stroke_images) != len(stroke_img_files):
        for i in range(len(gui.stroke_images), len(stroke_img_files)):
            img = cv2.imread(os.path.join(chars_path, char_dir, stroke_img_files[i]),
                             cv2.IMREAD_UNCHANGED)
            gui.add_stroke_image(img)


def run_gui(chars_path, characters, gui_config_file=None):
    gui = WritingGui(hd=True)
    gui.init()
    kinect = MovoKinectInterface()
    gui.set_kinect(kinect)
    gui.update_kinect_image_periodically(every=0.3)

    if gui_config_file is not None:
        gui.set_config_file(gui_config_file)
        if os.path.exists(gui_config_file):
            gui.load_config(gui_config_file)

    gui.register_periodic_event("update_stroke_images",
                                update_stroke_images,
                                params={
                                    'characters': characters,
                                    'chars_path': chars_path
                                })
    gui.spin()
    


def main():
    parser = argparse.ArgumentParser(description='Stand-alone GUI for display and udpates.')
    parser.add_argument("chars_dir", type=str, help="Path to a directory where the collected data is saved.")
    parser.add_argument("chars_data_path", type=str, help="Path to a .npy file that contains characters"\
                        "data (each character is list of strokes, each of which is a list of waypoints).")
    parser.add_argument("-g", "--gui-config-file", type=str, help="Path to a file that stores gui config."
                        "If the file does not exist, when gui saves config, it will use this path.",
                        default=None)
    args = parser.parse_args()
    characters = np.load(args.chars_data_path)

    rospy.init_node("writing_gui", anonymous=True)
    
    run_gui(args.chars_dir, characters, args.gui_config_file)

if __name__ == "__main__":
    main()


