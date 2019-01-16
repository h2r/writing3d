#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import argparse
import os
from writing3d.core.gui import WritingGui
from writing3d.robot.movo_vision import MovoKinectInterface

CURRENT_CHAR_NAME = ""

def update_stroke_images(gui, characters=None, chars_path=None):
    global CURRENT_CHAR_NAME
    """Check the characters path. Display the stroke images of the
    latest character."""
    char_dir = sorted(os.listdir(chars_path))[-1]
    if char_dir != CURRENT_CHAR_NAME:
        CURRENT_CHAR_NAME = char_dir
        char_indx = int(char_dir.split("-")[-1])
        gui.set_writing_character(characters[char_indx])
    stroke_img_files = []
    for f in sorted(os.listdir(os.path.join(chars_path, char_dir))):
        if f.startswith("stroke"):
            stroke_img_files.append(f)
    if len(gui.stroke_images) < len(stroke_img_files):
        for i in range(len(gui.stroke_images), len(stroke_img_files)):
            img = cv2.imread(os.path.join(chars_path, char_dir, stroke_img_files[i]),
                             cv2.IMREAD_UNCHANGED)
            gui.add_stroke_image(img)


def run_gui(chars_path, characters, gui_config_file=None):
    gui = WritingGui(hd=True)
    gui.init()
    kinect = MovoKinectInterface()
    gui.set_kinect(kinect)
    gui.update_kinect_image_periodically(every=0.1)

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


