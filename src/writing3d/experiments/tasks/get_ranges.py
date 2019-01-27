#!/usr/bin/env python


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

indices = [4 ,
           40 ,
           59 ,
           70 ,
           156,
           184,
           195,
           210,
           236,
           269,
           277,
           356,
           397,
           440,
           454,
           560,
           600,
           715,
           763,
           777]


def get_stroke_ranges(character):
    """A character is a list of strokes"""
    allmax = np.max(character[0], axis=0)
    allmin = np.min(character[0], axis=0)
    for stroke in character[1:]:
        allmax = np.max(np.vstack((allmax, stroke)), axis=0)
        allmin = np.min(np.vstack((allmin, stroke)), axis=0)
    return allmin, allmax

def obtain_ranges(characters):
    allmin, allmax = get_stroke_ranges(characters[0])
    for i, ch in enumerate(characters):
        if i not in indices:
            stroke_min, stroke_max = get_stroke_ranges(ch)
            allmax = np.max(np.vstack((allmax, stroke_max)), axis=0)
            allmin = np.min(np.vstack((allmin, stroke_min)), axis=0)
    return allmin, allmax


def main():
    parser = argparse.ArgumentParser(description='Output range of values in each dimension given a dataset.')
    parser.add_argument("chars_path", type=str, help="Path to a .npy file that contains characters"\
                        "data (each character is list of strokes, each of which is a list of waypoints).")
    args = parser.parse_args()
    characters = np.load(args.chars_path)
    allmin, allmax = obtain_ranges(characters)
    print(allmin)
    print(allmax)

if __name__ == "__main__":
    main()
