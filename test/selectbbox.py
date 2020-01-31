#####################################################################################################
# Copyright Info
#
# Author: Zian LI
# zian.li@mines-ales.org
# 
# This script is modified from
# https://www.learnopencv.com/object-tracking-using-opencv-cpp-python/ 
# created by Satya Mallick
#
#####################################################################################################

import cv2
import sys

""" 
    Usage : $ python3 selectbbox.py VIDEOPATH
    Select the rectangle box containing the object to be tracked

    This script works in accordance with test.sh
"""

if __name__ == '__main__' :

    vidpath = sys.argv[1]

    # Read video
    # path = "/home/lizian/Projects/EMA-2A-Mission-MOVCAP/prototype/env/samples/"
    video = cv2.VideoCapture(vidpath)
 
    # Exit if video not opened.
    if not video.isOpened():
        print("Could not open video")
        sys.exit()
 
    # Read first frame.
    ok, frame = video.read()
    if not ok:
        print("Cannot read video file")
        sys.exit()
     
    # Select the bounding bbox
    bbox = cv2.selectROI(frame, False)
    bbox = str(bbox).replace(' ','')
    
    print(bbox)

