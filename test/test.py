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
import time

""" 
    Usage : $ python3 test.py TRACKERTYPE VIDEOPATH BBOXTUPLE
    example
      TRACKNBR : 0 ~ 6
      VIDEOPATH : ../samples/runnerX.mp4
      BBOXTUPLE : (111,222,333,444)

    This script works in accordance with test.sh
"""

if __name__ == '__main__' :

    tracker_type = str(sys.argv[1])
    vidpath = str(sys.argv[2])
    #vidpath = "../samples/runner.mp4"

    # obtain the bbox tuple from the argument which is a string
    print(sys.argv[3])
    print(len(sys.argv))
    bboxtext = str(sys.argv[3])
    bboxtext = bboxtext.replace('(','').replace(')','').replace(' ','').split(',')
    bbox = [int(i) for i in bboxtext]
    bbox = tuple(bbox)

    # Set up tracker
 
    #tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'MOSSE', 'CSRT']
    #tracker_type = tracker_types[nth_tracker]

    if tracker_type == 'BOOSTING':
        tracker = cv2.TrackerBoosting_create()
    if tracker_type == 'MIL':
        tracker = cv2.TrackerMIL_create()
    if tracker_type == 'KCF':
        tracker = cv2.TrackerKCF_create()
    if tracker_type == 'TLD':
        tracker = cv2.TrackerTLD_create()
    if tracker_type == 'MEDIANFLOW':
        tracker = cv2.TrackerMedianFlow_create()
    if tracker_type == 'GOTURN':
        ## GOTURN is not yet fully implemented, abandon
        tracker = cv2.TrackerGOTURN_create()
    if tracker_type == 'MOSSE':
        tracker = cv2.TrackerMOSSE_create()
    if tracker_type == "CSRT":
        tracker = cv2.TrackerCSRT_create()
 
    # Read video
    path = "/home/lizian/Projects/EMA-2A-Mission-MOVCAP/prototype/env/samples/"
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
     
    # Define an initial bounding box
    # bbox = (1145, 122, 131, 440)
 
    # Uncomment the line below to select a different bounding box
    #bbox = cv2.selectROI(frame, False)
 
    # Initialize tracker with first frame and bounding box
    tracker.init(frame, bbox)

    fail = 0
    tik = time.perf_counter()
    while True:
        # Read a new frame
        ok, frame = video.read()
        if not ok:
            break
         
        # Start timer
        timer = cv2.getTickCount()
 
        # Update tracker
        ok, bbox = tracker.update(frame)
 
        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
 
        # Draw bounding box
        if ok:
            # Tracking success
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        else :
            # Tracking failure
            cv2.putText(frame, "Tracking failure detected", (100,80), cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
            fail = fail + 1
 
        # Display tracker type on frame
        cv2.putText(frame, tracker_type + " Tracker", (100,20), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
     
        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
 
        # Display result
        cv2.imshow("Tracking", frame)
 
        # Exit if ESC pressed
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break

    tok = time.perf_counter()
    duration = tok - tik
    
    print("{} algorithm: {} failed loops, runtime = {}".format(tracker_type, fail, duration))


