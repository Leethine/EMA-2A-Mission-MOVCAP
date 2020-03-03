#!/bin/bash

## usage:
## $ ./compile.sh srcname your/c/lib/path your/opencv/library/path
## src name without ".cpp"

fname=$1
sfname=$1".cpp"

export CPATH=$2
export LIBRARY_PATH=$3

gcc -lm -lopencv_core -lopencv_imgproc -lopencv_highgui \
 -lopencv_imgcodecs -lopencv_video -lopencv_videoio \
 -lopencv_tracking \
 -lstdc++ -o $fname $sfname

