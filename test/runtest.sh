#!/bin/bash

if [ $# != 2 ]; then
    echo "Bad argument, try -h for help"
fi

if [ $1 == "-h" ]||[ $1 == "--help" ]; then
    echo "Usage: ./runtest.sh VIDEOPATH RESULTFILENAME"
    echo "example: $ ./runtest.sh samples/runner1.mp4 result1.txt"
    exit
fi

vidfile=`realpath $1`
#vidfile=$1
result=$2

# test if video file is typed correctly
if ! test -f "$vidfile"; then
    echo "Video file does not exist"
    exit
fi

# now let the user select the object
bbox=`python selectbbox.py $vidfile`

#test if the result log exist
if test -f "$result"; then
    echo "Result file already exist, removing"
    rm $result
fi

# loop around the trackers
trackers=(
  "BOOSTING"
  "MIL"
  "KCF"
  "TLD"
  "MEDIANFLOW"
#  "GOTURN"
  "MOSSE"
  "CSRT"
)

TOTAL=${#trackers[@]}
RUN=1
for i in "${trackers[@]}"; do
  python test.py $i $vidfile $bbox >> $result
  echo "$RUN out of $TOTAL test case executed"
  RUN=$((RUN+1))
done
