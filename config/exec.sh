#!/bin/bash

## usage:
##  $ ./exec.sh binary [arg1 arg2 ... argn]

binary=$1

shift

export LD_LIBRARY_PATH=/home/lizian/.local/lib64

./$binary $*

