#!/bin/bash
set -x #echo on

SRI_DIR=~/Desktop/ros_groovy_base/cmu/src/landshark_cmu_demo/launch/
ls $SRI_DIR

#http://ubuntuforums.org/showthread.php?t=1277326
#http://wiki.compiz.org/WindowMatching
xprop WM_CLASS | cut -d\" -f2


