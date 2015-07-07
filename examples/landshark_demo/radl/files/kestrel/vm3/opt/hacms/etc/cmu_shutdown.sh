#!/bin/bash 
pkill radl
source /opt/ros/indigo/setup.bash
rosnode kill -a 
pkill ros
