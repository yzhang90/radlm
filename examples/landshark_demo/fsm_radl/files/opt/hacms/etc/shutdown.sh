#!/bin/bash 

source /etc/bash.bashrc
source /opt/ros/indigo/setup.bash 
export ROS_HOME="/tmp/.ros"
export LOG="/tmp/landshark.log"

# kill all ros nodes 
for i in $( rosnode list ); do
  rosnode kill $i > ${LOG} 2>&1
done

pkill rosmaster > ${LOG} 2>&1
sleep 0.05s
pkill roscore > ${LOG} 2>&1

