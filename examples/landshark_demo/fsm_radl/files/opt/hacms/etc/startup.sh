#!/bin/bash 

source /etc/bash.bashrc
source /opt/ros/indigo/setup.bash 
source /opt/hacms/landshark/setup.bash 
export ROS_HOME="/tmp/.ros"
export LOG="/tmp/landshark.log"

roscore > ${LOG} 2>&1 & 
sleep 0.5s
sleep 20s

bindir=/opt/hacms/landshark/lib/landshark/
for node in base moog turret paintball gps imu_front imu_rear hokuyo_front hokuyo_rear teleop; do 
  ${bindir}/landshark_${node} > ${LOG} 2>&1 &
  sleep 0.05s
done

