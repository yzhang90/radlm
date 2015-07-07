#!/bin/bash 
source /etc/bash.bashrc
source /opt/ros/indigo/setup.bash
source /opt/hacms/cmu/setup.bash
#roslaunch landshark_2dnav landshark_2dnav.launch &

roslaunch landshark_gps_meters landshark_gps_meters.launch &
sleep 0.25s
roslaunch landshark_2dnav map_server.launch &
sleep 0.25s
roslaunch landshark_2dnav ekf_template.launch &
sleep 0.25s
roslaunch landshark_2dnav fake_localization.launch &
sleep 0.25s
roslaunch landshark_2dnav move_base_only.launch &
sleep 0.25s
roslaunch landshark_2dnav local_pp.launch &
sleep 0.25s
roslaunch landshark_2dnav control.launch &
sleep 0.25s
roslaunch landshark_description landshark_description.launch &
