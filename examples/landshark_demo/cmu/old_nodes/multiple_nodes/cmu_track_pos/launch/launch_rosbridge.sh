#!/bin/bash

source /opt/ros/groovy/setup.bash
source /home/jason/ros_groovy_base/cmu/devel/setup.sh

roslaunch rosbridge_server rosbridge_websocket.launch 
