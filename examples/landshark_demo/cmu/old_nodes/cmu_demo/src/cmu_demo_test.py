#!/usr/bin/env python
# 
# Copyright (c) 2013 SpiralGen, Inc. 
# 
# The material contained in this release is copyrighted. It may not be copied, 
# reproduced, translated, reverse engineered, modified or reduced to any 
# electronic medium or machine-readable form without the prior written consent 
# of SpiralGen, Inc.
#
# Author(s): Jason Larkin (jason.larkin@spiralgen.com)
#            

import roslib
roslib.load_manifest('landshark_navigation')
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
import landshark_msgs.msg
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String

import cmu_dwmonitor.

import math
import signal
import sys
import threading
import re
import time

import pylab
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from matplotlib import animation


#-----------------------------------------------------------  
def publisherBaseVelocity(baseVel, angVel):
#turn deadman on, so you can move...
  deadmanCommandTopic = rospy.get_param(\
  'deadman_command_topic','/landshark_control/deadman')
  deadmanCommandPublisher = rospy.Publisher(\
  deadmanCommandTopic, landshark_msgs.msg.BoolStamped)
  deadmanOn = landshark_msgs.msg.BoolStamped()

  velocityCommandTopic = rospy.get_param(\
  'base_velocity_command_topic',\
  '/landshark_control/base_velocity')
  velocityCommandPublisher = rospy.Publisher(\
  velocityCommandTopic, geometry_msgs.msg.TwistStamped) 
  desiredVelocity = geometry_msgs.msg.TwistStamped()
  
  topicDwmonitorSwitch = rospy.get_param(\
  'deadman_command_topic','/cmu_dwmonitor/dwswitch')
  deadmanCommandPublisher = rospy.Publisher(\
  topicDwmonitorSwitch, landshark_msgs.msg.BoolStamped)
  
  
  
  /cmu_dwmonitor/dwswitch
  /cmu_dwmonitor/minSpeed
  /cmu_dwmonitor/obstacle
  /cmu_dwmonitor/safeSpeed
  /cmu_dwmonitor/status
  /landshark/ExecutionMonitor/AnomalyMonitor
  
  r = rospy.Rate(30.0)
  while not rospy.is_shutdown():
    deadmanOn.data = bool(1)
    now = rospy.get_rostime()
    deadmanOn.header.stamp.secs = now.secs
    deadmanOn.header.stamp.nsecs = now.nsecs
    desiredVelocity.header.stamp.secs = now.secs
    desiredVelocity.header.stamp.nsecs = now.nsecs
    desiredVelocity.twist.angular.z = float(angVel)
    desiredVelocity.twist.linear.x = float(baseVel)
    velocityCommandPublisher.publish(desiredVelocity)
    deadmanCommandPublisher.publish(deadmanOn)  
    r.sleep()  
    
  
if __name__ == '__main__':
  try:
    rospy.init_node('cmu_demo_test', anonymous=True)
    if len(sys.argv) < 2:
      print "input format: python pub_vel.py base_vel ang_vel deadman_only"
    else:
      publisherBaseVelocity(sys.argv[1],sys.argv[2])
  except rospy.ROSInterruptException:
    sys.exit
    
    
