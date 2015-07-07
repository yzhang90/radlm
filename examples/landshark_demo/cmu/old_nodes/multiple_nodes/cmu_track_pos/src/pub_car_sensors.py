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
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
from std_msgs.msg import String
import abv_msgs.msg

import math
import signal
import sys
import threading
import re
import time

  
def publisherCarSensors(baseVel, angVel):

  /abv/wheel_encoder_front abv_msgs/WheelEncoderFront
  /abv/wheel_encoder_rear abv_msgs/WheelEncoderRear
  /abv/imu abv_msgs/IMU
  
  



  velocityCommandTopic = rospy.get_param('base_velocity_command_topic', '/landshark_control/base_velocity')
  velocityCommandPublisher = rospy.Publisher(velocityCommandTopic, geometry_msgs.msg.TwistStamped) 
  desiredVelocity = geometry_msgs.msg.TwistStamped()
  while not rospy.is_shutdown():
    desiredVelocity.twist.angular.z = float(angVel)
    desiredVelocity.twist.linear.x = float(baseVel)
    velocityCommandPublisher.publish(desiredVelocity)
    rospy.sleep(0.1)
  
if __name__ == '__main__':
  try:
    rospy.init_node('pubVel', anonymous=True)
    if len(sys.argv) < 2:
      print "input format: python pubVel.py baseVel angVel"
    else:
      publisherBaseVelocity(sys.argv[1],sys.argv[2])
  except rospy.ROSInterruptException:
    sys.exit
    
    
