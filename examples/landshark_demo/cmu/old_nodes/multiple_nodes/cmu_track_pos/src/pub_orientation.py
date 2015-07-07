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

import math
import signal
import sys
import threading
import re
import time

#import pylab
#import numpy as np
#import scipy as sp

class Orientation:
  def __init__(self):
    self.orientationMsg = geometry_msgs.msg.TwistStamped()
    self.odometryLock = threading.Lock()
    self.odometryOrientation = None
    self.dt = 1.0/10.0;
    self.odometryOrientation_fromPosition = None;
  
  def publisherOrientation(self):    
    orientationTopic =\
    rospy.get_param('orientation_topic', '/landshark/orientation')
    odometryTopic =\
    rospy.get_param('odometry_topic', '/landshark/odom')
    baseVelocityTopic =\
    rospy.get_param('base_velocity_topic', '/landshark_control/base_velocity')
    rospy.Subscriber(\
    odometryTopic, nav_msgs.msg.Odometry, self.OdometryCallback)
    rospy.Subscriber(\
    baseVelocityTopic, geometry_msgs.msg.TwistStamped, self.BaseVelocityCallback)
    self.orientationPublisher =\
    rospy.Publisher(orientationTopic, geometry_msgs.msg.TwistStamped)

    r = rospy.Rate(30.0)
    while not rospy.is_shutdown():
      self.orientationMsg.twist.angular.x = self.odometryOrientation
#      self.orientationMsg.twist.angular.y = self.odometryOrientation_fromPosition
#      print "\nself.orientationMsg = \n", self.orientationMsg
      self.orientationPublisher.publish(self.orientationMsg)      
      r.sleep()
        
  def OdometryCallback(self, odometry):
    tfQuaternion =\
    [odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y,\
    odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w]
    angles = euler_from_quaternion(tfQuaternion)
    self.odometryLock.acquire()
    self.odometryOrientation = angles[2]
    self.odometryLock.release() 
    if self.odometryOrientation_fromPosition is None:
      self.odometryOrientation_fromPosition = self.odometryOrientation 
    
  def BaseVelocityCallback(self, data):
    if self.odometryOrientation_fromPosition is None and self.odometryOrientation is None:
      dummy = 1
    elif self.odometryOrientation_fromPosition is None:
      self.odometryOrientation_fromPosition = self.odometryOrientation
    else: 
      angle = self.odometryOrientation_fromPosition +\
      data.twist.angular.z*self.dt
      self.odometryOrientation_fromPosition = self.NormalizeAngle(angle)
    
  def NormalizeAngle(self,angle):
    if (angle <= math.pi) and (angle >= -math.pi):
      return angle
    else:
      angle = math.fmod(angle, 2 * math.pi)
      if (angle <= math.pi) and (angle >= -math.pi):
        return angle
      elif (angle > math.pi):
        return -2 * math.pi + angle
      else:
        return 2 * math.pi + angle    
  
if __name__ == '__main__':
  try:
    rospy.init_node('pub_vel', anonymous=True)
    orientation = Orientation()
    orientation.publisherOrientation()
  except rospy.ROSInterruptException:
    sys.exit
    
    
