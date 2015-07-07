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

#--------------------------------------------------------------------------------
import roslib
roslib.load_manifest('landshark_navigation')
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String

import math
import sys
import threading
import re
import time

import pylab
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from matplotlib import animation

from ros_animate import Animate
#--------------------------------------------------------------------------------
rospy.init_node('animate_waypoint_following')
#--------------------------------------------------------------------------------  
def listenerGps():  
  topicGpsMeters = rospy.get_param('gps_meters', '/landshark/gps_meters')
  rospy.Subscriber(\
  topicGpsMeters, geometry_msgs.msg.PointStamped, callbackGpsMeters)
  rospy.loginfo("Listening to %s", topicGpsMeters)
  topicGpsMetersClean =\
  rospy.get_param('gps_meters_clean', '/landshark/gps_meters_clean')
  rospy.Subscriber(\
  topicGpsMetersClean, geometry_msgs.msg.PointStamped, callbackGpsMetersClean)
  rospy.loginfo("Listening to %s", topicGpsMetersClean)
#  topicGpsMetersBias =\
#  rospy.get_param('gps_meters_bias', '/landshark/gps_meters_bias')
#  rospy.Subscriber(\
#  topicGpsMetersBias, geometry_msgs.msg.PointStamped, callbackGpsMetersBias)
#  rospy.loginfo("Listening to %s", topicGpsMetersBias)
  rospy.spin()
#--------------------------------------------------------------------------------  
def callbackGpsMeters(data):
  gps_meters_x.append(float(data.point.x))
  gps_meters_y.append(float(data.point.y))
  print "this is gps_meters_x :", data.point.x,\
  "this is gps_meters_y :", data.point.y, '\n'    
#  line1.set_xdata(gps_meters_x)
#  line1.set_ydata(gps_meters_y)
#  fig.canvas.draw()
#--------------------------------------------------------------------------------
def callbackGpsMetersClean(data):
  gps_meters_clean_x.append(float(data.point.x))
  gps_meters_clean_y.append(float(data.point.y))
  print "this is gps_meters_clean_x :", data.point.x,\
  "this is gps_meters_clean_y :", data.point.y, '\n'
#--------------------------------------------------------------------------------
  
# initialization function: plot the background of each frame
def init():
    line1.set_data([], [])
    line2.set_data([], [])
    return line1,line2

# animation function.  This is called sequentially
def animate(i):
    line2.set_data(gps_meters_x[:i], gps_meters_y[:i])
    line1.set_data(gps_meters_clean_x[:i], gps_meters_clean_y[:i])
    return line1,line2
  
  
#class AnimateTrajectory:
#  def __init__(self):  
gps_meters_x = []
gps_meters_y = []
gps_meters_clean_x = []
gps_meters_clean_y = []
gps_meters_bias_x = []
gps_meters_bias_y = []  

WAYPOINTS = [(22.0, -5.5), (15.5,-29.0), (-9.8,-21.5), (0.0,0.0)]

#try_1    
#plt.ion()
#fig = plt.figure()
#ax = fig.add_subplot(111)
#line1, =\
#ax.plot(gps_meters_x, gps_meters_y, 'r-')
#listenerGps()

  
fig = plt.figure()
ax = plt.axes(xlim=(-25, 25), ylim=(-40, 10))
line1,line2,line3,line4,line5,line6 =\
ax.plot([], [], [], [],\
WAYPOINTS[0][0], WAYPOINTS[0][1],'ro',\
WAYPOINTS[1][0], WAYPOINTS[1][1],'ro',\
WAYPOINTS[2][0], WAYPOINTS[2][1],'ro',\
WAYPOINTS[3][0], WAYPOINTS[3][1],'ro')
ax.set_xlabel(r'$gps_x$ (m)')
ax.set_ylabel(r'$gps_y$ (m)')

#WAYPOINTS[0][0], WAYPOINTS[0][1],\
#WAYPOINTS[1][0], WAYPOINTS[1][1],\
#WAYPOINTS[2][0], WAYPOINTS[2][1],\
#WAYPOINTS[3][0], WAYPOINTS[3][1])

listenerGps()

#blit=True, only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=6000, interval=10, blit=True)

anim.save('no_drift.mp4', fps=10, extra_args=['-vcodec', 'libx264'])

plt.show()



##--------------------------------------------------------------------------------    
#  def callbackGpsMetersBias(self, data):
#    self.gps_meters_bias_x.append(float(data.point.x))
#    self.gps_meters_bias_y.append(float(data.point.y))
#    print "this is gps_meters_bias_x :", data.point.x,\
#    "this is gps_meters_bias_y :", data.point.y, '\n'  
##--------------------------------------------------------------------------------
#if __name__ == '__main__':
#  try:
#    rospy.init_node('animate_waypoint_following')
##    print 'sys.argv[:]: ', sys.argv[:], '\n'
##    rospy.loginfo('sys.argv[:]= %s', sys.argv[:])
##    if sys.argv[1] == 'landshark':
#    animate = AnimateTrajectory()
#    animate.listenerGps()
#  except rospy.ROSInterruptException:
#    sys.exit
    
    
