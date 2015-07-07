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
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
import landshark_msgs.msg
import landshark_cmu_dwmonitor.msg
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String

import math
import signal
import sys
import threading
import re
import time
import subprocess
import os
import commands

import pylab
import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
from matplotlib import animation

class TestDwmonitor:
  def __init__(self): 
    self.angle = []
    self.vel = []
    self.angularVel = []
    self.x = []
    self.y = []
    self.z = [] 
    self.gps_lat = []
    self.gps_lon = []
    self.x_orientation = []
    self.y_orientation = []
    self.z_orientation = []
    self.w_orientation = []
    self.min_vel = []
    self.min_angular = []
    self.safe_vel = []
    self.safe_angular = []
    self.t_odom = []
    self.t_safe = []
    self.t_min = []
    self.t_odom_nsecs = []
    self.t_safe_nsecs = []
    self.t_min_nsecs = []
    self.switch_onoff = sys.argv[2]
    self.obstacle_coord_type = sys.argv[3]
    self.obstacle_x = float(sys.argv[4])
    self.obstacle_y = float(sys.argv[5])
    self.obstacle_z = float(sys.argv[6])
    #hrl
    self.gps_lat_hrl = []
    self.gps_lon_hrl = []
    self.gps_speed_hrl = []
    
  def flipMonitorSwitch(self):
  #turn dwmonitor switch on/off  
    switchLock = threading.Lock()
    switchLock.acquire()
    dwmonitorSwitchCommandTopic =\
    rospy.get_param('dwmonitor_switch_command_topic',\
    'landshark_control/dwmonitor/monitorSwitch')
    dwmonitorSwitchPublisher =\
    rospy.Publisher(dwmonitorSwitchCommandTopic,\
    landshark_cmu_dwmonitor.msg.dwMonitorSwitch)
    dwmonitorSwitchOnOff =\
    landshark_cmu_dwmonitor.msg.dwMonitorSwitch.dwMonitorSwitch
    if self.switch_onoff == "on":      
      cmd_onoff = 'rostopic pub /landshark_control/dwmonitor/monitorSwitch \
      landshark_cmu_dwmonitor/dwMonitorSwitch "dwMonitorSwitch: 1"'
      p1 = subprocess.Popen(cmd_onoff, stdin=subprocess.PIPE, shell=True) 
    elif self.switch_onoff == "off":
      cmd_onoff = 'rostopic pub /landshark_control/dwmonitor/monitorSwitch \
      landshark_cmu_dwmonitor/dwMonitorSwitch "dwMonitorSwitch: 0"'
      p1 = subprocess.Popen(cmd_onoff, stdin=subprocess.PIPE, shell=True)    
    rospy.loginfo(['flipSwitch: ', self.switch_onoff, ', waiting 1 sec...'])
#    rospy.sleep(1)

  def setObstacle(self):
    cmd_obstacle = []
    cmd_obstacle.append("rostopic ")
    cmd_obstacle.append("pub ")
    cmd_obstacle.append("/landshark/dwmonitor/obstacle ")
    cmd_obstacle.append("landshark_cmu_dwmonitor/obstacleMsg ")
    if self.obstacle_coord_type == 'latlon':
      cmd_obstacle.append(''.join(["'{type: 1, coordinates: {point: {x: ",\
      repr(self.obstacle_x)\
      ,", y: ",\
      repr(self.obstacle_y),\
      ", z: ",\
      repr(self.obstacle_z),"}}}'"]))
    elif self.obstacle_coord_type == 'xyz':
      cmd_obstacle.append(''.join(["'{type: 0, coordinates: {point: {x: ",\
      repr(self.obstacle_x)\
      ,", y: ",\
      repr(self.obstacle_y),\
      ", z: ",\
      repr(self.obstacle_z),"}}}'"]))
    print "cmd_obstacle: ", ''.join(cmd_obstacle), '\n'
    p1 = subprocess.Popen(''.join(cmd_obstacle), stdin=subprocess.PIPE, shell=True)
    rospy.loginfo(['set obstacle: ', self.switch_onoff, ', waiting 1 sec...'])
#    time.sleep(1)
    
  def listenGpsOdomSafeVel(self):
    subOdometryTopic = rospy.get_param('odometry_topic',\
    '/landshark/odom')
    rospy.Subscriber(subOdometryTopic, nav_msgs.msg.Odometry,\
    self.printOdometryCallback)
    subSafeSpeedTopic = rospy.get_param('safeSpeed_topic',\
    '/landshark/dwmonitor/safeSpeed')
    rospy.Subscriber(subSafeSpeedTopic,\
    geometry_msgs.msg.TwistStamped,\
    self.printSafeSpeedCallback)
    subMinSpeedTopic = rospy.get_param('minSpeed_topic',\
    '/landshark/dwmonitor/minSpeed')
    rospy.Subscriber(subMinSpeedTopic,\
    geometry_msgs.msg.TwistStamped,\
    self.printMinSpeedCallback)
    print "gpsXyzTopic: ", '\n'
    gpsXyzTopic = rospy.get_param('gps_xyz_topic', '/landshark/gps')
    rospy.Subscriber(gpsXyzTopic, sensor_msgs.msg.NavSatFix, self.printGPSCallback)
    rospy.spin()
    return
    
  def printGPSCallback(self, gpsXyz):
    gpsLock = threading.Lock()
    gpsLock.acquire()
    self.gps_lat.append(float(gpsXyz.latitude))
    self.gps_lon.append(float(gpsXyz.longitude))
#    print "this is gpsXyz.latitude :", gpsXyz.latitude,\
#    "this is gpsXyz.longitude :", gpsXyz.longitude, '\n'
    gpsLock.release()
    return
    
  def printOdometryCallback(self, data):
    odometryLock = threading.Lock()
    odometryLock.acquire()
    self.t_odom.append(data.header.stamp.secs)
    self.t_odom_nsecs.append(data.header.stamp.nsecs)
    self.vel.append(float(data.twist.twist.linear.x))
    self.angularVel.append(float(data.twist.twist.angular.z))
    self.x.append(float(data.pose.pose.position.x))
    self.y.append(float(data.pose.pose.position.y))    
    self.x_orientation.append(float(data.pose.pose.orientation.x))
    self.y_orientation.append(float(data.pose.pose.orientation.y))
    self.z_orientation.append(float(data.pose.pose.orientation.z))
    self.w_orientation.append(float(data.pose.pose.orientation.w))
    self.num_vel = len(self.vel)
    odometryLock.release()
    return  
    
  def printSafeSpeedCallback(self,data):
    safeSpeedLock = threading.Lock()
    safeSpeedLock.acquire()
    self.t_safe.append(float(data.header.stamp.secs))
    self.t_safe_nsecs.append(float(data.header.stamp.nsecs))
    self.safe_vel.append(float(data.twist.linear.x))
    self.safe_angular.append(float(data.twist.angular.z))
    safeSpeedLock.release()
    return
    
  def printMinSpeedCallback(self,data):
    minSpeedLock = threading.Lock()
    minSpeedLock.acquire()
    self.t_min.append(float(data.header.stamp.secs))
    self.t_min_nsecs.append(float(data.header.stamp.nsecs))
    self.min_vel.append(float(data.twist.linear.x))
    self.min_angular.append(float(data.twist.angular.z))
    minSpeedLock.release()
    return

  def dwmonitorCallback(data):
    print "switch on?: ", data.data, '\n'
    return 0;

  def dwmonitorObstacleCallback(data):
    data.coordinates.point.x, data.coordinates.point.y, data.coordinates.point.z, '\n'
    return 0;
    
  def transformTime(self):
    self.t_odom_norm_nsecs = []
    self.t_safe_norm_nsecs = []
    self.t_min_norm_nsecs = []
    
    min_t_odom = min(self.t_odom[:]) 
    min_t_safe = min(self.t_safe[:])
    min_t_min = min(self.t_min[:])
    
    self.t_odom_norm =\
    [t - 0.0 for t in self.t_odom]

    self.t_safe_norm =\
    [t - 0.0 for t in self.t_safe] 
    
    self.t_min_norm =\
    [t - 0.0 for t in self.t_min]
    
#    cnt = 0 
#    self.t_odom_norm_nsecs.append(self.t_odom_nsecs[0]/(1e9))
    for i in range(len(self.t_odom_nsecs)):
#      if self.t_odom_nsecs[i+1] < self.t_odom_nsecs[i]:        
#        cnt = cnt+1
      self.t_odom_norm_nsecs.append(\
      self.t_odom_norm[i] + self.t_odom_nsecs[i]/(1e9))  
    cnt = 0  
    
#    self.t_safe_norm_nsecs.append(self.t_safe_nsecs[0]/(1e9))
    for i in range(len(self.t_safe_nsecs)):
#      if self.t_safe_nsecs[i+1] < self.t_safe_nsecs[i]:        
#        cnt = cnt+1
      self.t_safe_norm_nsecs.append(\
      self.t_safe_norm[i] + self.t_safe_nsecs[i]/(1e9))   
    
#    self.t_min_norm_nsecs.append(self.t_min_nsecs[0]/(1e9))
    for i in range(len(self.t_min_nsecs)):
#      if self.t_min_nsecs[i+1] < self.t_min_nsecs[i]:        
#        cnt = cnt+1
      self.t_min_norm_nsecs.append(\
      self.t_min_norm[i] + self.t_min_nsecs[i]/(1e9))  
#    sys.exit(0) 
#    self.t_odom_norm_nsecs =\
#    [t - self.t_odom_nsecs[0] for t in self.t_odom_nsecs]

#    self.t_safe_norm_nsecs =\
#    [t - self.t_safe_nsecs[0] for t in self.t_safe_nsecs] 
#    
#    self.t_min_norm_nsecs =\
#    [t - self.t_min_nsecs[0] for t in self.t_min_nsecs]
  
    
  def plotTrajectory(self):
    print "plotTrajectory: ", '\n'
    
    self.transformTime()
    
    print "self.t_odom[0]: ", self.t_odom[0], '\n'
    print "len(self.t_odom): ", len(self.t_odom), '\n'
    print "len(self.vel): ", len(self.vel), '\n'
    print "self.t_odom_norm[0]: ", self.t_odom_norm[0], '\n'
    print "len(self.t_odom_norm): ", len(self.t_odom_norm), '\n'
    print "len(self.t_odom_nsecs): ", len(self.t_odom_nsecs), '\n'
    print "len(self.t_odom_norm_nsecs): ", len(self.t_odom_norm_nsecs), '\n'
    print "min(self.t_odom): ", min(self.t_odom), '\n'
    print "max(self.t_odom): ", max(self.t_odom), '\n'
    
    print "self.t_safe[0]: ", self.t_safe[0], '\n'
    print "len(self.t_safe): ", len(self.t_safe), '\n' 
    print "self.t_safe_norm[0]: ", self.t_safe_norm[0], '\n'
    print "len(self.t_safe_norm): ", len(self.t_safe_norm), '\n'
    print "len(self.t_safe_nsecs): ", len(self.t_safe_nsecs), '\n'
    print "min(self.t_safe): ", min(self.t_safe), '\n'
    print "max(self.t_safe): ", max(self.t_safe), '\n'  
       
    print "self.t_min[0]: ", self.t_min[0], '\n'
    print "len(self.t_min): ", len(self.t_min), '\n' 
    print "self.t_min_norm[0]: ", self.t_min_norm[0], '\n'
    print "len(self.t_min_norm): ", len(self.t_min_norm), '\n'
    print "len(self.t_min_nsecs): ", len(self.t_min_nsecs), '\n'
    print "min(self.t_min): ", min(self.t_min), '\n'  
    print "max(self.t_min): ", max(self.t_min), '\n' 
    
    self.t_safe_odom = np.linspace(0.0, max(self.t_odom_norm_nsecs),len(self.t_safe_norm_nsecs))
    self.t_min_odom = np.linspace(0.0, max(self.t_odom_norm_nsecs),len(self.t_min_norm_nsecs))     

#cmu        
#    plt.figure(1) 
#    ax1 =plt.subplot(311)
#    plt.plot(\
#    self.t_odom_norm_nsecs,\
#    self.vel[:],'ro')
#    
#    plt.figure(1) 
#    ax1 =plt.subplot(311)
#    plt.plot(\
#    self.t_safe_odom,\
#    self.safe_vel[:],'ro')
#    
#    ax1.set_xlabel(r'$t$ (s)')
#    ax1.set_ylabel(r'$v(t)$ (m)')
#    pylab.xlim(\
#    [min([min(self.t_safe_odom),min(self.t_safe_odom)]),\
#    max([max(self.t_safe_odom),max(self.t_safe_odom)])])
#    pylab.ylim([min(self.vel[:]),max(self.vel[:])])    

#    plt.figure(1) 
#    ax1 =plt.subplot(311)
#    plt.plot(\
#    self.t_min_norm_nsecs,\
#    self.min_vel[:],'ro')

    plt.figure(1) 
    ax1 =plt.subplot(311)
    plt.plot(\
    self.t_odom_norm_nsecs,\
    self.vel[:],'ro',\
    self.t_safe_norm_nsecs,\
    self.safe_vel[:],'bo',\
    self.t_min_norm_nsecs,\
    self.min_vel[:],'g-')
    
    ax1.set_xlabel(r'$t$ (s)')
    ax1.set_ylabel(r'$v(t)$ (m)')
    pylab.xlim(\
    [min([min(self.t_odom_norm_nsecs),min(self.t_safe_norm_nsecs)]),\
    max([max(self.t_odom_norm_nsecs),max(self.t_safe_norm_nsecs)])])
    pylab.ylim([min(self.vel[:]),max(self.vel[:])])
        
#    plt.figure(1) 
#    ax1 =plt.subplot(311)
#    plt.plot(\
#    self.t_odom,\
#    self.vel[:],'ro',\
#    self.t_safe,\
#    self.safe_vel[:],'bo',\
#    self.t_min,\
#    self.min_vel[:],'g-')
#    
#    ax1.set_xlabel(r'$t$ (s)')
#    ax1.set_ylabel(r'$v(t)$ (m)')
#    pylab.xlim(\
#    [min([min(self.t_odom),min(self.t_safe)]),\
#    max([max(self.t_odom),max(self.t_safe)])])
#    pylab.ylim([min(self.vel[:]),max(self.vel[:])])
    
#    sys.exit(0)
    
#sri   
##    self.t_safe_sri =\
##    [t*(1.0/len(self.safe_vel[:]))*max(self.t_odom_norm) for t in range(len(self.safe_vel[:]))]
#    self.t_safe_sri =\
#    np.linspace(0.0,max(self.t_odom_norm),len(self.safe_vel[:]))
#    print "max(self.t_odom_norm_nsecs): ", max(self.t_odom_norm), '\n'
#    print "(1.0/len(self.safe_vel[:])): ", (1.0/len(self.safe_vel[:])), '\n'
#    print "(1.0/len(self.safe_vel[:]))*max(self.t_odom_norm): ", (1.0/len(self.safe_vel[:]))*max(self.t_odom_norm), '\n'
#    print "self.t_safe_sri[:100]: ", self.t_safe_sri[:100], '\n' 
#    print "max(self.t_odom_norm): ", max(self.t_odom_norm), '\n'

#    ax1 =plt.subplot(311)
#    plt.plot(\
#    self.t_odom_norm_nsecs,\
#    self.vel[:],'ro',\
#    self.t_safe_sri,\
#    self.safe_vel[:],'bo')
    
    ax1 =plt.subplot(312)
    plt.plot(self.x[:],self.y[:],'r--',\
    self.x[0],self.y[0],'ro',\
    self.obstacle_x,self.obstacle_y,'bo')
    ax1.set_xlabel(r'$x(t)$ (m)')
    ax1.set_ylabel(r'$y(t)$ (m)')
    
    pylab.xlim([min(self.x),max(self.x)])
    pylab.ylim([min(self.y),max(self.y)])
    
    ax1 =plt.subplot(313)
    plt.plot(self.gps_lat[:],self.gps_lon[:],'r--',\
    self.gps_lat[0],self.gps_lon[0],'ro',\
    self.obstacle_x,self.obstacle_y,'bo')
    ax1.set_xlabel(r'$lat(t)$ (m)')
    ax1.set_ylabel(r'$lon(t)$ (m)')
    
    pylab.xlim([min(self.gps_lat),max(self.gps_lat)])
    pylab.ylim([min(self.gps_lon),max(self.gps_lon)])
    
    plt.show()
    plt.close()
#-----------------------------------------------------------
#HRL
#-----------------------------------------------------------    
  def listenGpsOdomSafeVel_hrl(self):
    print "gpsXyzTopic: ", '\n'
    gpsXyzTopic = rospy.get_param('gps_hrl_topic', '/abv/gps')
    rospy.Subscriber(gpsXyzTopic, sensor_msgs.msg.NavSatFix, self.printGPSCallback_hrl)
    print "gps_speed: ", '\n'
    gpsXyzTopic = rospy.get_param('gps_hrl_topic', '/abv/gps_speed')
    rospy.Subscriber(gpsXyzTopic, std_msgs.msg.Float32, self.printGPSSpeedCallback_hrl)
    
#    subOdometryTopic = rospy.get_param('odometry_topic',\
#    '/landshark/odom')
#    rospy.Subscriber(subOdometryTopic, nav_msgs.msg.Odometry,\
#    self.printOdometryCallback)
#    subSafeSpeedTopic = rospy.get_param('safeSpeed_topic',\
#    '/landshark/dwmonitor/safeSpeed')
#    rospy.Subscriber(subSafeSpeedTopic,\
#    geometry_msgs.msg.TwistStamped,\
#    self.printSafeSpeedCallback)
#    subMinSpeedTopic = rospy.get_param('minSpeed_topic',\
#    '/landshark/dwmonitor/minSpeed')
#    rospy.Subscriber(subMinSpeedTopic,\
#    geometry_msgs.msg.TwistStamped,\
#    self.printMinSpeedCallback)
    
    rospy.spin()
    return
    
  def printGPSCallback_hrl(self, gpsXyz):
    gpsLock = threading.Lock()
    gpsLock.acquire()
    self.gps_lat_hrl.append(float(gpsXyz.latitude))
    self.gps_lon_hrl.append(float(gpsXyz.longitude))
#    print "this is gpsXyz.latitude :", gpsXyz.latitude,\
#    "this is gpsXyz.longitude :", gpsXyz.longitude, '\n'
    gpsLock.release()
    return  
    
  def printGPSSpeedCallback_hrl(self, data):
    gpsSpeedLock = threading.Lock()
    gpsSpeedLock.acquire()
    self.gps_speed_hrl.append(float(data.data))
#    print "this is gpsXyz.latitude :", gpsXyz.latitude,\
#    "this is gpsXyz.longitude :", gpsXyz.longitude, '\n'
    gpsSpeedLock.release()
    return   
    
  def plotTrajectory_hrl(self):
    print "plotTrajectory: ", '\n'
    
    plt.figure(1) 
#    ax1 =plt.subplot(311)
#    plt.plot(\
#    self.t_odom_norm_nsecs,\
#    self.vel[:],'ro',\
#    self.t_safe_norm_nsecs,\
#    self.safe_vel[:],'bo',\
#    self.t_min_norm_nsecs,\
#    self.min_vel[:],'g-')
    
    ax1 = plt.subplot(311)
    plt.plot(\
    self.gps_speed_hrl,'ro')
    
    ax1 =plt.subplot(312)
    plt.plot(self.gps_lat_hrl[:],self.gps_lon_hrl[:],'ro',\
    self.gps_lat_hrl[0],self.gps_lon_hrl[0],'bo',\
    self.obstacle_x,self.obstacle_y,'bo')
    ax1.set_xlabel(r'$lat(t)$ (m)')
    ax1.set_ylabel(r'$lon(t)$ (m)')
    
    pylab.xlim([min(self.gps_lat_hrl),max(self.gps_lat_hrl)])
    pylab.ylim([min(self.gps_lon_hrl),max(self.gps_lon_hrl)])
    
    plt.show()
    plt.close()
    
#-----------------------------------------------------------
if __name__ == '__main__':
  try:
    rospy.init_node('test_dwmonitor_hrl', anonymous=True)
    if len(sys.argv) < 1:
      print "input format: python test_dwmonitor.py \
      on/off latlon/meters obstacle_x obstacle_y obstacle_z"
    else:
      if sys.argv[1] == 'landshark':
        test = TestDwmonitor()
        test.flipMonitorSwitch()
        test.setObstacle()
        test.listenGpsOdomSafeVel()
        test.plotTrajectory()
      elif sys.argv[1] == 'hrl':
        test = TestDwmonitor()
        test.flipMonitorSwitch()
        test.setObstacle()
        test.listenGpsOdomSafeVel_hrl()
        test.plotTrajectory_hrl()
        
#      test.publisherBaseVelocity(sys.argv[1],sys.argv[2])
  except rospy.ROSInterruptException:
    sys.exit
#-----------------------------------------------------------    
    
   
