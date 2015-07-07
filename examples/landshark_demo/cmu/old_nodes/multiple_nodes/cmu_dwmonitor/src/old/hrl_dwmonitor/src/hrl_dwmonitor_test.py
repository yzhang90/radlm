#!/usr/bin/env python
# 
# 
#
# Author(s): Jason Larkin (jason.larkin@spiralgen.com)
#            
#-----------------------------------------------------------
import roslib
roslib.load_manifest('landshark_navigation')
import rospy
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
import landshark_msgs.msg
import hrl_dwmonitor.msg
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
#-----------------------------------------------------------
class DwMonitor:
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
    self.lat_hrl = []
    self.lon_hrl = []
    self.gps_speed = []
    self.sv_speed = []
    self.gps_secs = []
    self.gps_nsecs = []
    
#-----------------------------------------------------------    
  def flipMonitorSwitch(self):
  #turn dwmonitor switch on/off  
    switchLock = threading.Lock()
    switchLock.acquire()
    dwmonitorSwitchCommandTopic =\
    rospy.get_param('dwmonitor_switch_command_topic',\
    'hrl/dwmonitor/switch')
    dwmonitorSwitchPublisher =\
    rospy.Publisher(dwmonitorSwitchCommandTopic,\
    hrl_dwmonitor.msg.dwMonitorSwitch)
    dwmonitorSwitchOnOff =\
    hrl_dwmonitor.msg.dwMonitorSwitch.dwMonitorSwitch
    if self.switch_onoff == "on":      
      cmd_onoff =\
      'rostopic pub /hrl/dwmonitor/switch \
      hrl_dwmonitor/dwMonitorSwitch "dwMonitorSwitch: 1"'
      p1 = subprocess.Popen(\
      cmd_onoff, stdin=subprocess.PIPE, shell=True) 
    elif self.switch_onoff == "off":
      cmd_onoff =\
      'rostopic pub /hrl/dwmonitor/monitorSwitch \
      hrl_dwmonitor/dwMonitorSwitch "dwMonitorSwitch: 0"'
      p1 = subprocess.Popen(\
      cmd_onoff, stdin=subprocess.PIPE, shell=True)    
    rospy.loginfo(['flipSwitch: ', self.switch_onoff, ', waiting 1 sec...'])
#    rospy.sleep(1)
#----------------------------------------------------------- 
  def setObstacle(self):
    cmd_obstacle = []
    cmd_obstacle.append("rostopic ")
    cmd_obstacle.append("pub ")
    cmd_obstacle.append("/hrl/dwmonitor/obstacle ")
    cmd_obstacle.append("hrl_dwmonitor/obstacleMsg ")
    if self.obstacle_coord_type == 'latlon':
      cmd_obstacle.append(\
      ''.join(["'{type: 1, coordinates: {point: {x: ",\
      repr(self.obstacle_x)\
      ,", y: ",\
      repr(self.obstacle_y),\
      ", z: ",\
      repr(self.obstacle_z),"}}}'"]))
    elif self.obstacle_coord_type == 'xyz':
      cmd_obstacle.append(\
      ''.join(["'{type: 0, coordinates: {point: {x: ",\
      repr(self.obstacle_x)\
      ,", y: ",\
      repr(self.obstacle_y),\
      ", z: ",\
      repr(self.obstacle_z),"}}}'"]))
    print "cmd_obstacle: ", ''.join(cmd_obstacle), '\n'
    p1 = subprocess.Popen(\
    ''.join(cmd_obstacle), stdin=subprocess.PIPE, shell=True)
    rospy.loginfo(['set obstacle: ', self.switch_onoff, ', waiting 1 sec...'])
#    time.sleep(1)
#----------------------------------------------------------- 
  def dwmonitorCallback(data):
    print "switch on?: ", data.data, '\n'
    return 0;
#----------------------------------------------------------- 
  def dwmonitorObstacleCallback(data):
    data.coordinates.point.x, data.coordinates.point.y, data.coordinates.point.z, '\n'
    return 0;
    
#-----------------------------------------------------------
#HRL
#-----------------------------------------------------------    
  def listenTopics(self):
    print "gpsXyzTopic: ", '\n'
    topicAbvGps = rospy.get_param('topic_hrl_gps', '/abv/gps')
    rospy.Subscriber(\
    topicAbvGps, sensor_msgs.msg.NavSatFix, self.printGPSCallback)
    print "gps_speed: ", '\n'
    topicAbvGpsSpeed = rospy.get_param('topic_hrl_gps', '/abv/gps_speed')
    rospy.Subscriber(\
    topicAbvGpsSpeed, std_msgs.msg.Float32, self.printGPSSpeedCallback)
    print "sv_speed: ", '\n'
    topicSvSpeed = rospy.get_param('topic_hrl_speed', '/sv/speed')
    rospy.Subscriber(\
    topicSvSpeed, hrl_dwmonitor.msg.Speed, self.printSvSpeed)
    
    subSafeSpeedTopic = rospy.get_param('safeSpeed_topic',\
    '/hrl/dwmonitor/safeSpeed')
    rospy.Subscriber(subSafeSpeedTopic,\
    geometry_msgs.msg.TwistStamped,\
    self.printSafeSpeedCallback)
    subMinSpeedTopic = rospy.get_param('minSpeed_topic',\
    '/hrl/dwmonitor/minSpeed')
    rospy.Subscriber(subMinSpeedTopic,\
    geometry_msgs.msg.TwistStamped,\
    self.printMinSpeedCallback)
    
    rospy.spin()
    return
#-----------------------------------------------------------       
  def printSafeSpeedCallback(self,data):
    safeSpeedLock = threading.Lock()
    safeSpeedLock.acquire()
#    self.t_safe.append(float(data.header.stamp.secs))
#    self.t_safe_nsecs.append(float(data.header.stamp.nsecs))
    self.safe_vel.append(float(data.twist.linear.x))
    self.safe_angular.append(float(data.twist.angular.z))
    safeSpeedLock.release()
    return
#-----------------------------------------------------------       
  def printMinSpeedCallback(self,data):
    minSpeedLock = threading.Lock()
    minSpeedLock.acquire()
#    self.t_min.append(float(data.header.stamp.secs))
#    self.t_min_nsecs.append(float(data.header.stamp.nsecs))
    self.min_vel.append(float(data.twist.linear.x))
    self.min_angular.append(float(data.twist.angular.z))
    minSpeedLock.release()
    return        

#-----------------------------------------------------------    
  def printGPSCallback(self, data):
    gpsLock = threading.Lock()
    gpsLock.acquire()
    self.gps_lat.append(float(data.latitude))
    self.gps_lon.append(float(data.longitude))
    self.gps_secs.append(float(data.header.stamp.secs))
    self.gps_nsecs.append(float(data.header.stamp.nsecs))
#    print "this is gpsXyz.latitude :", gpsXyz.latitude,\
#    "this is gpsXyz.longitude :", gpsXyz.longitude, '\n'
    gpsLock.release()
    return  
#-----------------------------------------------------------    
  def printGPSSpeedCallback(self, data):
    gpsSpeedLock = threading.Lock()
    gpsSpeedLock.acquire()
    self.gps_speed.append(float(data.data))
#    print "this is gpsXyz.latitude :", gpsXyz.latitude,\
#    "this is gpsXyz.longitude :", gpsXyz.longitude, '\n'
    gpsSpeedLock.release()
    return   
#-----------------------------------------------------------   
  def printSvSpeed(self, data):
#    print "dir(data): ", dir(data), '\n'
    abvSpeedLock = threading.Lock()
    abvSpeedLock.acquire()
    self.sv_speed.append(float(data.speed))
#    print "this is gpsXyz.latitude :", gpsXyz.latitude,\
#    "this is gpsXyz.longitude :", gpsXyz.longitude, '\n'
    abvSpeedLock.release()
    return   
#-----------------------------------------------------------   
  def plotTrajectory(self):
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

    self.t_safe =\
    np.linspace(min(self.gps_secs),max(self.gps_secs),len(self.safe_vel)) 

    print "len(self.gps_secs): ", len(self.gps_secs), '\n'
    print "len(self.gps_speed): ", len(self.gps_speed), '\n'
    
    print "len(self.t_safe): ", len(self.t_safe), '\n'
    print "len(self.safe_vel): ", len(self.safe_vel), '\n'
    
    print "len(self.sv_speed): ", len(self.sv_speed), '\n'
    
    self.t_sv_speed =\
    np.linspace(min(self.gps_secs),max(self.gps_secs),len(self.sv_speed))
    
#convert to m/s
    self.gps_speed = [vel*(1000.0/3600.0) for vel in self.gps_speed]
    self.sv_speed = [vel*(1000.0/3600.0) for vel in self.sv_speed]    

    ax1 = plt.subplot(211)
    plt.plot(\
    self.gps_secs[:len(self.gps_speed)],self.gps_speed,'ro',\
    self.t_sv_speed,self.sv_speed,'r+',\
    self.t_safe,self.safe_vel,'b-')
    ax1.set_xlabel(r'$t$ (secs from time stamp)')
    ax1.set_ylabel(r'$v(t)$ (m/s)') 
    pylab.ylim([\
    min(min(self.sv_speed),min(self.safe_vel)),\
    max(max(self.sv_speed),max(self.safe_vel))])
    ax1 =plt.subplot(212)
    plt.plot(\
    self.gps_lat[:],self.gps_lon[:],'r-',\
    self.gps_lat[0],self.gps_lon[0],'ro',\
    self.obstacle_x,self.obstacle_y,'bo')
    ax1.set_xlabel(r'$lat(t)$')
    ax1.set_ylabel(r'$lon(t)$')    
    pylab.xlim([min(self.gps_lat)-1.0,max(self.gps_lat)+1.0])
    pylab.ylim([min(self.gps_lon),max(self.gps_lon)])
    plt.show()
    plt.close()    
#-----------------------------------------------------------
if __name__ == '__main__':
  try:
    rospy.init_node('hrl_dwmonitor_test', anonymous=True)
    if len(sys.argv) < 1:
      print "input format: python hrl_dwmonitor_test.py \
      on/off latlon/meters obstacle_x obstacle_y obstacle_z"
    else:
      if sys.argv[1] == 'hrl':
        test = DwMonitor()
        test.flipMonitorSwitch()
        test.setObstacle()
        test.listenTopics()
        test.plotTrajectory()       
  except rospy.ROSInterruptException:
    sys.exit
#-----------------------------------------------------------    
   
