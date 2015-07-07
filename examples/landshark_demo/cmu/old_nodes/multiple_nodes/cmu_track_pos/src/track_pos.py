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

#rostopic pub -1 /landshark_control/base_velocity geometry_msgs/TwistStamped '{twist: {linear: {x: 1.0, y: 0.0, z: 0.0}}}'

import roslib
roslib.load_manifest('landshark_navigation')
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
from tf.transformations import euler_from_quaternion
from std_msgs.msg import String

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

    
class trackPos:
  def __init__(self):  
    self.pos = []
    self.vel = []
    self.num_vel = 0
    self.MAX_NUM_VEL = int(sys.argv[3])
    self.angularVel = []
    self.x = []
    self.y = []
    self.gps_x = []
    self.gps_y = []
    self.angle = []
    self.x_orientation = []
    self.y_orientation = []
    self.z_orientation = []
    self.w_orientation = []    
    self.dt = 1.0/float(sys.argv[1])
    self.integrate_type = sys.argv[2]
    print "self.dt", self.dt, '\n'
     
  def listenerBaseVelocity():
    subVelTopic = rospy.get_param('geometry_msgs/TwistStamped', '/landshark_control/base_velocity')
    rospy.Subscriber(subVelTopic, geometry_msgs.msg.TwistStamped, printBaseVelocityCallback)
    rospy.spin()
    return;
  def printBaseVelocityCallback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.twist.linear.x)
    print "This is the int: ", float(data.twist.linear.x) 
  
  def listenerGpsMeters():  
    print "listenerGpsMeters", '\n' 
    gpsLock = threading.Lock()
    gpsXyzTopic = rospy.get_param('gps_xyz_topic', '/landshark/gps_original')
    rospy.Subscriber(gpsXyzTopic, geometry_msgs.msg.PointStamped, GpsXyzCallback)
    rospy.spin()
    return; 
#  def printGpsMetersCallback(data):
#    print dir(data)
#    print "This is the latitude: ", data.point.x, " and the longitude: ", data.point.y 
  
  def listenerOdometry(self):
    print "listenerOdometry", '\n' 
    subOdometryTopic = rospy.get_param('odometry_topic', '/landshark/odom')
    rospy.Subscriber(subOdometryTopic, nav_msgs.msg.Odometry, self.printOdometryCallback)
    rospy.spin()
    self.num_ang = len(self.angle)
    self.num_vel = len(self.vel)
    self.theta_i = math.atan2(self.y[1] - self.y[0], self.x[1] - self.x[0] )
    self.t = np.linspace(0.0,self.dt*self.num_vel,self.num_vel)
    return;

  def printOdometryCallback(self, data):
    print "self.MAX_NUM_VEL", self.MAX_NUM_VEL, '\n'
    print "self.num_vel", self.num_vel, '\n' 
#    print "dir(data): ", dir(data.header), '\n'
#    sys.exit(0)
    if self.num_vel < self.MAX_NUM_VEL:     
      odometryLock = threading.Lock()
      odometryLock.acquire()
      tfQuaternion = [data.pose.pose.orientation.x,\
      data.pose.pose.orientation.y,\
      data.pose.pose.orientation.z,\
      data.pose.pose.orientation.w]
      angles = euler_from_quaternion(tfQuaternion)
      odometryLock.release()
      self.angle.append(float(angles[2]))
      self.vel.append(float(data.twist.twist.linear.x))
      self.angularVel.append(float(data.twist.twist.angular.z))
      self.x.append(float(data.pose.pose.position.x))
      self.y.append(float(data.pose.pose.position.y))    
      self.x_orientation.append(float(data.pose.pose.orientation.x))
      self.y_orientation.append(float(data.pose.pose.orientation.y))
      self.z_orientation.append(float(data.pose.pose.orientation.z))
      self.w_orientation.append(float(data.pose.pose.orientation.w))
      self.num_vel = len(self.vel)
    else:
      return
      
  def listenerOdometryGPS(self):
    print "listenerOdometry", '\n' 
    subOdometryTopic = rospy.get_param('odometry_topic', '/landshark/odom')
    rospy.Subscriber(subOdometryTopic, nav_msgs.msg.Odometry, self.printOdometryCallback)
    
    print "listenerGpsMeters", '\n' 
#    gpsXyzTopic = rospy.get_param('gps_xyz_topic', '/landshark/gps')
#    rospy.Subscriber(gpsXyzTopic, sensor_msgs.msg.NavSatFix, self.printGPSCallback)
    gpsXyzTopic = rospy.get_param('gps_xyz_topic', '/landshark/gps_pose')
    rospy.Subscriber(gpsXyzTopic, nav_msgs.msg.Odometry, self.printGPSCallback)
    
    rospy.spin()
    
    self.num_ang = len(self.angle)
    self.num_vel = len(self.vel)
    self.theta_i = math.atan2(self.y[1] - self.y[0], self.x[1] - self.x[0] )
    self.t = np.linspace(0.0,self.dt*self.num_vel,self.num_vel)
    return;    
    
  def printGPSCallback(self, gpsXyz):
    gpsLock = threading.Lock()
    gpsLock.acquire()
#    print "dir(gpsXyz): ", dir(gpsXyz), '\n'
    gpsLock.release()
    self.gps_x.append(float(gpsXyz.pose.pose.position.x))
    self.gps_y.append(float(gpsXyz.pose.pose.position.y))
    print "this is gpsXyz.pose.pose.position.x :", gpsXyz.pose.pose.position.x,\
    "this is gpsXyz.pose.pose.position.y :", gpsXyz.pose.pose.position.y, '\n'

#    self.gps_x.append(float(gpsXyz.latitude))
#    self.gps_y.append(float(gpsXyz.longitude))
#    print "this is gpsXyz.latitude :", gpsXyz.latitude,\
#    "this is gpsXyz.longitude :", gpsXyz.longitude, '\n'

    sys.exit(0)  
    
  def NormalizeAngle(angle):
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
    
  def integrateVel(self): #euler method
#    print "self.vel[0]: ", self.vel[0], '\n'
#    print "self.angle[0]: ", self.angle, '\n'
#    print "np.cos: ", np.cos(2.0*np.pi*np.asarray(self.angle)), '\n'
#FIX ME: factor 0f 2.0?
    self.velx = 2.0*np.asarray(self.vel) * np.cos(np.asarray(self.angle))
    self.vely = 2.0*np.asarray(self.vel) * np.sin(np.asarray(self.angle))
    self.x_int = np.linspace(0.0,0.0,self.num_vel)
    self.y_int = np.linspace(0.0,0.0,self.num_vel)
    
    if self.integrate_type == 'euler':
      print "integrate: euler", '\n'
      for idx in range(len(self.angle)):
        if idx==0:
          self.x_int[idx] = self.x[0]
          self.y_int[idx] = self.y[0]
        else:
          self.x_int[idx] = self.x_int[idx-1] + self.dt*self.velx[idx-1]
          self.y_int[idx] = self.y_int[idx-1] + self.dt*self.vely[idx-1]
    elif self.integrate_type == 'ab':
      print "integrate: adams-bashforth", '\n'
      for idx in range(len(self.angle)):
        if idx==0:
          self.x_int[idx] = self.x[0]
          self.y_int[idx] = self.y[0]
        elif idx==1:
          self.x_int[idx] = self.x_int[idx-1] + self.dt*self.velx[idx-1]
          self.y_int[idx] = self.y_int[idx-1] + self.dt*self.vely[idx-1]  
        else:
          self.x_int[idx] = self.x_int[idx-1] + \
                       (self.dt/2) * \
                       (3*self.velx[idx-1] - self.velx[idx-2])
          self.y_int[idx] = self.y_int[idx-1] + \
                       (self.dt/2) * \
                       (3*self.vely[idx-1] - self.vely[idx-2])
    elif self.integrate_type == 'ab5':
      print "integrate: adams-bashforth 5", '\n'
      for idx in range(len(self.angle)):
        if idx==0:
          self.x_int[idx] = self.x[0]
          self.y_int[idx] = self.y[0]
        elif idx==1:
          self.x_int[idx] = self.x_int[idx-1] + self.dt*self.velx[idx-1]
          self.y_int[idx] = self.y_int[idx-1] + self.dt*self.vely[idx-1]  
        elif idx==2:
          self.x_int[idx] = self.x_int[idx-1] + \
                       (self.dt) * \
                       ((3/2)*self.velx[idx-1] - \
                       (1/2)*self.velx[idx-2])
          self.y_int[idx] = self.y_int[idx-1] + \
                       (self.dt) * \
                       ((3/2)*self.vely[idx-1] - \
                       (1/2)*self.vely[idx-2])      
        elif idx==3:
          self.x_int[idx] = self.x_int[idx-1] + \
                       (self.dt) * \
                       ((23/12)*self.velx[idx-1] - \
                       (4/3)*self.velx[idx-2] + \
                       (5/12)*self.velx[idx-3])
          self.y_int[idx] = self.y_int[idx-1] + \
                       (self.dt) * \
                       ((23/12)*self.vely[idx-1] - \
                       (4/3)*self.vely[idx-2] + \
                       (5/12)*self.vely[idx-3]) 
        elif idx==4:
          self.x_int[idx] = self.x_int[idx-1] + \
                       (self.dt) * \
                       ((55/24)*self.velx[idx-1] - \
                       (59/24)*self.velx[idx-2] + \
                       (37/24)*self.velx[idx-3] - \
                       (3/8)*self.velx[idx-4])
          self.y_int[idx] = self.y_int[idx-1] + \
                       (self.dt) * \
                       ((55/24)*self.vely[idx-1] - \
                       (59/24)*self.vely[idx-2] + \
                       (37/24)*self.vely[idx-3] - \
                       (3/8)*self.vely[idx-4])   
        else:
          self.x_int[idx] = self.x_int[idx-1] + \
                       (self.dt) * \
                       ((1901/720)*self.velx[idx-1] - \
                       (1387/360)*self.velx[idx-2] + \
                       (109/30)*self.velx[idx-3] - \
                       (637/360)*self.velx[idx-4] + \
                       (251/720)*self.velx[idx-5])
          self.y_int[idx] = self.y_int[idx-1] + \
                       (self.dt) * \
                       ((1901/720)*self.vely[idx-1] - \
                       (1387/360)*self.vely[idx-2] + \
                       (109/30)*self.vely[idx-3] - \
                       (637/360)*self.vely[idx-4] + \
                       (251/720)*self.vely[idx-5])                                                          
    return 0;  
    
  def plotTrajectory(self):
    print "plotTrajectory: ", '\n'
    print self.gps_x
  
    plt.figure(1)
    ax1 =plt.subplot(411)
    plt.plot(self.t[:],self.vel[:],'k',self.t[:],self.vel[:],'ro')
    ax1.set_xlabel(r'$t$ (s)')
    ax1.set_ylabel(r'$v(t)$ (m)')
    pylab.xlim([min(self.t[:]),max(self.t[:])])
    pylab.ylim([min(self.vel[:]),max(self.vel[:])])
    #ax1.set_title('x(t),y(t)')
    
    ax1 =plt.subplot(412)
    plt.plot(self.t[:],self.angle[:],'k',self.t[:],self.angle[:],'ro')
    ax1.set_xlabel(r'$t$ (s)')
    ax1.set_ylabel(r'$\theta(t)$ (rads)')
    pylab.xlim([min(self.t[:]),max(self.t[:])])
    pylab.ylim([min(self.angle[:]),max(self.angle[:])])
    #ax1.set_title('$\theta(t)$')
    
    ax1 =plt.subplot(413)
#    plt.plot(self.x_int,self.y_int,'k',self.x_int[0],self.y_int[0],'ro',\
#             self.x,self.y,'r--',self.x[0],self.y[0],'ro',\
#            )
    plt.plot(self.x_int,self.y_int,'k',self.x_int[0],self.y_int[0],'ro',\
             self.x,self.y,'r--',self.x[0],self.y[0],'ro',\
             self.gps_x,self.gps_y,'b--')
    ax1.set_xlabel(r'$x(t), x_{int}$ (m)')
    ax1.set_ylabel(r'$y(t), y_{int} $ (m)')
    
    pylab.xlim([min(min(self.x_int),min(self.x)),max(max(self.x_int),max(self.x),)])
    pylab.ylim([min(min(self.y_int),min(self.y)),max(max(self.y_int),max(self.y),)])
    
#    pylab.xlim([min(min(self.x_int),min(self.x),min(self.gps_x)),max(max(self.x_int),max(self.x),max(self.gps_x))])
#    pylab.ylim([min(min(self.y_int),min(self.y),min(self.gps_y)),max(max(self.y_int),max(self.x),max(self.gps_y))])
    #ax1.set_title('x_{int},y_{int}(t)')
    
    
    
    ax1 =plt.subplot(414)
    plt.plot(\
             self.t,
             np.sqrt((self.x - self.x_int)**2  + (self.y - self.y_int)**2),'k')
    ax1.set_xlabel(r'$t$ (s)')
    ax1.set_ylabel(r'$r(t)$ (m)')
    pylab.xlim([min(self.t),max(self.t)])
    pylab.ylim(\
    [\
    min(np.sqrt((self.x - self.x_int)**2  + (self.y - self.y_int)**2)),\
    max(np.sqrt((self.x - self.x_int)**2  + (self.y - self.y_int)**2))])
    
    plt.show()
    plt.close()
  
if __name__ == '__main__':
  try:
    rospy.init_node('track_pos', '\n')
    print 'sys.argv[:]: ', sys.argv[:], '\n'
    if sys.argv[4] == 'landshark':
      trackPos = trackPos()
      if sys.argv[5] == 'odom':
        trackPos.listenerOdometry()
      elif sys.argv[5] == 'gps':  
        trackPos.listenerOdometryGPS()
    elif sys.argv[4] == 'car':
      trackPos = trackPos()
      trackPos.listenerOdometryCar()   
    trackPos.integrateVel()
    trackPos.plotTrajectory()
  except rospy.ROSInterruptException:
    sys.exit
    
    
