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

class generateTrajectory:
  def __init__(self):  
    self.theta = []
    self.theta_dot = []
    self.pos = []
    self.vel = []
    
  def Initialize(self, argv):
    print "Initialize", '\n'
    self.t_total = float(self.input_argv[0])
    print "t_total (s) = ", self.t_total
    self.dt_theta = float(self.input_argv[1])
    print "dt_theta (s)= ", self.dt_theta
    self.dt_vel = float(self.input_argv[2])  
    print "dt_vel (s)= ", self.dt_vel  
    self.theta_i = float(self.input_argv[3])
    print "theta_i (rads)= ", self.theta_i
    self.vel_i = float(self.input_argv[4])
    print "vel_i (m/s)= ", self.vel_i
    self.theta_theta = float(self.input_argv[5])
    print "theta_theta (rads/s)= ", self.theta_theta
    self.theta_amp = float(self.input_argv[6])
    print "theta_amp (rads/s)= ", self.theta_amp
    self.xi = float(self.input_argv[7])
    print "xi (m)= ", self.xi
    self.yi = float(self.input_argv[8])
    print "yi (m)= ", self.yi
    self.theta_whiggle = float(self.input_argv[9])
    print "theta_whiggle (rads)= ", self.theta_whiggle
    self.num_theta = int(self.t_total/self.dt_theta)
    self.num_vel = int(self.t_total/self.dt_vel)
    self.t = np.linspace(0.0, self.t_total, self.num_theta)
    self.t_vel = np.linspace(0.0,self.t_total,self.num_vel+1)
    
    self.integrate_type = argv[5]
    
    if argv[2] == 'node':
      self.subYaw()
      #self.subVel()
    elif sys.argv[2] == 'generate':
      self.generateTheta_dot(argv[3])
      self.generateVel(argv[4])    
    self.Integrate()
    return 0;
    
  def Integrate(self):
    self.integrateTheta_dot()
    self.integrateVel()
    self.plotSensors()
    #self.plotTrajectory()
    return 0;  
    
  def subYaw(self):
    #pubYawTopic = rospy.get_param('pubYaw', '/landshark_control/base_velocity')
    #rospy.init_node('pubVel')
    subVelTopic = rospy.get_param('geometry_msgs/TwistStamped', '/landshark_control/base_velocity')
    rospy.Subscriber(subVelTopic, geometry_msgs.msg.TwistStamped, self.callback)
    print "subYaw", '\n'
    print dir(geometry_msgs.msg.TwistStamped)
    print geometry_msgs.msg.TwistStamped.twist
    sys.exit(0)
    return;    
    
  def callback(data):
    print "I heard %s", data.linear.x
    
  def generateTheta_dot(self,type_theta):
    if type_theta == 'constant':
      self.theta_dot = self.theta_amp*np.linspace(1.0, 1.0, self.num_theta)
    elif type_theta == 'linear':
      self.theta_dot = self.theta_amp*np.linspace(0.0, 1.0, self.num_theta)
    elif type_theta == 'cos':
      self.theta_dot = self.theta_amp*np.cos(2*np.pi*self.theta_theta*self.t)
    elif type_theta == 'whiggle':
      self.theta_dot = self.theta_amp*np.cos(2*np.pi*self.theta_theta*self.t) + \
                       self.theta_whiggle
    return 0;  
  
  def generateVel(self, type_vel):
    if type_vel == 'constant':
      self.vel = np.linspace(1.0, 1.0, self.num_vel+1)
    elif type_vel == 'linear':
      self.vel = np.linspace(0.0, self.t_total, self.num_vel+1)
    elif type_vel == 'whiggle':
      self.vel = np.cos(2*np.pi*np.linspace(0.0, self.t_total, self.num_vel+1))
    self.vel_pad = np.linspace(1,1,self.num_theta)
    cnt = 1
    pad_len = int(self.num_theta/self.num_vel)
    for v in self.vel:
      self.vel_pad[(cnt-1)*pad_len:(cnt)*pad_len] = \
      self.vel[cnt-1]*self.vel_pad[(cnt-1)*pad_len:(cnt)*pad_len]
      cnt = cnt+1
    return 0;
    
  def integrateTheta_dot(self): #euler method
    self.theta = np.linspace(0.0,0.0,self.num_theta)
    print self.integrate_type
    if self.integrate_type == 'euler':
      print "integrate: euler", '\n'
      for idx in range(self.num_theta):
        if idx==0:
          self.theta[idx] = self.theta_i
        else:
          self.theta[idx] = self.theta[idx-1] + self.dt_theta * self.theta_dot[idx-1]   
    elif self.integrate_type == 'ab':
      print "integrate: adams-bashforth", '\n'
      for idx in range(self.num_theta):
        if idx==0:
          self.theta[idx] = self.theta_i
        elif idx==1: #euler kickstart
          self.theta[idx] = self.theta[idx-1] + self.dt_theta * self.theta_dot[idx-1]   
        else:
          self.theta[idx] = self.theta[idx-1] + \
                            (self.dt_theta/2) * \
                            (3*self.theta_dot[idx-1] - self.theta_dot[idx-2])   
    elif self.integrate_type == 'ab4':
      print "integrate: adams-bashforth 4", '\n'
      for idx in range(self.num_theta):
        if idx==0:
          self.theta[idx] = self.theta_i
        elif idx==1: #euler kickstart
          self.theta[idx] = self.theta[idx-1] + \
                            self.dt_theta * \
                            self.theta_dot[idx-1]   
        elif idx==2:
          self.theta[idx] = self.theta[idx-1] + \
                            (self.dt_theta) * \
                            ((3/2)*self.theta_dot[idx-1] - \
                            (1/2)*self.theta_dot[idx-2])   
        elif idx==3:
          self.theta[idx] = self.theta[idx-1] + \
                            (self.dt_theta) * \
                            ((23/12)*self.theta_dot[idx-1] - \
                             (4/3)*self.theta_dot[idx-2] + \
                             (5/12)*self.theta_dot[idx-3])
        elif idx==4:
          self.theta[idx] = self.theta[idx-1] + \
                            (self.dt_theta) * \
                            ((55/24)*self.theta_dot[idx-1] - \
                             (59/24)*self.theta_dot[idx-2] + \
                             (37/24)*self.theta_dot[idx-3] - \
                             (3/8)*self.theta_dot[idx-4])
        else:
          self.theta[idx] = self.theta[idx-1] + \
                            (self.dt_theta) * \
                            ((1901/720)*self.theta_dot[idx-1] - \
                             (1387/360)*self.theta_dot[idx-2] + \
                             (109/30)*self.theta_dot[idx-3] - \
                             (637/360)*self.theta_dot[idx-4] + \
                             (251/720)*self.theta_dot[idx-5])                                   
    return 0;
    
  def integrateVel(self): #euler method
    self.velx = self.vel_pad * np.cos(2*np.pi*self.theta)
    self.vely = self.vel_pad * np.sin(2*np.pi*self.theta)
    self.x = np.linspace(0.0,0.0,self.num_theta)
    self.y = np.linspace(0.0,0.0,self.num_theta)
    
    if self.integrate_type == 'euler':
      for idx in range(self.num_theta):
        if idx==0:
          self.x[idx] = self.xi
          self.y[idx] = self.yi
        else:
          self.x[idx] = self.x[idx-1] + self.dt_theta*self.velx[idx-1]
          self.y[idx] = self.y[idx-1] + self.dt_theta*self.vely[idx-1]
    elif self.integrate_type == 'ab':
      for idx in range(self.num_theta):
        if idx==0:
          self.x[idx] = self.xi
          self.y[idx] = self.yi
        elif idx==1:
          self.x[idx] = self.x[idx-1] + self.dt_theta*self.velx[idx-1]
          self.y[idx] = self.y[idx-1] + self.dt_theta*self.vely[idx-1]  
        else:
          self.x[idx] = self.x[idx-1] + \
                       (self.dt_theta/2) * \
                       (3*self.velx[idx-1] - self.velx[idx-2])
          self.y[idx] = self.y[idx-1] + \
                       (self.dt_theta/2) * \
                       (3*self.vely[idx-1] - self.vely[idx-2])
    elif self.integrate_type == 'ab4':
      print "integrate: adams-bashforth 4", '\n'
      for idx in range(self.num_theta):
        if idx==0:
          self.x[idx] = self.xi
          self.y[idx] = self.yi
        elif idx==1:
          self.x[idx] = self.x[idx-1] + self.dt_theta*self.velx[idx-1]
          self.y[idx] = self.y[idx-1] + self.dt_theta*self.vely[idx-1]  
        elif idx==2:
          self.x[idx] = self.x[idx-1] + \
                       (self.dt_theta) * \
                       ((3/2)*self.velx[idx-1] - \
                       (1/2)*self.velx[idx-2])
          self.y[idx] = self.y[idx-1] + \
                       (self.dt_theta) * \
                       ((3/2)*self.vely[idx-1] - \
                       (1/2)*self.vely[idx-2])      
        elif idx==3:
          self.x[idx] = self.x[idx-1] + \
                       (self.dt_theta) * \
                       ((23/12)*self.velx[idx-1] - \
                       (4/3)*self.velx[idx-2] + \
                       (5/12)*self.velx[idx-3])
          self.y[idx] = self.y[idx-1] + \
                       (self.dt_theta) * \
                       ((23/12)*self.vely[idx-1] - \
                       (4/3)*self.vely[idx-2] + \
                       (5/12)*self.vely[idx-3]) 
        elif idx==4:
          self.x[idx] = self.x[idx-1] + \
                       (self.dt_theta) * \
                       ((55/24)*self.velx[idx-1] - \
                       (59/24)*self.velx[idx-2] + \
                       (37/24)*self.velx[idx-3] - \
                       (3/8)*self.velx[idx-4])
          self.y[idx] = self.y[idx-1] + \
                       (self.dt_theta) * \
                       ((55/24)*self.vely[idx-1] - \
                       (59/24)*self.vely[idx-2] + \
                       (37/24)*self.vely[idx-3] - \
                       (3/8)*self.vely[idx-4])   
        else:
          self.x[idx] = self.x[idx-1] + \
                       (self.dt_theta) * \
                       ((1901/720)*self.velx[idx-1] - \
                       (1387/360)*self.velx[idx-2] + \
                       (109/30)*self.velx[idx-3] - \
                       (637/360)*self.velx[idx-4] + \
                       (251/720)*self.velx[idx-5])
          self.y[idx] = self.y[idx-1] + \
                       (self.dt_theta) * \
                       ((1901/720)*self.vely[idx-1] - \
                       (1387/360)*self.vely[idx-2] + \
                       (109/30)*self.vely[idx-3] - \
                       (637/360)*self.vely[idx-4] + \
                       (251/720)*self.vely[idx-5])                                                          
    return 0;
    
  def animatePos(self):
    return 
    
  def plotSensors(self):
    params = {'backend': 'ps',
          'axes.labelsize': 16,
          'text.fontsize': 16,
          'legend.fontsize': 10,
          'xtick.labelsize': 12,
          'ytick.labelsize': 12,
          'text.usetex': True}
    pylab.rcParams.update(params)
  
    plt.figure(1)
    ax1 = plt.subplot(711)
    plt.plot(self.t,self.theta_dot,self.t,self.theta_dot,'ro',label='$\sin(x)')
    ax2 =plt.subplot(712)
    plt.plot(self.t,self.theta,self.t,self.theta,'ro')
    ax3 =plt.subplot(713)
    plt.plot(self.t,self.vel_pad,self.t_vel,self.vel,'ro')
    ax4 =plt.subplot(714)
    plt.plot(self.t,self.velx,self.t,self.velx,'ro')
    ax5 =plt.subplot(715)
    plt.plot(self.t,self.vely,self.t,self.vely,'ro')
    ax6 =plt.subplot(716)
    plt.plot(self.t,self.x,self.t,self.x,'ro')
    ax7 =plt.subplot(717)
    plt.plot(self.t,self.y,self.t,self.y,'ro')
     
    ax1.set_ylabel('$\dot{\Theta}(t)$ (rads/s)')
    ax2.set_ylabel(r'$\Theta(t)$ (rads)')
    ax3.set_ylabel(r'$v(t)$ (m/s)')
    ax4.set_ylabel(r'$v_{x}(t)$ (m/s)')
    ax5.set_ylabel(r'$v_{y}(t)$ (m/s)')
    ax6.set_ylabel(r'$x(t)$ (m)')
    ax7.set_ylabel(r'$y(t)$ (m)')
    ax7.set_xlabel(r'$t$ (s)')
     
    plt.figure(2)
    ax8 =plt.subplot(111)
    plt.plot(self.x,self.y,'k')
    ax8.set_xlabel(r'$x(t)$ (m)')
    ax8.set_ylabel(r'$y(t)$ (m)')
    pylab.xlim([-4.0,4.0])
    pylab.ylim([-4.0,4.0])
    ax8.set_title(self.integrate_type)
    plt.show()
    
    plt.close()
     
    return 0;
     
  def init():
    line.set_data([], [])
    return line,  
    
  def returnX(self):
    return self.x 
    
  def returnY(self):
    return self.y 
    
  def animate(i,data,scat):
    x = returnX
    y = returnY
    scat.set_array(data[:i],y[:i])
    return scat,
 
  def plotTrajectory(idx):
    color_data = np.random.random((100,10))
    fig = plt.figure()
    ax = plt.axes(xlim=(0, 2), ylim=(-2, 2))
    scat = plt.scatter([], [], c=c, s=100)
    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=self.num_theta, fargs=(color_data,scat))
    plt.show()
    return 
    
  def readInput(self,fname):
    f = open(fname)
    self.input_argv = []
    for line in f:
      s = re.split("[, ]",line)
      self.input_argv.append(float(s[0])) 
    print self.input_argv
    if(len(self.input_argv) < 10):
      print(\
      "Input format is: \n", \
      "t_total (s)", '\n', \
      "dt_theta (s)", '\n', \
      "dt_vel (s)", '\n', \
      "theta_dot_i (degrees/s)", '\n', \
      "vel_i (m/s)", '\n', \
      "theta_theta (1/s) ", '\n', \
      "theta_amp (degrees/s)", '\n', \
      "xi (m)", '\n', \
      "yi (m)", '\n', \
      "whiggle (m)", '\n' )  
    return;
    
def YawPublish():
  return;
  
def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)
  
def listenerBaseVelocity():
  subVelTopic = rospy.get_param('geometry_msgs/TwistStamped', '/landshark_control/base_velocity')
  rospy.Subscriber(subVelTopic, geometry_msgs.msg.TwistStamped, printBaseVelocityCallback)
  rospy.spin()
  return;
def printBaseVelocityCallback(data):
  print dir(data)
  rospy.loginfo(rospy.get_name() + ": I heard %s" % data.twist.linear.x)
  print "This is the int: ", float(data.twist.linear.x) 
  
def listenerGpsMeters():
  subGpsMetersTopic = rospy.get_param('geometry_msgs/PointStamped', '/landshark/gps_original')
  print dir(geometry_msgs.msg)
  rospy.Subscriber(subGpsMetersTopic, geometry_msgs.msg.PointStamped, printGpsMetersCallback)  
  rospy.spin()
  return; 
def printGpsMetersCallback(data):
  #print dir(data.point.x)
  print "This is the latitude: ", data.point.x, " and the longitude: ", data.point.y
  

def listenerOdometry():
  subVelTopic = rospy.get_param('nav_msgs/Odometry', '/landshark/odom')
  print dir(geometry_msgs.msg.TwistStamped.twist)
  rospy.Subscriber(subVelTopic, geometry_msgs.msg.TwistStamped, printOdometryCallback)
  rospy.spin()
  return; 
def printOdometryCallback(data):
  print data
  rospy.loginfo(rospy.get_name() + ": I heard %s" % data.twist.position.x)  
  
def publisherBaseVelocity():
  velocityCommandTopic = rospy.get_param('base_velocity_command_topic', '/landshark_control/base_velocity')
  velocityCommandPublisher = rospy.Publisher(velocityCommandTopic, geometry_msgs.msg.TwistStamped) 
  desiredVelocity = geometry_msgs.msg.TwistStamped()
  while not rospy.is_shutdown():
    desiredVelocity.twist.angular.z = 0.1
    desiredVelocity.twist.linear.x = 0.1
    velocityCommandPublisher.publish(desiredVelocity)
  
#def listenerImu():
#  subVelTopic = rospy.get_param('geometry_msgs/TwistStamped', '/landshark_control/base_velocity')
#  rospy.Subscriber(subVelTopic, geometry_msgs.msg.TwistStamped, printSubBaseVelocityCallback)
#  rospy.spin()
#  return;  
#def printSubImuCallback(data):
#  rospy.loginfo(rospy.get_name() + ": I heard %s" % data.twist.linear.x)
#  print "This is the int: ", float(data.twist.linear.x)  
  

  
if __name__ == '__main__':
  try:
    #talker()
    rospy.init_node('track_pos_generate', anonymous=True)
    #publisherBaseVelocity()
    #listenerBaseVelocity()
    #listenerGpsMeters()
    #listenerOdometry()

    trajectory = generateTrajectory()
    trajectory.readInput(sys.argv[1])
    print sys.argv
    trajectory.Initialize(sys.argv)
  except rospy.ROSInterruptException:
    sys.exit
    
    
