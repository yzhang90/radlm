#!/usr/bin/env python

import roslib
#roslib.load_manifest('landshark_navigation')
import rospy

import time
import numpy as np
import matplotlib.pyplot as plt

from Tkinter import *
import tkMessageBox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped 

import signal
import sys


class PlotMap:
  def __init__(self):
    self.topicMapMeters = rospy.get_param('map_meters_topic', '/landshark/map_meters')
    self.msgMapMeters = Path()
    self.topicGoalMeters = rospy.get_param('goal_meters_topic', '/landshark/goal_meters')
    self.msgGoalMeters = PointStamped()
    self.topicOdom = rospy.get_param('odom_topic', '/radl/odom')
    self.msgOdom = Odometry()
    self.topicPlan = rospy.get_param('plan_topic', '/move_base/NavfnROS/plan')
    self.msgPlan = Path()

#    plt.axis([-50, 50, -50, 50])
    plt.ion()
#    plt.show()
    
  def Initialize(self):
    rospy.init_node('plot_map')
    rospy.Subscriber(self.topicMapMeters, Path, self.getMapMeters)
    rospy.Subscriber(self.topicOdom, Odometry, self.getOdom)
    rospy.Subscriber(self.topicGoalMeters, PointStamped, self.getGoalMeters)
    rospy.Subscriber(self.topicPlan, Path, self.getPlan)

    my_dpi = 96
    self.fig = plt.figure(figsize=(500/my_dpi, 400/my_dpi), dpi=my_dpi)
    self.ax = self.fig.add_subplot(111)
    self.ax.set_xlim( -15, 35 ) 
    self.ax.set_ylim( -15, 35 ) 

    time.sleep(0.5)

#    self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'o-')

    self.p1, = self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'go',markersize=6,label="Spoofed")   
    self.p2, = self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'bo',markersize=6,label="Unspoofed")
    self.p3, = self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'ro',markersize=6,label="Obstacle")
    self.p4, = self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'ro',markersize=20,alpha = 0.5)

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
#      x=self.msgOdom.pose.pose.position.x
#      y=self.msgOdom.pose.pose.position.y 
#      print "x = ",x,"y = ",y

      self.p1.set_ydata(self.msgOdom.pose.pose.position.x)
      self.p1.set_xdata(-1.0*self.msgOdom.pose.pose.position.y) #mirror for frame 
      x_obs = []
      y_obs = []
      for pose in self.msgMapMeters.poses:
        x_obs.append(pose.pose.position.x)
        y_obs.append(-1.0*pose.pose.position.y) #mirror for frame 
      self.p2.set_ydata(x_obs)
      self.p2.set_xdata(y_obs)         

      x_plan = []
      y_plan = [] 
      for pose in self.msgPlan.poses:
        x_plan.append(pose.pose.position.x)
        y_plan.append(pose.pose.position.y)
      self.p3.set_ydata(x_plan)
      self.p3.set_xdata(y_plan) 

      self.p4.set_ydata(self.msgGoalMeters.point.x)
      self.p4.set_xdata(-1.0*self.msgGoalMeters.point.y)
      
#      self.fig.canvas.draw()
#      plt.scatter(x,y)

      plt.draw()
      time.sleep(0.05)

      

#      self.p1.set_xdata(self.msgOdom.pose.pose.position.x)
#      self.p1.set_ydata(self.msgOdom.pose.pose.position.y)
##      self.p2.set_xdata(x_gpsMetersClean_rotated)
##      self.p2.set_ydata(y_gpsMetersClean_rotated)
##      self.p3.set_xdata(x_obstacle_rotated)
##      self.p3.set_ydata(y_obstacle_rotated)
##      self.p4.set_xdata(x_obstacle_rotated)
##      self.p4.set_ydata(y_obstacle_rotated)
#      self.fig.canvas.draw()
#      self.c.update_idletasks()

  def getMapMeters(self,msg):
    self.msgMapMeters=msg

  def getGoalMeters(self,msg):
    self.msgGoalMeters=msg

  def getPlan(self,msg):
    self.msgPlan=msg

  def getOdom(self,msg):
    self.msgOdom=msg
#    plt.scatter(x, y)
#    plt.draw()
#    time.sleep(0.05)
    

def sigintHandler( *args ):
  sys.stderr.write('\r')
  
if __name__ == '__main__':
  signal.signal(signal.SIGINT, sigintHandler)
  
  try:
    plotMap = PlotMap()
    plotMap.Initialize()
    
  except rospy.ROSInterruptException: 
    sys.exit
