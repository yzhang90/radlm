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

from nav_msgs.msg import Path, Odometry, OccupancyGrid
from geometry_msgs.msg import PointStamped 

import signal
import sys

import yaml

class PlotMap:
  def __init__(self):
    self.topicMapMeters = rospy.get_param('map_meters_topic', '/landshark/map_meters')
    self.msgMapMeters = Path()
    self.topicGoalMeters = rospy.get_param('goal_meters_topic', '/landshark/goal_meters')
    self.msgGoalMeters = PointStamped()
    self.topicOdom = rospy.get_param('odom_topic', '/radl/odom')
    self.msgOdom = Odometry()
    self.topicPlan = rospy.get_param('plan_topic', '/move_base/NavfnROS/plan')
    self.msgPlan = []
#    self.topicMap = rospy.get_param('map_pp_topic', '/map')
#    self.msgPlan = Path()

    self.land_2d_dir = sys.argv[1]
    self.file_yaml = self.land_2d_dir + '/map/map.yaml'
    rospy.loginfo("map.yaml =  %s", self.file_yaml)
    self.dataMap = self.mapMetaRead()

#    plt.axis([-50, 50, -50, 50])
    plt.ion()
#    plt.show()
    
  def Initialize(self):
    rospy.init_node('landshark_2dnav_plot')
    rospy.Subscriber('/map', OccupancyGrid, self.getMap)
    rospy.Subscriber('/landshark/map_meters', Path, self.getMapMeters)
    rospy.Subscriber('/radl/odom', Odometry, self.getOdom)
    rospy.Subscriber('/landshark/goal_meters', PointStamped, self.getGoalMeters)
    rospy.Subscriber('/landshark_2dnav/plan', Path, self.getPlan)

    my_dpi = 75
    self.fig = plt.figure(figsize=(525/my_dpi, 500/my_dpi), dpi=my_dpi)
    self.ax = self.fig.add_subplot(111)
    self.ax.set_xlim( -40, 40 ) 
    self.ax.set_ylim( -40, 40 ) 

    self.root = Tk()
#controls
    self.app1 = Frame(self.root)
    self.app1.grid(sticky=N+S+E+W)
    self.frame_a = LabelFrame(self.app1, text='', padx=5, pady=5)

#for trajecotries
    self.canvas = FigureCanvasTkAgg(self.fig,master=self.root)
    self.canvas.show()
    self.c = self.canvas.get_tk_widget()
    self.c.grid()

    time.sleep(0.5)

#    self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'o-')

    self.p1, = self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'gs',markersize=8,label="Spoofed")   
    self.p2, = self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'ro',markersize=6,label="Unspoofed")
    self.p3, = self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'bo',markersize=3,label="Obstacle")
    self.p4, = self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'bo',markersize=12,alpha = 0.5)
    self.p5, = self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'rs',markersize=3,alpha = 0.5)

    self.p6, = self.ax.plot(self.msgOdom.pose.pose.position.x,self.msgOdom.pose.pose.position.y,'ro',markersize=20,alpha = 0.5)

    r = rospy.Rate(10)
    map_check = 0
    x_plan = []
    y_plan = []
    while not rospy.is_shutdown():
      self.p1.set_ydata(1.0*self.msgOdom.pose.pose.position.x)
      self.p1.set_xdata(-1.0*self.msgOdom.pose.pose.position.y) #mirror for frame 
#      self.p1.set_xdata(1.0*self.msgOdom.pose.pose.position.x)
#      self.p1.set_ydata(1.0*self.msgOdom.pose.pose.position.y) #mirror for frame 
      x_obs = []
      y_obs = []
      for pose in self.msgMapMeters.poses:
        x_obs.append(1.0*pose.pose.position.x)
        y_obs.append(-1.0*pose.pose.position.y) #mirror for frame 
#        x_obs.append(1.0*pose.pose.position.x)
#        y_obs.append(1.0*pose.pose.position.y) #mirror for frame 
      self.p2.set_ydata(x_obs)
      self.p2.set_xdata(y_obs)    
#      self.p2.set_xdata(x_obs)
#      self.p2.set_ydata(y_obs)     
      self.p6.set_ydata(x_obs)
      self.p6.set_xdata(y_obs)     

      x_plan = []
      y_plan = []
      if hasattr(self.msgPlan,'poses'): #and map_check==0:
#        map_check=1  
        for pose in self.msgPlan.poses:
          x_plan.append(1.0*pose.pose.position.x)
          y_plan.append(-1.0*pose.pose.position.y)
#          x_plan.append(1.0*pose.pose.position.x)
#          y_plan.append(1.0*pose.pose.position.y)
        self.p3.set_ydata(x_plan)
        self.p3.set_xdata(y_plan)
#        self.p3.set_xdata(x_plan)
#        self.p3.set_ydata(y_plan) 

      self.p4.set_ydata(self.msgGoalMeters.point.x)
      self.p4.set_xdata(-1.0*self.msgGoalMeters.point.y)
#      self.p4.set_xdata(1.0*self.msgGoalMeters.point.x)
#      self.p4.set_ydata(1.0*self.msgGoalMeters.point.y)

      pts = np.asarray(self.msgMap.data) 
      (x_pts) = np.where( np.asarray(self.msgMap.data) > 1 ) 

      origin = self.dataMap['origin']
      res = self.dataMap['resolution']

      height = int(2*abs(origin[0])/res) + 1 #2* for origin centered at (0,0)
      width = int(2*abs(origin[1])/res) + 1 #2* for origin centered at (0,0)

      for idx, x in enumerate(x_pts):   
        self.p5.set_ydata( 1.0*((x - (x/height)*height)*res + origin[1])) 
        self.p5.set_xdata(-1.0*(x/width*res + origin[0]))
#        self.p5.set_xdata(1.0*((x - (x/height)*height)*res + origin[1])) 
#        self.p5.set_ydata(1.0*(x/width*res + origin[0]))

#        self.p5.set_ydata(1.0*((x - (x/21)*21)*5.0 - 50.0))
#        self.p5.set_xdata(-1.0*(x/21*5.0 - 50.0))
#      self.fig.canvas.draw()
#      plt.scatter(x,y)

      self.fig.canvas.draw()
#      self.c.update_idletasks()
      r.sleep()

#      plt.draw()
#      time.sleep(0.05)

  def mapMetaRead(self):
    f = open(self.file_yaml)
    return yaml.safe_load(f)

  def getMap(self,msg):
    self.msgMap=msg

  def getMapMeters(self,msg):
    self.msgMapMeters=msg

  def getGoalMeters(self,msg):
    self.msgGoalMeters=msg

  def getPlan(self,msg):
    self.msgPlan=msg

  def getOdom(self,msg):
    self.msgOdom=msg

def sigintHandler( *args ):
  sys.stderr.write('\r')
  
if __name__ == '__main__':
  signal.signal(signal.SIGINT, sigintHandler)
  
  try:
    plotMap = PlotMap()
    plotMap.Initialize()
    
  except rospy.ROSInterruptException: 
    sys.exit
