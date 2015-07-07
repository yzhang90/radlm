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
from std_msgs.msg import UInt8

import signal
import sys

import yaml

class PlotMap:
  def __init__(self):
    print "initilaize"
    
  def Initialize(self):
    rospy.init_node('landshark_2dnav_control_gui')

    self.root = Tk()
#controls
    self.app1 = Frame(self.root)
    self.app1.grid(sticky=N+S+E+W)

    self.frame_a = LabelFrame(self.app1, text='', padx=5, pady=5)
    self.frame_b = LabelFrame(self.app1, text='', padx=5, pady=5)
    self.frame_c = LabelFrame(self.app1, text='', padx=5, pady=5)
    self.frame_d = LabelFrame(self.app1, text='', padx=5, pady=5)
    self.frame_a.grid(sticky=E+W)
    self.frame_b.grid(sticky=E+W)
    self.frame_c.grid(sticky=E+W)
    self.frame_d.grid(sticky=E+W)

    self.labelThreats = Label(self.app1, font=("Helvetica",20), text = "Path Planner")
    self.labelThreats.grid(row=0,column=0,sticky=W,in_=self.frame_a)

    self.buttonStartPP = Button(self.app1, font=("Helvetica",12), text="Request", command=self.startPPRequest)
    self.buttonStartPP.grid(row=1,column=0,sticky=W,in_=self.frame_a)
    self.buttonStopPP = Button(self.app1, font=("Helvetica",12), text="Cancel", command=self.stopPPRequest)
    self.buttonStopPP.grid(row=1,column=1,sticky=W,in_=self.frame_a)
#    self.checkObstacle.toggle()

    self.labelPPStatusTitle = Label(self.app1, font=("Helvetica",12), text = "Status: ")
    self.labelPPStatusTitle.grid(row=2,column=0,sticky=W,in_=self.frame_b)

    self.labelPPStatus = Label(self.app1, font=("Helvetica",12), text = " ")
    self.labelPPStatus.grid(row=2,column=1,padx=10,sticky=W,in_=self.frame_b)

    self.labelMonitorDWStatusTitle = Label(self.app1, font=("Helvetica",12), text = "Monitor DW: ")
    self.labelMonitorDWStatusTitle.grid(row=3,column=0,sticky=W,in_=self.frame_c)

    self.labelMonitorDWStatus = Label(self.app1, font=("Helvetica",12), text = " ")
    self.labelMonitorDWStatus.grid(row=3,column=1,padx=10,sticky=W,in_=self.frame_c)

    self.labelMonitorFtestStatusTitle = Label(self.app1, font=("Helvetica",12), text = "Monitor Ftest: ")
    self.labelMonitorFtestStatusTitle.grid(row=4,column=0,sticky=W,in_=self.frame_d)

    self.labelMonitorFtestStatus = Label(self.app1, font=("Helvetica",12), text = " ")
    self.labelMonitorFtestStatus.grid(row=4,column=1,padx=10,sticky=W,in_=self.frame_d)

    time.sleep(0.5)


    rospy.Subscriber('/radl/pp_status', UInt8, self.getPPStatus)
    self.pubPPRequest = rospy.Publisher('/landshark_sim/pp_request', UInt8, queue_size=10)    
    self.msgPPRequest = UInt8()

    rospy.Subscriber('/radl/monitor_dw/status', UInt8, self.getMonitorDWStatus)
    rospy.Subscriber('/radl/monitor_ftest/status', UInt8, self.getMonitorFtestStatus)
    self.msgMonitorDWStatus = UInt8
    self.msgMonitorFtestStatus = UInt8

    self.root.mainloop()

#    r = rospy.Rate(10)
#    while not rospy.is_shutdown():
#      r.sleep()

  def startPPRequest(self):
    self.msgPPRequest.data = 1
    self.pubPPRequest.publish(self.msgPPRequest)
    rospy.sleep(1.0/30.0)
    self.msgPPRequest.data = 0   
    self.pubPPRequest.publish(self.msgPPRequest)

  def stopPPRequest(self):
    self.msgPPRequest.data = 2
    self.pubPPRequest.publish(self.msgPPRequest)
    rospy.sleep(1.0/1.0)
    self.msgPPRequest.data = 0   
    self.pubPPRequest.publish(self.msgPPRequest)

  def getPPStatus(self,msg):
    self.msgPPStatus = msg
    if self.msgPPStatus.data == 0:
      self.labelPPStatus.config(text="off")
    if self.msgPPStatus.data == 2:
      self.labelPPStatus.config(text="engaged")
    if self.msgPPStatus.data == 3:
      self.labelPPStatus.config(text="failure")

  def getMonitorDWStatus(self,msg):
    self.getMonitorDWStatus = msg
    if self.getMonitorDWStatus.data == 0:
      self.labelMonitorDWStatus.config(text=" ")
    if self.getMonitorDWStatus.data == 1:
      self.labelMonitorDWStatus.config(text="ESTOP")

  def getMonitorFtestStatus(self,msg):
    self.getMonitorFtestStatus = msg

def sigintHandler( *args ):
  sys.stderr.write('\r')
  
if __name__ == '__main__':
  signal.signal(signal.SIGINT, sigintHandler)
  
  try:
    plotMap = PlotMap()
    plotMap.Initialize()
    
  except rospy.ROSInterruptException: 
    sys.exit
