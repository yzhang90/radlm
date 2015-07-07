#!/usr/bin/env python

from Tkinter import *
import tkMessageBox

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import roslib
import rospy

from std_msgs.msg import UInt8

import threading

from random import randint

class ModeSwitcher():
  def __init__(self,master):

    self.pubPPRequest = rospy.Publisher('/landshark_2dnav/pp_request', UInt8, queue_size=10)
    self.subPPStatus = rospy.Subscriber('/radl/pp_status', UInt8, self.getPPStatus)

    self.msgPPRequest = UInt8()
    self.msgPPStatus = UInt8()

#    self.monitor()

  def monitor(self):
  
    #create the window
    self.root = Tk()

    self.root.title("RADL Demo")
    self.root.geometry("320x180")

#    #create frame to hold widgets
    self.app1 = Frame(self.root)
    self.app1.grid(sticky=N+S+E+W)

#    self.frame_a = LabelFrame(self.app1, text='', padx=5, pady=5)
#    self.frame_b = LabelFrame(self.app1, text='', padx=5, pady=5)
#    self.frame_c = LabelFrame(self.app1, text='', padx=5, pady=5)
#    self.frame_a.grid(sticky=E+W)
#    self.frame_b.grid(sticky=E+W)
#    self.frame_c.grid(sticky=E+W)

#    self.labelThreats = Label(self.app1, font=("Helvetica",20), text = "Threats")
#    self.labelThreats.grid(row=0,column=0,sticky=W,in_=self.frame_b)

#    self.varObstacle = IntVar()
#    self.checkObstacle = Checkbutton(self.app1, font=("Helvetica",12), text="Obstacle", variable=self.varObstacle,state=DISABLED)
#    self.checkObstacle.grid(row=1,column=0,sticky=W,in_=self.frame_b)
#    self.checkObstacle.toggle()

#    self.varSpoof = IntVar()
#    self.checkSpoof = Checkbutton(self.app1, font=("Helvetica",12), text="Spoof GPS", variable=self.varSpoof,command=self.driftCallBack)
#    self.checkSpoof.grid(row=2,column=0,sticky=W,in_=self.frame_b)

#    self.labelSafegaurds = Label(self.app1, font=("Helvetica",20), text = "Safeguards")
#    self.labelSafegaurds.grid(row=0,column=1,padx=10,sticky=W,in_=self.frame_b)

#    self.varAvoidance = IntVar()
#    self.checkAvoidance = Checkbutton(self.app1, font=("Helvetica",12), text="Collision Avoidance", variable=self.varAvoidance,command=self.dynamicButtonCallBack)
#    self.checkAvoidance.grid(row=1,column=1,padx=10,sticky=W,in_=self.frame_b)

#    self.varDetection = IntVar()
#    self.checkDetection = Checkbutton(self.app1, font=("Helvetica",12), text="Spoofing Detection", variable=self.varDetection,command=self.ztestButtonCallBack)
#    self.checkDetection.grid(row=2,column=1,padx=10,sticky=W,in_=self.frame_b)

#    self.labelTriggerAlarm = Label(self.app1, font=("Helvetica",20), text = "")
#    self.labelTriggerAlarm.grid(row=3,column=0,columnspan=2,pady=5,sticky=W,in_=self.frame_c)

#    r = rospy.Rate(10)
#    while not rospy.is_shutdown():
#      print 'ok...'
#      self.msgPPRequest.data = randint(0,9)
#      self.pubPPRequest.publish(self.msgPPRequest)
#      r.sleep()

    

#    rospy.spin()

    self.root.mainloop()
    self.root.quit()

#    threading.Thread(target=self.click_event_inner, args=(event,)).start()

#  def start(self):
#    self.root.mainloop()

#  def click_event_inner(self, event):
#    pan, tilt = self.panorama_coords_to_pantilt(event.x, event.y)
#                self.pan  = pan
#                self.tilt = tilt
#                self.ptu_client.send_goal(ptu_control.msg.PtuGotoGoal(pan=pan, tilt=tilt))
#                rospy.sleep(2)
#                # self.localize_in_panorama()

  def getPPStatus(self,msg):
    print "msgPPStatus.data = ", msgPPStatus.data

  def driftCallBack(self):
    print "driftCallBack"
  def dynamicButtonCallBack(self):
    print "dynamicButtonCallBack"
  def ztestButtonCallBack(self):
    print "ztestButtonCallBack"

if __name__ == "__main__":
  rospy.init_node("demo_gui")
  modeSwitcher = ModeSwitcher(None)
  modeSwitcher.monitor()

