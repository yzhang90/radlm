#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image 

import matplotlib.pyplot as plt
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError


class Geometry():
  def __init__(self):
    self.bridge = CvBridge()
    rospy.init_node('cmu_geometry')
    rospy.Subscriber('/landshark/moog_left_camera',Image,self.imageCb)

#    rospy.Subscriber('/landshark/rotation',Image,self.rotationCb)

    rospy.spin()


  def imageCb(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e

    cv2.imshow("Image window", cv_image)
    cv2.imwrite("image_processed.png", cv_image)
    if cv2.waitKey(1) == 0x1b: # ESC
      print 'ESC pressed. Exiting ...'
  
#  def rotationCb(self, data):
    


if __name__ == "__main__":
  Geometry()  

