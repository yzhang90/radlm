#!/usr/bin/env python

import roslib 
import rospy

from std_msgs.msg import UInt8

import sys

class Request():
  def __init__(self):
    rospy.init_node('pub_pp_request')
    self.pubPPRequest = rospy.Publisher('/landshark_sim/pp_request', UInt8, queue_size=10)
    self.msgPPRequest = UInt8()
    self.msgPPRequest.data = int(sys.argv[1])

  def publish(self):
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      self.pubPPRequest.publish(self.msgPPRequest)
      r.sleep()
    


#PPRequest { FIELDS
#  NONE: uint8 0
#  ENGAGE: uint8 1
#  DISENGAGE: uint8 2
#}

if __name__ == '__main__':
  try:
    myRequest = Request()
    myRequest.publish()
  except rospy.ROSInterruptException:
    rospy.loginfo("landshark_2dnav failed")
