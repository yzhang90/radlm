#!/usr/bin/env python

import argparse

import roslib
roslib.load_manifest('upenn_controller')
import rospy
import landshark_msgs.msg

if __name__ == '__main__':
  
  parser = argparse.ArgumentParser(description='Enable/Disable Upenn Controller.')
  group = parser.add_mutually_exclusive_group()
  group.add_argument('-e', '--enable', help="Enable Upenn Controller.", action="store_true")
  group.add_argument('-d', '--disable', help="Disable Upenn Controller.", action="store_true")
  args = parser.parse_args()
  
    
  rospy.init_node('controller_test')
  controllerStatePublisher = rospy.Publisher('/landshark/upenn_controller_state/control', landshark_msgs.msg.BoolStamped) 
  
  message = landshark_msgs.msg.BoolStamped()
  message.header.stamp = rospy.get_rostime()
  message.data = True
  
  if args.disable:
    message.data = False
    
  while not rospy.is_shutdown():
    controllerStatePublisher.publish(message)
    rospy.sleep(1.0)
