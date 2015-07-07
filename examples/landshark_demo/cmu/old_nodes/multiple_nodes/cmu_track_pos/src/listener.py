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

import rospy
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_name() + ": I heard %s" % data.data)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", String, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
