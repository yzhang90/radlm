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


def talker():
    pub = rospy.Publisher('chatter', String)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
