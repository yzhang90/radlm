#!/usr/bin/env python
'''
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Carnegie Mellon 
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Carnegie Mellon nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Pooyan Fazli (pooyan@cmu.edu)
*
'''

import rospy
from std_msgs.msg import Bool
#from landshark_msgs.msg import BoolStamped


def deadman_publisher():
    deadmanCommandPublisher = rospy.Publisher('landshark_control/deadman', BoolStamped, queue_size=10)
#    deadmanCommandPublisher = rospy.Publisher('landshark_control/deadman', Bool, queue_size=10)
    rospy.init_node('deadman_publisher', anonymous=True)
    deadmanOn = BoolStamped()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      deadmanOn.data = bool(1)
      now = rospy.get_rostime()
      deadmanOn.header.stamp.secs = now.secs
      deadmanOn.header.stamp.nsecs = now.nsecs
      deadmanCommandPublisher.publish(deadmanOn)
      r.sleep()

if __name__ == '__main__':
    try:
        deadman_publisher()
    except rospy.ROSInterruptException: pass

