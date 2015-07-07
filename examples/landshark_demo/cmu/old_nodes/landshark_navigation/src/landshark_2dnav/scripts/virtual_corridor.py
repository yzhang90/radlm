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

from nav_msgs.msg import Path
from math import hypot

flag = 1
origPath = Path()
virtualCorridorThreshold = 0.1

def stop_landshark():
    
    deadmanCommandPublisher = rospy.Publisher('landshark_control/deadman', BoolStamped, queue_size=10)
    deadmanOn = BoolStamped()
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
      deadmanOn.data = bool(0)
      now = rospy.get_rostime()
      deadmanOn.header.stamp.secs = now.secs
      deadmanOn.header.stamp.nsecs = now.nsecs
      deadmanCommandPublisher.publish(deadmanOn)
      r.sleep()

def virtual_corridor(gPath):
    
    global flag
    global origPath

    if flag == 1:
        origPath = gPath
        flag = 0
    else:
        
	print "Comparing the new Global Path with the Original Global Path. . ."

	if len(gPath.poses) >= len(origPath.poses):
            offset = len(gPath.poses) - len(origPath.poses)
	    for i in range(0, len(origPath.poses)):
	        if hypot(origPath.poses[i].pose.position.x - gPath.poses[i+offset].pose.position.x , \
                         origPath.poses[i].pose.position.y - gPath.poses[i+offset].pose.position.y) > virtualCorridorThreshold:
                    print "Stopping Robot!"
                    stop_landshark()
                   
                   
        elif len(gPath.poses) < len(origPath.poses):
            offset = len(origPath.poses) - len(gPath.poses)
            for i in range(0, len(gPath.poses)):
                if hypot(gPath.poses[i].pose.position.x - origPath.poses[i+offset].pose.position.x , \
                         gPath.poses[i].pose.position.y - origPath.poses[i+offset].pose.position.y) > virtualCorridorThreshold:
                    print "Stopping Robot!"        
                    stop_landshark()  
                    
    
def read_global_plans():
    rospy.Subscriber("move_base/NavfnROS/plan", Path, virtual_corridor)
    rospy.spin()

if __name__ == '__main__':
 
    rospy.init_node('virtual_corridor', anonymous=True)
    read_global_plans()

