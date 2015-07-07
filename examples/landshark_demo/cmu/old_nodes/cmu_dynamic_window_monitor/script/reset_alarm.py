#!/usr/bin/env python

import roslib
roslib.load_manifest('cmu_monitors')
import rospy
from landshark_msgs.msg import ResetAlarm

if __name__ == "__main__":
    rospy.init_node('reset_alarm')

    resetAlarmPublisher = rospy.Publisher('/landshark_control/reset_alarm', ResetAlarm)
    msg = ResetAlarm()
    
    while not rospy.is_shutdown():
        resetAlarmPublisher.publish(msg)
        rospy.sleep(0.1)
