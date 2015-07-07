#!/usr/bin/env python

import sys
import roslib
roslib.load_manifest('cmu_monitors')
import rospy
from sensor_msgs.msg import NavSatFix

if __name__ == "__main__":
    DEFAULT_LONGITUDE = -122.173497005
    DEFAULT_LATITUDE = 37.4571017234
    
    rospy.init_node('set_obstacle')

    obstaclePublisher = rospy.Publisher('/landshark/cmu_dynamic_window_monitor/obstacle', NavSatFix)

    msg = NavSatFix()

    if len(sys.argv) != 3:
        print 'you may also use arguments: ' + sys.argv[0] + ' long lat'
        msg.longitude = DEFAULT_LONGITUDE
        msg.latitude = DEFAULT_LATITUDE
    else:
        msg.longitude = float(sys.argv[1])
        msg.latitude = float(sys.argv[2])

    while not rospy.is_shutdown():
        obstaclePublisher.publish(msg)
        rospy.sleep(1)

