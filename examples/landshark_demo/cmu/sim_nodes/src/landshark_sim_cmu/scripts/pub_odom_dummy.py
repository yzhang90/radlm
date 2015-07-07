#!/usr/bin/env python

import roslib #; roslib.load_manifest('rbx1_nav')
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

from std_msgs.msg import UInt8

def odomPub():

#  ppRequestPublisher = rospy.Publisher('/radl/pp_request', NavSatFix, queue_size=10)

#  msgPPRequest = UInt8

  gpsPublisher = rospy.Publisher('/landshark/gps', NavSatFix, queue_size=10)
  odomPublisher = rospy.Publisher('/landshark/odom', Odometry, queue_size=10)
  imuPublisher = rospy.Publisher('/landshark/imu', Imu, queue_size=10)
  rospy.init_node('odom_stub_pub', anonymous=True)

#pp_request

#gps
  msgGps = NavSatFix()
  msgGps.latitude = 37.4571014863
  msgGps.longitude = -122.173496991
  msgGps.altitude = 0.0
#odom
  msgOdom = Odometry()
  msgOdom.header.frame_id = '/odom';
  msgOdom.child_frame_id = '/base_footprint';
  msgOdom.pose.pose.position.x = 0.0;
  msgOdom.pose.pose.position.y = 0.0;
  msgOdom.pose.pose.position.z = 0.0;
  msgOdom.pose.pose.orientation.x = 0.998864293098;
  msgOdom.pose.pose.orientation.y = -0.034317471087;
  msgOdom.pose.pose.orientation.z = -0.0326555036008;
  msgOdom.pose.pose.orientation.w = 0.00509702041745;
  msgOdom.twist.twist.linear.x = 0.0;
  msgOdom.twist.twist.linear.y = 0.0;
  msgOdom.twist.twist.linear.z = 0.0;
  msgOdom.twist.twist.angular.x = -0.00636286381632
  msgOdom.twist.twist.angular.y = 0.00192605413031
  msgOdom.twist.twist.angular.z = 0.000487280223751;
  msgOdom.pose.covariance = [0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03];


#  msgOdom.pose.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];


#imu
  msgImu = Imu()
  msgImu.header.frame_id = '/landshark/imu'
  msgImu.orientation.x = 0.998864293098
  msgImu.orientation.y = -0.034317471087
  msgImu.orientation.z = -0.0326555036008
  msgImu.orientation.w = 0.00509702041745

  msgImu.angular_velocity.x = -0.00636286381632
  msgImu.angular_velocity.y = 0.00192605413031
  msgImu.angular_velocity.z = 0.000487280223751

  msgImu.linear_acceleration.x = -0.735982544155
  msgImu.linear_acceleration.y = 0.0413017048039
  msgImu.linear_acceleration.z = -9.79757237505


  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    now = rospy.get_rostime()
    msgGps.header.stamp.secs = now.secs
    msgGps.header.stamp.nsecs = now.nsecs
    msgOdom.header.stamp.secs = now.secs
    msgOdom.header.stamp.nsecs = now.nsecs
    msgImu.header.stamp.secs = now.secs
    msgImu.header.stamp.nsecs = now.nsecs

#    ppRequestPublisher.publish(msgPPRequest)

    gpsPublisher.publish(msgGps)
    odomPublisher.publish(msgOdom)
    imuPublisher.publish(msgImu)
    r.sleep()

if __name__ == '__main__':
    try:
        odomPub()
    except rospy.ROSInterruptException: pass



#aravind_odom
#header: 
#  seq: 1549
#  stamp: 
#    secs: 0
#    nsecs: 0
#  frame_id: /odom
#child_frame_id: /base_footprint
#pose: 
#  pose: 
#    position: 
#      x: 0.0
#      y: 0.0
#      z: 0.0
#    orientation: 
#      x: 0.998864293098
#      y: -0.034317471087
#      z: -0.0326555036008
#      w: 0.00509702041745
#  covariance: [0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03]
#twist: 
#  twist: 
#    linear: 
#      x: 0.0
#      y: 0.0
#      z: 0.0
#    angular: 
#      x: -0.00636286381632
#      y: 0.00192605413031
#      z: 0.000487280223751
#  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#sim_gateway_odom
#header: 
#  seq: 61
#  stamp: 
#    secs: 0
#    nsecs: 0
#  frame_id: /odom
#child_frame_id: /base_footprint
#pose: 
#  pose: 
#    position: 
#      x: 0.0
#      y: 0.0
#      z: 0.0
#    orientation: 
#      x: -0.510182580947
#      y: 0.860063309735
#      z: 0.000942057061251
#      w: 0.00198742907634
#  covariance: [0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03]
#twist: 
#  twist: 
#    linear: 
#      x: 0.0
#      y: 0.0
#      z: 0.0
#    angular: 
#      x: 3.60383399149e-15
#      y: 1.7338787507e-11
#      z: -9.96876636183e-15
#  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


#aravind_imu
#header: 
#  seq: 1514
#  stamp: 
#    secs: 0
#    nsecs: 0
#  frame_id: /landshark/imu
#orientation: 
#  x: 0.998864293098
#  y: -0.034317471087
#  z: -0.0326555036008
#  w: 0.00509702041745
#orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#angular_velocity: 
#  x: -0.00636286381632
#  y: 0.00192605413031
#  z: 0.000487280223751
#angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#linear_acceleration: 
#  x: -0.735982544155
#  y: 0.0413017048039
#  z: -9.79757237505
#linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


