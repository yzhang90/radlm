#!/usr/bin/env python

import roslib #; roslib.load_manifest('rbx1_nav')
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

def odomPub():
  odomPublisher = rospy.Publisher('/landshark/odom', Odometry, queue_size=10)
  imuPublisher = rospy.Publisher('/landshark/imu', Imu, queue_size=10)
  rospy.init_node('odom_stub_pub', anonymous=True)
  msgOdom = Odometry()
  msgOdom.header.frame_id = '/odom';
  msgOdom.child_frame_id = '/base_footprint';
#set to dummy values
  msgOdom.pose.pose.position.x = 0.0;
  msgOdom.pose.pose.position.y = 0.0;
  msgOdom.pose.pose.position.z = 0.0;
  msgOdom.pose.pose.orientation.z = -0.860065561041;
  msgOdom.pose.pose.orientation.w = 0.510183526499;
  msgOdom.twist.twist.linear.x = 0.0;
  msgOdom.twist.twist.linear.y = 0.0;
  msgOdom.twist.twist.angular.z = 0.0;
  msgOdom.pose.covariance = [0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03];

#set to dummy values
  msgImu = Imu()
  msgImu.header.frame_id = '/landshark/imu'
  msgImu.orientation.x = -0.510182746082
  msgImu.orientation.y = 0.86006340058
  msgImu.orientation.z = 0.000903564151857
  msgImu.orientation.w = 0.00192253885885

  msgImu.angular_velocity.x = 0.0
  msgImu.angular_velocity.y = 0.0
  msgImu.angular_velocity.z = 0.0
  msgImu.linear_acceleration.x = 0.0
  msgImu.linear_acceleration.y = 0.0
  msgImu.linear_acceleration.z = -9.81

  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    now = rospy.get_rostime()
    msgOdom.header.stamp.secs = now.secs
    msgOdom.header.stamp.nsecs = now.nsecs
    msgImu.header.stamp.secs = now.secs
    msgImu.header.stamp.nsecs = now.nsecs
    odomPublisher.publish(msgOdom)
    imuPublisher.publish(msgImu)
    r.sleep()

if __name__ == '__main__':
    try:
        odomPub()
    except rospy.ROSInterruptException: pass
