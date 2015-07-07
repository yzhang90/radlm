#!/usr/bin/env python

import roslib #; roslib.load_manifest('rbx1_nav')
import rospy

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from landshark_2dnav.msg import LandsharkNav
from landshark_2dnav.srv import LandsharkNavWaypoint

from std_msgs.msg import Bool
from std_msgs.msg import String


class Nav():
  def __init__(self):
    rospy.init_node('landshark_2dnav_navigate')
    self.navigateStatusMsg = String()
    self.navigateStatusPub = rospy.Publisher('landshark_2dnav/status', String, queue_size=10)
#    self.navigateStatusMsg.header.stamp = rospy.Time.now()
    self.navigateStatusMsg.data = 'PENDING'
    self.navigateStatusPub.publish(self.navigateStatusMsg)
# Creates the SimpleActionClient, passing the type of the action
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
# Wait 5 seconds for the action server to become available
    self.move_base.wait_for_server(rospy.Duration(5))
    rospy.loginfo("Connected to move base server")
#offer LandsharkNavWaypoint service
    s = rospy.Service('/landshark_2dnav/waypoint', LandsharkNavWaypoint, self.LandsharkNavClient)
    rospy.on_shutdown(self.shutdown)
    rospy.spin()

  def LandsharkNavClient(self,LandsharkNavWaypoint):
  # A variable to hold the initial pose of the robot to be set by 
# the user in RViz
    self.initial_pose = PoseWithCovarianceStamped()  
#  waypoint_x = 0.0
#  waypoint_y = -10.0
# Set up the next goal location
    self.goal = MoveBaseGoal()
    self.goal.target_pose.pose.position.x = LandsharkNavWaypoint.x;
    self.goal.target_pose.pose.position.y = LandsharkNavWaypoint.y;
    self.goal.target_pose.pose.orientation.w = 1.0;
    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()
# Let the user know where the robot is going next
    rospy.loginfo("Going to: " + str(LandsharkNavWaypoint.x) + ", " + str(LandsharkNavWaypoint.y))
# Start the robot toward the next location
    self.navigateStatusMsg.data = 'ACTIVE'
    self.navigateStatusPub.publish(self.navigateStatusMsg)
    self.move_base.send_goal(self.goal)
# Allow 5 minutes to get there
    self.finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))
    if self.finished_within_time:
      rospy.loginfo("Success")
      self.navigateStatusMsg.data = 'SUCCESS'
      self.navigateStatusPub.publish(self.navigateStatusMsg)
    else:
      rospy.loginfo("Failure")
      self.navigateStatusMsg.data = 'FAILURE'
      self.navigateStatusPub.publish(self.navigateStatusMsg)
    print "self.move_base.get_result() = ", self.move_base.get_result()  # A move_base result   

  def shutdown(self):
    rospy.loginfo("Stopping the robot...")
    self.move_base.cancel_goal()
    rospy.sleep(2)
    self.cmd_vel_pub.publish(Twist())
    rospy.sleep(1)

if __name__ == '__main__':
  try:
    Nav()
  except rospy.ROSInterruptException:
    rospy.loginfo("landshark_2dnav failed")
