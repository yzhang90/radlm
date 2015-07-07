#!/usr/bin/env python
# 
# Copyright (c) 2012 SRI International
# 
# The material contained in this release is copyrighted. It may not be copied, 
# reproduced, translated, reverse engineered, modified or reduced to any 
# electronic medium or machine-readable form without the prior written consent 
# of SRI International.
#
# Portions of files in this release may be unpublished work 
# containing SRI International CONFIDENTIAL AND PROPRIETARY INFORMATION. 
# Disclosure, use, reverse engineering, modification, or reproduction without
# written authorization of SRI International is likewise prohibited.
#
# Author(s): Thomas de Candia (thomasd@ai.sri.com)
#            changes by Jason Larkin (jason.larkin@spiralgen.com) (Phase 2)
# 
# $Id$
#

import roslib
#roslib.load_manifest('landshark_navigation')
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
from tf.transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid, Path

from std_msgs.msg import Bool, UInt8
from landshark_2dnav.msg import LocalPPGoal, LocalPPAction

import actionlib

import math
import signal
import sys
import threading
    
class LandsharkWayPointController:
  def __init__(self):
# Orientation estimation parameters
    self.gpsPosition = geometry_msgs.msg.Point()
    self.odomPosition = geometry_msgs.msg.Point()
    self.gpsLock = threading.Lock()
    self.orientation = 0.0
    self.gpsOrientation = 0.0
    self.odometryOrientation = 0.0
    self.odometryLock = threading.Lock()
    self.previousOdometryOrientation = 0.0
    self.gpsStraightPointList = []
    self.straightRatioThreshold = 1.0
    self.minStraightDisplacement = 0.25
# Control parameters
    self.headingControlOnlyErrorThreshold = 0.2
    self.maxControlAngularVelocity = 0.3 #ADJUST
    self.angularVelocityKp = self.maxControlAngularVelocity / self.headingControlOnlyErrorThreshold
    self.translationalVelocitKp = self.angularVelocityKp / 2.0
    self.maxControlTranslationalVelocity = 0.6 #ADJUST
    self.distanceControlThreshold = self.maxControlTranslationalVelocity / self.translationalVelocitKp
    self.minDistanceToGoal = 1.0 #ADJUST
  
    self.WAY_POINT_LIST=[]

  def Initialize(self):
    rospy.init_node('landshark_2dnav_local_pp')
    
    rospy.Subscriber('/landshark/gps_meters', geometry_msgs.msg.PointStamped, self.getGpsMeters)
    rospy.Subscriber('/radl/odom', nav_msgs.msg.Odometry, self.OdometryCallback)
    self.pubBaseVelocity = rospy.Publisher('/landshark_2dnav/base_velocity', geometry_msgs.msg.TwistStamped, queue_size=10)
# pub deadman for gateway_pp_sim
    self.pubDeadman = rospy.Publisher('/landshark_2dnav/deadman',Bool,queue_size=10)
    self.msgDeadman = Bool
    self.msgDeadman = bool(0)
    self.pubDeadman.publish(self.msgDeadman)
# offer LocalPPAction for navigate_vanilla
    self._action_name = 'landshark_2dnav/local_pp/start'
    self._as = actionlib.SimpleActionServer(self._action_name, LocalPPAction, execute_cb=self.startLocalPP, auto_start = False)
    self._as.start()

    rospy.loginfo("vanilla_way_point_controller")
    rospy.spin()
    

  def path_cb(self, path):
#    rospy.loginfo("I am in path_cb")
    self.path = path

  def getPPRequest(self,msg):
    self.msgPPRequest = msg

  def startLocalPP(self, goal):
    rospy.loginfo("startLocalPP")
    self.cancel = False
    self.WAY_POINT_LIST=[]
    if not hasattr(goal.path, 'poses'):
      rospy.loginfo("global plan does not exist")
      return
    else:
      for pose in goal.path.poses:
        x = pose.pose.position.x 
        y = pose.pose.position.y 
        self.WAY_POINT_LIST.append((x, y))  
      rospy.loginfo("I am at ProcessWayPoint")         
      self.ProcessWayPoints(self.WAY_POINT_LIST)
        
  def ProcessWayPoints(self, wayPointList):
    for wayPoint in wayPointList:
      while not self.HasWayPointBeenReached(wayPoint) and not rospy.is_shutdown() and not self.cancel:
        if self._as.is_preempt_requested():
          rospy.loginfo('%s: Preempted' % self._action_name)
          self._as.set_preempted()
          self.cancel = True
# Orientation Estimate 
        self.gpsLock.acquire()    
        if len(self.gpsStraightPointList) > 1:
#            print 'gps'
          self.gpsOrientation = math.atan2(self.gpsStraightPointList[1][1] - self.gpsStraightPointList[0][1], self.gpsStraightPointList[1][0] - self.gpsStraightPointList[0][0])
          self.previousOdometryOrientation = self.odometryOrientation
          self.orientation = self.gpsOrientation
        else:
#            print'odom'
          self.odometryLock.acquire()
          self.orientation = NormalizeAngle(self.gpsOrientation + (self.odometryOrientation - self.previousOdometryOrientation))
          self.odometryLock.release()
          
        self.gpsLock.release()
# Control 
        errorX = wayPoint[0] - self.gpsPosition.x
        errorY = wayPoint[1] - self.gpsPosition.y
        distanceError2 = errorX * errorX + errorY * errorY
        
        desiredHeading = math.atan2(errorY, errorX)
        headingError = NormalizeAngle(desiredHeading - self.orientation)
        
#        print 'Current Pose: {0:f}, {1:f}, Waypoint: {2:f}, {3:f}, Distance Error: {4:f}'.format(self.gpsPosition.x, self.gpsPosition.y, wayPoint[0], wayPoint[1], math.sqrt(distanceError2))
#        print 'Current Orientation {0:f}, Desired Orientation: {1:f}, heading error {2:f}'.format(self.orientation * 180.0 / math.pi, desiredHeading * 180.0 / math.pi, headingError * 180.0 / math.pi) 
        
        desiredVelocity = geometry_msgs.msg.TwistStamped()
        
        if headingError > self.headingControlOnlyErrorThreshold:
          desiredVelocity.twist.angular.z = self.maxControlAngularVelocity
        else:
          desiredVelocity.twist.angular.z = self.angularVelocityKp * headingError
          if distanceError2 > (self.distanceControlThreshold * self.distanceControlThreshold):
            desiredVelocity.twist.linear.x = self.maxControlTranslationalVelocity
          else:
            desiredVelocity.twist.linear.x = self.translationalVelocitKp * math.sqrt(distanceError2)
        
        self.pubBaseVelocity.publish(desiredVelocity)
        self.msgDeadman = bool(1)
        self.pubDeadman.publish(self.msgDeadman)
        rospy.sleep(.05)
        
      rospy.loginfo('Processed {0:f}, {1:f}'.format(wayPoint[0], wayPoint[1]))
    desiredVelocity = geometry_msgs.msg.TwistStamped() 
    self.pubBaseVelocity.publish(desiredVelocity)
    self.msgDeadman = bool(0)
    self.pubDeadman.publish(self.msgDeadman)   
    if not self.cancel: #FIX ME
      self._as.set_succeeded() 

    
  def HasWayPointBeenReached(self, wayPoint):
    errorX = wayPoint[0] - self.gpsPosition.x
    errorY = wayPoint[1] - self.gpsPosition.y
    distanceError2 = errorX * errorX + errorY * errorY
    
    if distanceError2 < (self.minDistanceToGoal * self.minDistanceToGoal):
      return True
    else:
      return False
  
  def getGpsMeters(self, gpsXyz):
    self.gpsLock.acquire()
    self.gpsPosition = gpsXyz.point
    self.gpsLock.release()
    
  def OdometryCallback(self, odometry):
    tfQuaternion = [odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w]
    angles = euler_from_quaternion(tfQuaternion)
    
    self.odometryLock.acquire()
    self.odometryOrientation = angles[2]
    self.odometryLock.release()

    if odometry.twist.twist.angular.z == 0:
      return
    
    translationalVelocity = math.sqrt(odometry.twist.twist.linear.x*odometry.twist.twist.linear.x + odometry.twist.twist.linear.y*odometry.twist.twist.linear.y)
    
    self.gpsLock.acquire()

    if math.fabs(translationalVelocity / odometry.twist.twist.angular.z) >  self.straightRatioThreshold:
      listLength = len(self.gpsStraightPointList)
      if listLength == 0:
        self.gpsStraightPointList = [(self.gpsPosition.x, self.gpsPosition.y)]
      elif listLength == 1:
        if self.ArePointsFarEnough(self.gpsStraightPointList[0], (self.gpsPosition.x, self.gpsPosition.y)):
          self.gpsStraightPointList.append( (self.gpsPosition.x, self.gpsPosition.y) )
      elif self.ArePointsFarEnough(self.gpsStraightPointList[1], (self.gpsPosition.x, self.gpsPosition.y)):
          self.gpsStraightPointList.pop(0)
          self.gpsStraightPointList.append( (self.gpsPosition.x, self.gpsPosition.y) )
    else: 
      self.gpsStraightPointList = []
    
    self.gpsLock.release()
        
  def ArePointsFarEnough(self, pointA, pointB):
    distance2 = (pointA[0] - pointB[0]) * (pointA[0] - pointB[0]) + (pointA[1] - pointB[1]) * (pointA[1] - pointB[1])
    if distance2 > (self.minStraightDisplacement * self.minStraightDisplacement):
      return True
    else:
      return False

def NormalizeAngle(angle):
  if (angle <= math.pi) and (angle >= -math.pi):
    return angle
  else:
    angle = math.fmod(angle, 2 * math.pi)
    if (angle <= math.pi) and (angle >= -math.pi):
      return angle
    elif (angle > math.pi):
      return -2 * math.pi + angle
    else:
      return 2 * math.pi + angle

def sigintHandler( *args ):
  sys.stderr.write('\r')
  
if __name__ == '__main__':
  signal.signal(signal.SIGINT, sigintHandler)
  
  try:
    wayPointController = LandsharkWayPointController()
    wayPointController.Initialize()
    
  except rospy.ROSInterruptException: 
    sys.exit
  
