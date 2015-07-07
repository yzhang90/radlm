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
# 
# $Id$
#

import roslib
roslib.load_manifest('landshark_navigation')
import rospy
import geometry_msgs.msg
import sensor_msgs.msg
import nav_msgs.msg
from tf.transformations import euler_from_quaternion

from landshark_cmu_track_pos.srv import *

import math
import signal
import sys
import threading
#--------------------------------------------------------------------------------
#WAY_POINT_LIST = [(12.0, 0.0), (12.0, -12.0), (0.0, -12.0), (0,0)]
WAY_POINT_LIST = [(22.0, -5.5), (15.5,-29.0), (-9.8,-21.5), (0.0,0.0)]
#--------------------------------------------------------------------------------    
class LandsharkWayPointController:
  def __init__(self):
    # Orientation estimation parameters
    self.gpsPosition = geometry_msgs.msg.PointStamped()
    self.waypointPosition = geometry_msgs.msg.PointStamped()
#    self.gpsPosition = sensor_msgs.msg.NavSatFix()
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
    self.headingControlOnlyErrorThreshold = 1.0
    self.maxControlAngularVelocity = 1.0
    self.angularVelocityKp =\
    self.maxControlAngularVelocity / self.headingControlOnlyErrorThreshold
    self.translationalVelocityKp = self.angularVelocityKp / 1.0
    self.maxControlTranslationalVelocity = 1.0
    self.distanceControlThreshold =\
    self.maxControlTranslationalVelocity / self.translationalVelocityKp
    self.minDistanceToGoal = 0.2
    
    print "self.straightRatioThreshold= ", self.straightRatioThreshold, '\n'
    print "self.minStraightDisplacement= ", self.minStraightDisplacement, '\n'
    print "self.headingControlOnlyErrorThreshold= ",\
    self.headingControlOnlyErrorThreshold, '\n'
    print "self.maxControlAngularVelocity= ", self.maxControlAngularVelocity, '\n'
    print "self.angularVelocityKp= ", self.angularVelocityKp, '\n'
    print "self.translationalVelocityKp= ", self.translationalVelocityKp, '\n'
    print "self.maxControlTranslationalVelocity= ", self.maxControlTranslationalVelocity, '\n'
    print "self.distanceControlThreshold= ", self.distanceControlThreshold, '\n'
    print "self.distanceControlThreshold= ", self.distanceControlThreshold, '\n'
#    sys.exit(0)
    
#--------------------------------------------------------------------------------
  def Initialize(self):
    rospy.init_node('vanilla_way_point_controller')
    
    gpsXyzTopic = rospy.get_param('gps_xyz_topic', '/landshark/gps_meters')
    odometryTopic = rospy.get_param('odometry_topic', '/landshark/odom')
    velocityCommandTopic = rospy.get_param(\
    'base_velocity_command_topic', '/landshark_control/base_velocity')
#    waypointTopic = rospy.get_param('waypoint_topic', '/landshark/waypoint')
    
    rospy.Subscriber(\
    gpsXyzTopic, geometry_msgs.msg.PointStamped, self.GpsXyzCallback)
    rospy.Subscriber(\
    odometryTopic, nav_msgs.msg.Odometry, self.OdometryCallback)
    self.velocityCommandPublisher =\
    rospy.Publisher(velocityCommandTopic, geometry_msgs.msg.TwistStamped)
#    self.waypointCommandPublisher =\
#    rospy.Publisher(waypointTopic, geometry_msgs.msg.PointStamped)
    self.ProcessWayPoint(WAY_POINT_LIST)
#--------------------------------------------------------------------------------
  def handle_wayPoints(self,req):
    return WAY_POINT_LIST
#--------------------------------------------------------------------------------    
  def ProcessWayPoint(self, wayPointList):
    s = rospy.Service(\
    '/landshark/get_waypoints',serviceWayPoints,self.handle_wayPoints)
    for wayPoint in wayPointList:
      while not self.HasWayPointBeenReached(wayPoint) and not rospy.is_shutdown():
        # Orientation Estimate 
        self.gpsLock.acquire()
        
#        self.waypointPosition.point.x = wayPoint[0]
#        self.waypointPosition.point.y = wayPoint[1]
#        self.waypointCommandPublisher.publish(self.waypointPosition)
        
        if len(self.gpsStraightPointList) > 1:
          print 'gps'
          self.gpsOrientation =\
          math.atan2(self.gpsStraightPointList[1][1] -\
          self.gpsStraightPointList[0][1],\
          self.gpsStraightPointList[1][0] -\
          self.gpsStraightPointList[0][0])
          self.previousOdometryOrientation = self.odometryOrientation
          self.orientation = self.gpsOrientation
        else:
          print'odom'
          self.odometryLock.acquire()
          self.orientation =\
          self.NormalizeAngle(self.gpsOrientation +\
          (self.odometryOrientation - self.previousOdometryOrientation))
          self.odometryLock.release()
          
        self.gpsLock.release()
        
        # Control 
        errorX = wayPoint[0] - self.gpsPosition.point.x
        errorY = wayPoint[1] - self.gpsPosition.point.y
        distanceError2 = errorX * errorX + errorY * errorY
        
        print "wayPoint[0]= ", wayPoint[0], "wayPoint[1]= ", wayPoint[1], '\n'
        
        print "self.gpsPosition.point.x = ", self.gpsPosition.point.x, '\n'
        print "self.gpsPosition.point.y = ", self.gpsPosition.point.y, '\n' 
        
        desiredHeading = math.atan2(errorY, errorX)
        headingError = self.NormalizeAngle(desiredHeading - self.orientation)
        
        print 'Current Pose: {0:f}, {1:f}, Waypoint: {2:f}, {3:f}, Distance Error: {4:f}'\
        .format(self.gpsPosition.point.x, self.gpsPosition.point.y,\
        wayPoint[0], wayPoint[1], math.sqrt(distanceError2))
        print 'Current Orientation {0:f}, Desired Orientation: {1:f}, heading error {2:f}'\
        .format(self.orientation * 180.0 / math.pi,\
        desiredHeading * 180.0 / math.pi, headingError * 180.0 / math.pi) 
        
        desiredVelocity = geometry_msgs.msg.TwistStamped()
        
        if headingError > self.headingControlOnlyErrorThreshold:
          desiredVelocity.twist.angular.z = self.maxControlAngularVelocity
        else:
          desiredVelocity.twist.angular.z = self.angularVelocityKp * headingError
          if distanceError2 > (\
          self.distanceControlThreshold * self.distanceControlThreshold):
            desiredVelocity.twist.linear.x = self.maxControlTranslationalVelocity
          else:
            desiredVelocity.twist.linear.x =\
            self.translationalVelocityKp * math.sqrt(distanceError2)
        
        self.velocityCommandPublisher.publish(desiredVelocity)
        
        print 'Desired Rot Speed: {0:f}'.format(desiredVelocity.twist.angular.z)
        
        rospy.sleep(.05)
        
      print 'Porcessed {0:f}, {1:f}'.format(wayPoint[0], wayPoint[1])
#--------------------------------------------------------------------------------
  def NormalizeAngle(self,angle):
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
#--------------------------------------------------------------------------------        
  def HasWayPointBeenReached(self, wayPoint):
    errorX = wayPoint[0] - self.gpsPosition.point.x
    errorY = wayPoint[1] - self.gpsPosition.point.y
    distanceError2 = errorX * errorX + errorY * errorY
    
    if distanceError2 < (self.minDistanceToGoal * self.minDistanceToGoal):
      return True
    else:
      return False
  
  def GpsXyzCallback(self, gpsXyz):
    self.gpsLock.acquire()
    self.gpsPosition = gpsXyz
    self.gpsLock.release()
    
  def OdometryCallback(self, odometry):
    tfQuaternion =\
    [odometry.pose.pose.orientation.x, odometry.pose.pose.orientation.y,\
    odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w]
    angles = euler_from_quaternion(tfQuaternion)
    
    self.odometryLock.acquire()
    self.odometryOrientation = angles[2]
    self.odometryLock.release()
    
    if odometry.twist.twist.angular.z == 0:
      return
    
    translationalVelocity =\
    math.sqrt(odometry.twist.twist.linear.x*odometry.twist.twist.linear.x +\
    odometry.twist.twist.linear.y*odometry.twist.twist.linear.y)
    
    self.gpsLock.acquire()
    
    if math.fabs(translationalVelocity / odometry.twist.twist.angular.z) >\
    self.straightRatioThreshold:
      listLength = len(self.gpsStraightPointList)
      if listLength == 0:
        self.gpsStraightPointList =\
        [(self.gpsPosition.point.x, self.gpsPosition.point.y)]
      elif listLength == 1:
        if self.ArePointsFarEnough(self.gpsStraightPointList[0],\
        (self.gpsPosition.point.x, self.gpsPosition.point.y)):
          self.gpsStraightPointList.append(\
          (self.gpsPosition.point.x, self.gpsPosition.point.y) )
      elif self.ArePointsFarEnough(self.gpsStraightPointList[1],\
      (self.gpsPosition.point.x, self.gpsPosition.point.y)):
          self.gpsStraightPointList.pop(0)
          self.gpsStraightPointList.append(\
          (self.gpsPosition.point.x, self.gpsPosition.point.y) )
    else: 
      self.gpsStraightPointList = []
    
    self.gpsLock.release()
#--------------------------------------------------------------------------------        
  def ArePointsFarEnough(self, pointA, pointB):
    distance2 = (pointA[0] - pointB[0]) * (pointA[0] - pointB[0]) +\
    (pointA[1] - pointB[1]) * (pointA[1] - pointB[1])
    if distance2 > (self.minStraightDisplacement * self.minStraightDisplacement):
      return True
    else:
      return False
#--------------------------------------------------------------------------------
def sigintHandler( *args ):
  sys.stderr.write('\r')
#--------------------------------------------------------------------------------
if __name__ == '__main__':
  signal.signal(signal.SIGINT, sigintHandler)
  try:
    wayPointController = LandsharkWayPointController()
    wayPointController.Initialize()
  except rospy.ROSInterruptException: 
    sys.exit
  
