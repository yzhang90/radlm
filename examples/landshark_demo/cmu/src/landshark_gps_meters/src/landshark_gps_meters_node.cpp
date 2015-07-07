/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, SpiralGen, Inc.
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
* Author: Jason Larkin (jason.larkin@spiralgen.com)
*/

#include "ros/ros.h"

#include <string>
#include <stdlib.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include "GpsProjection.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8.h>

sensor_msgs::NavSatFix msgGps;
geometry_msgs::PointStamped msgGpsMeters;
sensor_msgs::NavSatFix msgGoal;
geometry_msgs::PointStamped msgGoalMeters;
landshark::GpsProjection gpsProjection;
nav_msgs::Path msgMap;
nav_msgs::Path msgMapMeters;
std_msgs::UInt8 msgReset;

void getGps(\
const sensor_msgs::NavSatFix::ConstPtr& pMsg){msgGps = *pMsg;}

void getGoal(\
const sensor_msgs::NavSatFix::ConstPtr& pMsg){msgGoal = *pMsg;}

void getMap(const nav_msgs::Path::ConstPtr& pMsg)
{msgMap = *pMsg;}

void getReset(const std_msgs::UInt8::ConstPtr& pMsg)
{msgReset = *pMsg;}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "landshark_gps_meters_node");
  ros::NodeHandle node;

  std::string topicGps;
  std::string topicGpsMeters;

  std::string topicGoal;
  std::string topicGoalMeters;

  std::string topicMap;
  std::string topicMapMeters;

  std::string topicReset;
    
  bool latch;
  std::string paramName;
  
  double origin_lat; double origin_lon;
  double robot_meters_x; double robot_meters_y;
  double goal_meters_x; double goal_meters_y;
  double obstacle_meters_x; double obstacle_meters_y; 
    
//--------------------------------------------------------------------------------
//input params
//--------------------------------------------------------------------------------
//  node.getParam("origin_lat",origin_lat);
//  node.getParam("origin_lon",origin_lon);	
//--------------------------------------------------------------------------------
//topics  
//--------------------------------------------------------------------------------
  node.getParam("gps_topic", topicGps); 
  ROS_INFO("Subscribing to %s",topicGps.c_str());
  ros::Subscriber subGps = node.subscribe < sensor_msgs::NavSatFix > \
  (topicGps, 1, getGps);

  node.getParam("goal_topic", topicGoal); 
  ROS_INFO("Subscribing to %s",topicGoal.c_str());
  ros::Subscriber subGoal = node.subscribe < sensor_msgs::NavSatFix > \
  (topicGoal, 1, getGoal);

  node.getParam("map_topic", topicMap); 
  ROS_INFO("Subscribing to %s",topicMap.c_str());
  ros::Subscriber subMap = node.subscribe < nav_msgs::Path > \
  (topicMap, 1, getMap);

  node.getParam("gps_meters_reset_topic", topicReset); 
  ROS_INFO("Subscribing to %s",topicReset.c_str());
  ros::Subscriber subReset = node.subscribe < std_msgs::UInt8 > \
  (topicReset, 1, getReset);

//sleep to SetLocalCoordinateSystemOrigin and let topics get published to
  ros::Duration(0.5).sleep();  
  ros::spinOnce();  

//automatically set origin to current position FIX ME
  try {
    gpsProjection.SetLocalCoordinateSystemOrigin(msgGps.longitude,msgGps.latitude);
    origin_lat = msgGps.latitude; 
    origin_lon = msgGps.longitude;
    ROS_INFO("\n origin_lon = %f origin_lat = %f",origin_lon,origin_lat);
  }
  catch (std::exception& exception) {
    ROS_INFO("\nlat/lon origin not valid");
  }

  node.getParam("gps_meters_topic", topicGpsMeters);
  ROS_INFO("Publishing to %s",topicGpsMeters.c_str());
  latch = true;
  ros::Publisher pubGpsMeters = node.advertise < geometry_msgs::PointStamped > \
  (topicGpsMeters, 1, latch);     

  node.getParam("goal_meters_topic", topicGoalMeters);
  ROS_INFO("Publishing to %s",topicGoalMeters.c_str());
  latch = true;
  ros::Publisher pubGoalMeters = node.advertise < geometry_msgs::PointStamped > \
  (topicGoalMeters, 1, latch);    

  node.getParam("map_meters_topic", topicMapMeters);
  ROS_INFO("Publishing to %s",topicMapMeters.c_str());
  latch = true;
  ros::Publisher pubMapMeters = node.advertise < nav_msgs::Path > \
  (topicMapMeters, 1, latch);     



//landshark/gps is only 5 Hz  
  ros::Rate rate(10.0);
  while (ros::ok())
  {
    if (msgReset.data==1)
    {
      try {
        gpsProjection.SetLocalCoordinateSystemOrigin(msgGps.longitude,msgGps.latitude);
        origin_lat = msgGps.latitude; 
        origin_lon = msgGps.longitude;
        ROS_INFO("\n origin_lon = %f origin_lat = %f",origin_lon,origin_lat);
      }
      catch (std::exception& exception) {
        ROS_INFO("\nlat/lon origin not valid");
      }
    }
    try {
      gpsProjection.GetLocalCoordinatesFromGps(\
      (double)(msgGps.longitude),(double)(msgGps.latitude),\
      robot_meters_x,robot_meters_y);    
      msgGpsMeters.header.stamp = msgGps.header.stamp; 
      msgGpsMeters.point.x = robot_meters_x;  
      msgGpsMeters.point.y = robot_meters_y;
      msgGpsMeters.point.z = msgGps.altitude;
      pubGpsMeters.publish(msgGpsMeters);
    }
    catch (std::exception& exception) {
      ROS_INFO("\nlat/lon robot position not valid");
    }
    try {
      gpsProjection.GetLocalCoordinatesFromGps(\
        (double)(msgGoal.longitude),(double)(msgGoal.latitude),\
        goal_meters_x,goal_meters_y);    
      msgGoalMeters.header.stamp = msgGps.header.stamp; 
      msgGoalMeters.point.x = goal_meters_x;  
      msgGoalMeters.point.y = goal_meters_y;
      msgGoalMeters.point.z = msgGoal.altitude;
      pubGoalMeters.publish(msgGoalMeters);
    }
    catch (std::exception& exception) {
      ROS_INFO("\nlat/lon goal position not valid");
    }

    msgMapMeters.poses.resize( msgMap.poses.size() );
    for(int i=0; i < msgMap.poses.size(); i++)
    {
      try{ 
        gpsProjection.GetLocalCoordinatesFromGps(\
        (double)(msgMap.poses[i].pose.position.y),(double)(msgMap.poses[i].pose.position.x),\
        obstacle_meters_x,obstacle_meters_y);
        msgMapMeters.poses[i].pose.position.x = obstacle_meters_x;
        msgMapMeters.poses[i].pose.position.y = obstacle_meters_y;
        msgMapMeters.poses[i].pose.position.z = msgMap.poses[i].pose.position.z;
      }
      catch (std::exception& exception) {
        ROS_INFO("\nlat/lon obstacle position not valid");
      }
    }
    pubMapMeters.publish(msgMapMeters);

    ros::spinOnce();    
    rate.sleep();
  }
 
//--------------------------------------------------------------------------------
return 0;
}
