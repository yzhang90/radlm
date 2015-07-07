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

#ifndef GATEWAY_GPS_H
#define GATEWAY_GPS_H

#pragma once
#include RADL_HEADER

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt8.h>

class GatewayGps {
public:
  GatewayGps() 
    : map_size_( *RADL_THIS->array_size )
  {
    pubReset = h.advertise<std_msgs::UInt8>("/radl/gps_meters/reset", 2); 
    pubGps = h.advertise<sensor_msgs::NavSatFix>("/radl/gps", 2);
    subGpsMeters = h.subscribe("/landshark/gps_meters", 2, &GatewayGps::getGpsMeters, this);
    pubGoal = h.advertise<sensor_msgs::NavSatFix>("/radl/goal", 2);
    subGoalMeters = h.subscribe("/landshark/goal_meters", 2, &GatewayGps::getGoalMeters, this);
    pubMap = h.advertise<nav_msgs::Path>("/radl/map", 2);
    subMapMeters = h.subscribe("/landshark/map_meters", 2, &GatewayGps::getMapMeters, this);
  }

  void getGpsMeters(const geometry_msgs::PointStamped::ConstPtr& pMsg) {
    this->msgGpsMeters = *pMsg;
  }

  void getGoalMeters(const geometry_msgs::PointStamped::ConstPtr& pMsg) {
    this->msgGoalMeters = *pMsg;
  }

  void getMapMeters(const nav_msgs::Path::ConstPtr& pMsg) {
    this->msgMapMeters = *pMsg;
  }

  void step(const radl_in_t* in, const radl_in_flags_t* i_f,
            radl_out_t* out, radl_out_flags_t* o_f) 
  {
//send origin reset 
//    msgReset.data = in->pp_cmu_gps_reset->data;
//    pubReset.publish(msgReset);
//send gps to ROS    
    msgGps.latitude = in->navsatfix->latitude;
    msgGps.longitude = in->navsatfix->longitude;
    msgGps.altitude = in->navsatfix->altitude;
    this->pubGps.publish(msgGps);
//send gps_meters to RADL
    out->navsatfix_meters->x = this->msgGpsMeters.point.x;
    out->navsatfix_meters->y = this->msgGpsMeters.point.y;
    out->navsatfix_meters->z = this->msgGpsMeters.point.z;
//send map to ROS
    this->msgMap.poses.resize( map_size_ );
    this->msgMapMeters.poses.resize( map_size_ );

    for (int i1=0; i1< map_size_; i1++)
    {
      this->msgMap.poses[i1].pose.position.x = in->map->points[i1].x;
      this->msgMap.poses[i1].pose.position.y = in->map->points[i1].y;
      this->msgMap.poses[i1].pose.position.z = in->map->points[i1].z;
//send map_meters to RADL
      out->map_meters->points[i1].x = this->msgMapMeters.poses[i1].pose.position.x;
      out->map_meters->points[i1].y = this->msgMapMeters.poses[i1].pose.position.y;
      out->map_meters->points[i1].z = this->msgMapMeters.poses[i1].pose.position.z;
    }
    pubMap.publish(msgMap);
//send goal to ROS
    ros::Time time_now = ros::Time::now();
    msgGoal.header.stamp = time_now;
    msgGoal.latitude = in->goal->latitude;
    msgGoal.longitude = in->goal->longitude;
    msgGoal.altitude = in->goal->altitude;
    pubGoal.publish(msgGoal);
//send goal_meters to RADL
    out->goal_meters->x = this->msgGoalMeters.point.x;
    out->goal_meters->y = this->msgGoalMeters.point.y;
    out->goal_meters->z = this->msgGoalMeters.point.z;

  }

private:
  const size_t map_size_;
  ros::NodeHandle h;
  ros::Publisher pubGps;
  sensor_msgs::NavSatFix msgGps;
  ros::Subscriber subGpsMeters;
  geometry_msgs::PointStamped msgGpsMeters;
  ros::Publisher pubGoal;
  sensor_msgs::NavSatFix msgGoal;
  ros::Publisher pubMap;
  nav_msgs::Path msgMap;
  ros::Subscriber subMapMeters;
  nav_msgs::Path msgMapMeters;
  ros::Publisher pubReset;
  std_msgs::UInt8 msgReset;
  ros::Subscriber subGoalMeters;
  geometry_msgs::PointStamped msgGoalMeters;
};

#endif // GATEWAY_GPS_H
