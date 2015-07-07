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

#ifndef GATEWAY_ODOM_H
#define GATEWAY_ODOM_H

#pragma once
#include RADL_HEADER

#include "ros/ros.h"
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

class GatewayOdom {
public:
  GatewayOdom() {
    pubOdom = h.advertise<nav_msgs::Odometry>("/radl/odom", 2);
    pubImu = h.advertise<sensor_msgs::Imu>("/radl/imu", 2);
    subGpsMeters = h.subscribe("/landshark/gps_meters", 2, &GatewayOdom::getGpsMeters, this);
  }

  void getGpsMeters(const geometry_msgs::PointStamped::ConstPtr& pMsg) {
    this->msgGpsMeters = *pMsg;
  }

  void step(const radl_in_t* in, const radl_in_flags_t* i_f,
            radl_out_t* out, radl_out_flags_t* o_f) 
  {

    ros::Time time_now = ros::Time::now();
    msgOdomGpsImu.header.stamp = time_now;

    msgOdomGpsImu.header.frame_id = "/odom";
    msgOdomGpsImu.child_frame_id = "/base_footprint";

    msgOdomGpsImu.pose.pose.position.x = in->navsatfix_meters->x; //msgGpsMeters.point.x;
    msgOdomGpsImu.pose.pose.position.y = in->navsatfix_meters->y; //msgGpsMeters.point.y;
    msgOdomGpsImu.pose.pose.position.z = in->navsatfix_meters->z; //msgGpsMeters.point.z;

    msgOdomGpsImu.pose.pose.orientation.x = in->imu->orientation.x;
    msgOdomGpsImu.pose.pose.orientation.y = in->imu->orientation.y;
    msgOdomGpsImu.pose.pose.orientation.z = in->imu->orientation.z;
    msgOdomGpsImu.pose.pose.orientation.w = in->imu->orientation.w;

    for ( size_t i = 0; i < 36; i++ ) {
      msgOdomGpsImu.pose.covariance[i] = 0.0;
    }
//from landshark_sim, possible FIX ME
    msgOdomGpsImu.pose.covariance[0] = 0.005;
    msgOdomGpsImu.pose.covariance[7] = 0.005;
    msgOdomGpsImu.pose.covariance[14] = 99999.0;
    msgOdomGpsImu.pose.covariance[21] = 99999.0;
    msgOdomGpsImu.pose.covariance[28] = 99999.0;
    msgOdomGpsImu.pose.covariance[35] = 0.03;
    
    msgOdomGpsImu.twist.twist.linear.x = in->twist->linear.x;
    msgOdomGpsImu.twist.twist.linear.y = in->twist->linear.y;
    msgOdomGpsImu.twist.twist.linear.z = in->twist->linear.z;

    msgOdomGpsImu.twist.twist.angular.x = in->imu->angular_velocity.x;
    msgOdomGpsImu.twist.twist.angular.y = in->imu->angular_velocity.y;
    msgOdomGpsImu.twist.twist.angular.z = in->imu->angular_velocity.z;
//from landshark_sim, possible FIX ME
    for ( size_t i = 0; i < 36; i++ ) {
      msgOdomGpsImu.twist.covariance[i] = 0.0;
    }
    pubOdom.publish(msgOdomGpsImu);

    msgImu.header.frame_id = "/landshark/imu";
    msgImu.header.stamp = time_now;

    msgImu.orientation.x = in->imu->orientation.x;
    msgImu.orientation.y = in->imu->orientation.y;
    msgImu.orientation.z = in->imu->orientation.z;
    msgImu.orientation.w = in->imu->orientation.w; 

    for ( size_t i = 0; i < 9; i++ ) {
      msgImu.orientation_covariance[i] = 0.0;
    }

    msgImu.angular_velocity.x = in->imu->angular_velocity.x;
    msgImu.angular_velocity.y = in->imu->angular_velocity.y;
    msgImu.angular_velocity.z = in->imu->angular_velocity.z;

    for ( size_t i = 0; i < 9; i++ ) {
      msgImu.angular_velocity_covariance[i] = 0.0;
    }

    msgImu.linear_acceleration.x = in->imu->linear_acceleration.x;
    msgImu.linear_acceleration.y = in->imu->linear_acceleration.y;
    msgImu.linear_acceleration.z = in->imu->linear_acceleration.z;

    for ( size_t i = 0; i < 9; i++ ) {
      msgImu.linear_acceleration_covariance[i] = 0.0;
    }
    pubImu.publish(msgImu);
  }

private:
  ros::NodeHandle h;
  ros::Subscriber subGpsMeters;
  geometry_msgs::PointStamped msgGpsMeters;
  ros::Publisher pubOdom;
  nav_msgs::Odometry msgOdomGpsImu;
  ros::Publisher pubImu;
  sensor_msgs::Imu msgImu;
};

#endif // GATEWAY_ODOM_H
