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

#ifndef GATEWAY_GPS_SIM_H
#define GATEWAY_GPS_SIM_H

#include RADL_HEADER

#include "ros/ros.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

class GatewayGpsSim {
public:
  GatewayGpsSim() {
    subGps = h.subscribe("/landshark/gps", 2, &GatewayGpsSim::getGps, this);
    subOdom = h.subscribe("/landshark/odom", 2, &GatewayGpsSim::getOdom, this);
  }

  void getGps(const sensor_msgs::NavSatFix::ConstPtr& pMsg) {
    this->msgGps = *pMsg;
  }
  
  void getOdom(const nav_msgs::Odometry::ConstPtr& pMsg) {
    this->msgOdom = *pMsg;
  }

  void step(const radl_in_t* i, const radl_in_flags_t* i_f,
            radl_out_t* o, radl_out_flags_t* o_f) {
//-------------------------------------------------------------------
//landshark_gps->landshark_gps_meters
//-------------------------------------------------------------------

    o->navsatfix->stamp = radl_gettime();
    o->navsatfix->latitude = msgGps.latitude;
    o->navsatfix->longitude = msgGps.longitude;
    o->navsatfix->altitude = msgGps.altitude;
    for ( size_t i = 0; i < 9; i++ ) {
          o->navsatfix->position_covariance[i] = 0.0;
        }
//   o->landshark_gps->stamp = msgGps.stamp;

    o->navsatfix->position_covariance[0] = 1.0;
    o->navsatfix->position_covariance[4] = 1.0;
    o->navsatfix->position_covariance[8] = 1.0;

//    printf("o->navsatfix->latitude = %15.10f\n",o->navsatfix->latitude);
//    printf("o->navsatfix->longitude = %15.10f\n",o->navsatfix->longitude);
//    printf("o->navsatfix->altitude = %15.10f\n",o->navsatfix->altitude);

    o->twist->stamp = radl_gettime();

    o->twist->linear.x = msgOdom.twist.twist.linear.x;
    o->twist->linear.y = msgOdom.twist.twist.linear.y;
    o->twist->linear.z = msgOdom.twist.twist.linear.z;
    o->twist->angular.x = 0.0;
    o->twist->angular.y = 0.0;
    o->twist->angular.z = 0.0;

//    printf("o->twist->linear.x = %15.10f\n",o->twist->linear.x);
//    printf("o->twist->linear.y = %15.10f\n",o->twist->linear.y);
//    printf("o->twist->linear.z = %15.10f\n",o->twist->linear.z);

    o->timeref->stamp = o->navsatfix->stamp;
    o->timeref->time_ref = o->navsatfix->stamp;
 
  }

private:
  ros::NodeHandle h;

  ros::Subscriber subGps;
  ros::Subscriber subOdom;

  sensor_msgs::NavSatFix msgGps;
  nav_msgs::Odometry msgOdom;

};

#endif //GATEWAY_GPS_SIM_H
