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

#ifndef PP_GATEWAY_H
#define PP_GATEWAY_H

#include RADL_HEADER

#include <iostream>
using namespace std;

#include "ros/ros.h"
#include <std_msgs/UInt8.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TwistStamped.h>

class GatewayPP {
public:
  GatewayPP() {
    pubPPRequest = h.advertise<std_msgs::UInt8>("/radl/pp_request", 2);
    subPPStatus = h.subscribe("/landshark_2dnav/pp_status", 2, &GatewayPP::getPPStatus, this);
    subBaseVelocity = h.subscribe("/landshark_2dnav/base_velocity", 2, &GatewayPP::getBaseVelocity, this);
  }

  void getPPStatus(const std_msgs::UInt8::ConstPtr& pMsg) {
    this->msgPPStatus = *pMsg;
  }

  void getBaseVelocity(const geometry_msgs::TwistStamped::ConstPtr& pMsg) {
    this->msgBaseVelocity = *pMsg;
  }

  void step(const radl_in_t* in, const radl_in_flags_t* i_f,
            radl_out_t* out, radl_out_flags_t* o_f) 
  {

//OFF, ON, ENGAGED, FAILURE, TRYING, GOAL_SUCCESS, GOAL_FAIL 
    out->pp_status->data = this->msgPPStatus.data;
//    std::cout << std::endl << "out->pp_status->data =  " << int(out->pp_status->data) << std::endl; 
//NONE, ENGAGE, DISENGAGE
    this->msgPPRequest.data = in->pp_request->data;
//    std::cout << std::endl << "this->msgPPRequest.data =  " << int(this->msgPPRequest.data) << std::endl; 
    pubPPRequest.publish(this->msgPPRequest);
    out->pp_base->linear.x = msgBaseVelocity.twist.linear.x;
    out->pp_base->linear.y = msgBaseVelocity.twist.linear.y;
    out->pp_base->linear.z = msgBaseVelocity.twist.linear.z;
    out->pp_base->angular.x = msgBaseVelocity.twist.angular.x;
    out->pp_base->angular.y = msgBaseVelocity.twist.angular.y;
    out->pp_base->angular.z = msgBaseVelocity.twist.angular.z;
  }

private:
  ros::NodeHandle h;
  ros::Publisher pubPPRequest;
  std_msgs::UInt8 msgPPRequest;
  ros::Subscriber subPPStatus;
  std_msgs::UInt8 msgPPStatus;
  ros::Subscriber subBaseVelocity;
  geometry_msgs::TwistStamped msgBaseVelocity;
};

#endif //PP_GATEWAY_H
