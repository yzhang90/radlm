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

#ifndef GATEWAY_PP_SIM_H
#define GATEWAY_PP_SIM_H

//#pragma once
#include RADL_HEADER

#include "ros/ros.h"

#include <iostream>
using namespace std;

#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <landshark_msgs/BoolStamped.h>

class GatewayPPSim {
public:
  GatewayPPSim() {
    subGpsReset = h.subscribe<std_msgs::UInt8>("/landshark_sim/gps_meters/reset", 2, &GatewayPPSim::getGpsReset, this); 
    subPPRequest = h.subscribe("/landshark_sim/pp_request", 2, &GatewayPPSim::getPPRequest, this);
    subDeadman = h.subscribe("/landshark_2dnav/deadman", 2, &GatewayPPSim::getDeadman, this);
    pubPPStatus = h.advertise<std_msgs::UInt8>("/radl/pp_status", 2);
    pubBaseVelocity = h.advertise<geometry_msgs::TwistStamped>("/landshark_control/base_velocity", 2);
    pubDeadman = h.advertise<landshark_msgs::BoolStamped>("/landshark_control/deadman", 2);
  }

  void getPPRequest(const std_msgs::UInt8::ConstPtr& pMsg) {
    this->msgPPRequest = *pMsg;
  }

  void getGpsReset(const std_msgs::UInt8::ConstPtr& pMsg) {
    this->msgGpsReset = *pMsg;
  }

  void getDeadman(const std_msgs::Bool::ConstPtr& pMsg) {
    this->msgDeadman = *pMsg;
  }

  void step(const radl_in_t* in, const radl_in_flags_t* i_f,
            radl_out_t* out, radl_out_flags_t* o_f) 
  {

    out->pp_cmu_gps_reset->data = msgGpsReset.data;

    out->pp_request->data = this->msgPPRequest.data;
    msgPPStatus.data = in->pp_status->data;

    pubPPStatus.publish(this->msgPPStatus);
       
    msgBaseVelocity.twist.linear.x = in->pp_base->linear.x;
    msgBaseVelocity.twist.linear.y = in->pp_base->linear.y;
    msgBaseVelocity.twist.linear.z = in->pp_base->linear.z;
    msgBaseVelocity.twist.angular.x = in->pp_base->angular.x;
    msgBaseVelocity.twist.angular.y = in->pp_base->angular.y;
    msgBaseVelocity.twist.angular.z = in->pp_base->angular.z;

    pubBaseVelocity.publish(msgBaseVelocity);

    msgDeadmanStamped.data = msgDeadman.data;
    pubDeadman.publish(msgDeadmanStamped);

  }

private:
  ros::NodeHandle h;

  ros::Publisher pubPPStatus;
  std_msgs::UInt8 msgPPStatus;
  ros::Subscriber subPPRequest;
  std_msgs::UInt8 msgPPRequest;

  ros::Publisher pubBaseVelocity;
  geometry_msgs::TwistStamped msgBaseVelocity;
  ros::Publisher pubDeadman;
  landshark_msgs::BoolStamped msgDeadmanStamped;

  ros::Subscriber subGpsReset;
  std_msgs::UInt8 msgGpsReset;

  ros::Subscriber subDeadman;
  std_msgs::Bool msgDeadman;

};

#endif //GATEWAY_PP_SIM_H

