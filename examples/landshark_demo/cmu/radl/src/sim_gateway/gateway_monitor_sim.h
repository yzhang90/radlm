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

#ifndef GATEWAY_MONITOR_SIM_H
#define GATEWAY_MONITOR_SIM_H

#include RADL_HEADER

#include "ros/ros.h"
#include <std_msgs/UInt8.h>

class GatewayMonitorSim {
public:
  GatewayMonitorSim() {
    pubMonitorDWStatus = h.advertise<std_msgs::UInt8>("/radl/monitor_dw/status", 2);
    pubMonitorFtestStatus = h.advertise<std_msgs::UInt8>("/radl/monitor_ftest/status", 2);
  }

  void step(const radl_in_t* in, const radl_in_flags_t* i_f,
            radl_out_t* out, radl_out_flags_t* o_f) 
  {
    this->msgMonitorDWStatus.data = in->estop->data;
    this->pubMonitorDWStatus.publish(this->msgMonitorDWStatus);
    this->msgMonitorFtestStatus.data = in->monitor_ftest->data;
    this->pubMonitorFtestStatus.publish(this->msgMonitorFtestStatus);
  }

private:
  ros::NodeHandle h;

  ros::Publisher pubMonitorDWStatus;
  ros::Publisher pubMonitorFtestStatus;

  std_msgs::UInt8 msgMonitorDWStatus;
  std_msgs::UInt8 msgMonitorFtestStatus;

};

#endif //GATEWAY_GPS_SIM_H
