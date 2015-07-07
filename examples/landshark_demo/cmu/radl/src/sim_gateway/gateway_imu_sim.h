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

#ifndef GATEWAY_IMU_SIM_H
#define GATEWAY_IMU_SIM_H

#include RADL_HEADER

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>

class GatewayImuSim {
public:
  GatewayImuSim() {
    subImu = h.subscribe("/landshark/imu", 2, &GatewayImuSim::getImu, this);
    subMag = h.subscribe("/landshark/magnetometer", 2, &GatewayImuSim::getMag, this);
  }

  void getImu(const sensor_msgs::Imu::ConstPtr& pMsg) {
    this->msgImu = *pMsg;
  }

  void getMag(const geometry_msgs::Vector3Stamped::ConstPtr& pMsg) {
    this->msgMag = *pMsg;
  }

  void step(const radl_in_t* i, const radl_in_flags_t* i_f,
            radl_out_t* o, radl_out_flags_t* o_f) {
//-------------------------------------------------------------------
//landshark_imu->radl_imu
//-------------------------------------------------------------------

    o->imu->stamp = radl_gettime();

    o->imu->orientation.x = msgImu.orientation.x;
    o->imu->orientation.y = msgImu.orientation.y;
    o->imu->orientation.z = msgImu.orientation.z;
    o->imu->orientation.w = msgImu.orientation.w; 

    for ( size_t i = 0; i < 9; i++ ) {
      o->imu->orientation_covariance[i] = 0.0;
    }

    o->imu->angular_velocity.x = msgImu.angular_velocity.x;
    o->imu->angular_velocity.y = msgImu.angular_velocity.y;
    o->imu->angular_velocity.z = msgImu.angular_velocity.z;

    for ( size_t i = 0; i < 9; i++ ) {
      o->imu->angular_velocity_covariance[i] = 0.0;
    }

    o->imu->linear_acceleration.x = msgImu.linear_acceleration.x;
    o->imu->linear_acceleration.y = msgImu.linear_acceleration.y;
    o->imu->linear_acceleration.z = msgImu.linear_acceleration.z;

    for ( size_t i = 0; i < 9; i++ ) {
      o->imu->linear_acceleration_covariance[i] = 0.0;
    }

    o->magnetometer->stamp = radl_gettime();

    o->magnetometer->vector.x = msgMag.vector.x;
    o->magnetometer->vector.y = msgMag.vector.y;
    o->magnetometer->vector.z = msgMag.vector.z;
 
  }

private:
  ros::NodeHandle h;
  ros::Subscriber subImu;
  ros::Subscriber subMag;
  sensor_msgs::Imu msgImu;
  geometry_msgs::Vector3Stamped msgMag;
};

#endif //GATEWAY_IMU_SIM_H
