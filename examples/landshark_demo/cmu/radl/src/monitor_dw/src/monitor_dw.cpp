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

#include "monitor_dw.h"
#include <stdio.h>
#include <math.h>

MonitorDW::MonitorDW() 
: map_size_( *RADL_THIS->array_size )
{
  this->cnt = 1;
//dwmonitor input constants
  this->max_accv = 0.5;     //m^2/s
  this->dt = 0.1;           //s
  this->outer_radius = 2.0; //m
  this->accv = max_accv;    //m^2/s
}

void MonitorDW::step(const radl_in_t * in, const radl_in_flags_t* iflags,
                 radl_out_t * out, radl_out_flags_t * oflags) 
{

//input sensor values
  double vel_x = in->twist->linear.x;
  double vel_y = in->twist->linear.y;
  double vel = sqrt(vel_x*vel_x + vel_y*vel_y); //m/s
  float robot_x = in->navsatfix_meters->x;  //m
  float robot_y = in->navsatfix_meters->y;  //m
  float robot_z = in->navsatfix_meters->z;  //m

  float obs_x = in->map_meters->points[0].x;   //m
  float obs_y = in->map_meters->points[0].y;  //m
  float obs_z = in->map_meters->points[0].z;  //m
  double f_v = fabs(vel); //f_V = maxObsVel;

//find closest obstacle for dwmonitor
  double dis_x = robot_x - obs_x;
  double dis_y = robot_y - obs_y;
  double dis_z = robot_z - obs_z;
  double dis = dis_x*dis_x + dis_y*dis_y ;
  double dis_check;
  for (int i=1; i < map_size_; i++)
  {
    dis_x = in->map_meters->points[i].x - robot_x;
    dis_y = in->map_meters->points[i].y - robot_y;
    dis_check = dis_x*dis_x + dis_y*dis_y ;
    if (dis_check < dis)
    {
      dis = dis_check;
      obs_x = in->map_meters->points[i].x;
      obs_y = in->map_meters->points[i].y;
      obs_z = in->map_meters->points[i].z;
    }
  }

//make obs list for dwmonitor2
  float obs[map_size_ * 3];
  for (int i = 0; i != map_size_; ++i){
    obs[i*3+0] = in->map_meters->points[i].x;
    obs[i*3+1] = in->map_meters->points[i].y;
    obs[i*3+2] = 0.0; //FIX ME: z position is obs_r
  }
  
//  printf("/n closest obs_x = %f obs_y = %f/n",obs_x,obs_y);

//-------------------------------------------------------------------
//dwmonitor
//-------------------------------------------------------------------

//  float accv = this->accv;
//  float timePeriod = this->dt;
//  float X[5];
//  
//  X[0] = fabs((float)vel);
//  X[1] = (float)robot_x; X[2] = (float)robot_y;
//  X[3] = (float)obs_x; X[4] = (float)obs_y;
//  double d0 = (double)((accv / accv + 1) *\
//  (accv / 2 * timePeriod * timePeriod + f_v * timePeriod));
//  double D[3];
//  D[0] = d0 + this->outer_radius;
//  D[1] = (double)((f_v / accv) + timePeriod * (accv / accv + 1));
//  D[2] = (double)(1 / (2 * accv));

//  int result = dwmonitor(X, D);

//  if (result==1)
//  {
////    printf("\n okay to go");
//    out->estop->data = 0;
//  }
//  else
//  {
////    printf("\n too close to the obstacle");
//    out->estop->data = 1;
//  }

////-------------------------------------------------------------------
////dwmonitor2
////-------------------------------------------------------------------

  float accv = this->accv;
  float dt = this->dt;

  double d0 = ((accv/accv+1) * (accv/2*dt*dt) + f_v * dt);
  double D[3];
  D[0] = d0 + this->outer_radius;
  D[1] = (f_v/accv) + dt*(accv/accv+1);
  D[2] = 1/(2*accv);

  float X[3]; X[0] = f_v;
  X[1] = (float)robot_x; X[2] = (float)robot_y;

  int result[2]; 
  dwmonitor_all(result, X, D, obs);

  if (result[1]==1)
  {
//    printf("\n okay to go");
    out->estop->data = 0;
  }
  else
  {
//    printf("\n too close to the obstacle");
    out->estop->data = 1;
  }

  if (result[0]==0)
  {
    printf("\n input is out of bounds (FLT_MAX, FLT_MIN, Inf, or NaN)");
  }

////-------------------------------------------------------------------
////dwmonitor2
////-------------------------------------------------------------------
 
return;
}
