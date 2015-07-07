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

#include "monitor_stats.h"
#include <stdio.h>
#include "ztest.h"
#include "ftest.h"
//#include "sensor_fusion.h" //CFLAGS: ‘for’ loop initial declarations are only allowed in C99 mode
#include <stdlib.h>


MonitorStats::MonitorStats() {
  this->cnt = 1;
  this->acctDev=0.1;
  this->NUM_STATES_FTEST=16;

  for (this->i = 0; this->i != this->NUM_STATES_FTEST+1*2; ++this->i)
    this->states_ftest_1[this->i] = 0.0;
  for (this->i = 0; this->i != this->NUM_STATES_FTEST+1*2; ++this->i)
    this->states_ftest_2[this->i] = 0.0;

  for (this->i = 0; this->i != this->NUM_STATES_ZTEST+4*2; ++this->i)
    this->states_ztest_1[this->i] = 0.0;
  for (this->i = 0; this->i != this->NUM_STATES_ZTEST+4*2; ++this->i)
    this->states_ztest_2[this->i] = 0.0;
}

void MonitorStats::step(const radl_in_t * in, const radl_in_flags_t* iflags,
                 radl_out_t * out, radl_out_flags_t * oflags) {

  this-cnt++;

//-------------------------------------------------------------------
//ftest
//-------------------------------------------------------------------

  double err;

  int i;

  err = ((double)random()/(RAND_MAX/2.0))-1;
  int ftest_1 = ftest(&err, this->states_ftest_1, 1.97221562, i % 16);  //F-test at 90% left-tail with df (15, 15)

  err = 1.0;
  int ftest_2 = ftest(&err, this->states_ftest_2, 1.97221562, i % 16);  //F-test at 90% left-tail with df (15, 15)
  printf("cnt++ = %d ftest_1 = %d ftest_2 = %d\n", this->cnt++, ftest_1, ftest_2);

//-------------------------------------------------------------------
//ztest
//-------------------------------------------------------------------

//fail test
    err = 1.0*((double)random()/(RAND_MAX/2.0))-1;
    int ztest_1 = ztest(&err, this->states_ztest_1, this->cnt++ % 256, 0.1);
//pass test
    err = 0.0;  //0.1*((double)random()/(RAND_MAX/2.0))-1;
    int ztest_2 = ztest(&err, this->states_ztest_2, this->cnt++ % 256, 0.1);
    printf("cnt++ = %d ztest_1 = %d ztest_2 = %d\n", this->cnt++, ztest_1, ztest_2);
  

//-------------------------------------------------------------------
//sensor_fusion
//-------------------------------------------------------------------

//  double sensors[(2+1+1)*2];
//  double radius[4] = {4.0, 3.0, 2.0, 1.0};
////define vertices and norms
//  double vtx[8] = {-1.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 1.0};
//  double nrm[8] = {-0.7071, -0.7071, 0.7071, -0.7071, 0.7071, 0.7071, -0.7071, 0.7071};
////make sensor values 0, state values very large
//  double state[8];
//  for (int i = 0; i != 8; ++i){
//    sensors[i] = 0.0;
//    state[i] = 10000.0;
//  }
////test Result *: 1 0, pass precision and sensor_fusion  
//  for (int i =0; i != 10; ++i){
//    int out[2];
//    fused(out, sensors, vtx, nrm, radius, &state[0]) ;
//    printf("Result %d: %d %d\n", i, out[0], out[1]);
//  }
//  printf("\n\n");
////test Result *: 0 1, fail precision and sensor_fusion
//  for (int i = 0; i != 8; ++i){
//    sensors[i] = 0.0;
//    state[i] = 10000.0;
//  }
//  for (int i=10; i != 20; ++i){
//    int out[2];
//    sensors[2] = 1.0/0;
//    fused(out, sensors, vtx, nrm, radius, state) ;
//    printf("Result %d: %d %d\n", i, out[0], out[1]);
//  }
//  printf("\n\n");
////test Result *: 1 1, fail precision and sensor_fusion after 
////several iterations
//  for (int i = 0; i != 8; ++i){
//    sensors[i] = 0.0;
//    state[i] = 10000.0;
//  }
//  for (int i=20; i != 30; ++i){
//    int out[2];
//    sensors[2] += 1;
//    fused(out, sensors, vtx, nrm, radius, state) ;
//    printf("Result %d: %d %d\n", i, out[0], out[1]);
//  }
 
return;
}

