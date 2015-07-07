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

#include "monitor_ftest.h"
#include "ftest.h"

#include <stdio.h>
#include <math.h>

MonitorFtest::MonitorFtest() {
  this->NUM_STATES_FTEST = 16;
  this->alpha=1.97221562;
  for (this->i = 0; this->i != this->NUM_STATES_FTEST+1*2; ++this->i)
    this->states_ftest[this->i] = 0.0;
}

void MonitorFtest::step(const radl_in_t * in, const radl_in_flags_t* iflags,
                 radl_out_t * out, radl_out_flags_t * oflags) {

  float err;
  int i;
  int ftest_out;

  err = in->imu->angular_velocity.z;

//  ftest_out = ftest(&err, this->states_ftest, this->alpha, i % 16);  //F-test at 90% left-tail with df (15, 15)
//  ftest(ftest_out, &err, this->states_ftest, this->alpha, i % 16); //F-test at 90% left-tail with df (15, 15)

  ftest(&ftest_out, &err, this->alpha, i % 16);


  out->monitor_ftest->data=0;
  if (ftest_out)
  {
//    printf("\n ftest fail");
    out->monitor_ftest->data=1;
  }
  else
  {
//    printf("\n ftest pass");
  }
 
return;
}
