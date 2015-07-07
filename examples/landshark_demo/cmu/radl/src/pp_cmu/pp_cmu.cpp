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

#include "pp_cmu.h"

#include <fstream>

PP::PP() {}

int PP::step(const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags) {
  radl_turn_off(radl_STALE, &out_flags->pp_base);
  radl_turn_off(radl_TIMEOUT, &out_flags->pp_base);

  radl_turn_off(radl_STALE, &out_flags->pp_status);
  radl_turn_off(radl_TIMEOUT, &out_flags->pp_status);

  out->pp_base->linear.x = in->pp_cmu_base->linear.x;
  out->pp_base->linear.y = in->pp_cmu_base->linear.y;
  out->pp_base->linear.z = in->pp_cmu_base->linear.z;
  out->pp_base->angular.x = in->pp_cmu_base->angular.x;
  out->pp_base->angular.y = in->pp_cmu_base->angular.y;
  out->pp_base->angular.z = in->pp_cmu_base->angular.z;

//OFF, ON, ENGAGED, FAILURE, TRYING, GOAL_SUCCESS, GOAL_FAIL 
  out->pp_status->data = in->pp_cmu_status->data; 
//  std::cout << std::endl << "in->pp_cmu_status->data =  " << int(in->pp_cmu_status->data) << std::endl; 

//NONE, ENGAGE, DISENGAGE
  out->pp_cmu_request->data = in->pp_request->data;
//  std::cout << std::endl << "in->pp_request->data =  " << int(in->pp_request->data) << std::endl; 

  return 0;
}
