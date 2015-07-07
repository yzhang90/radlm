#include "LandsharkJoystickWrapper.h"
#define ROUND 50

LandsharkJoystickWrapper::LandsharkJoystickWrapper() {
  this->counter = 0;
  this->flag = 0;
}

void LandsharkJoystickWrapper::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                                     radl_out_t * out, radl_out_flags_t* outflags) {
  _node.step(in, inflags, out, outflags);
  
  if(this->counter == 0) {
    this->flag = 0;
  }
  
  this->counter++;
  this->flag = this->flag | outflags->ocu_teleop_report;
  if(this->counter == ROUND) {
    out->ocu_teleop_report->flag = this->flag;
    outflags->ocu_teleop_report = 0;
    this->counter == 0;
  }
}