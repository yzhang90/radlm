#include "GatewayTeleopWrapper.h"
#define ROUND 50

GatewayTeleopWrapper::GatewayTeleopWrapper() {
  this->counter = 0;
  this->flag = 0;
}

void GatewayTeleopWrapper::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                                     radl_out_t * out, radl_out_flags_t* outflags) {
  _node.step(in, inflags, out, outflags);
  
  if(this->counter == 0) {
    this->flag = 0;
  }
  
  this->counter++;
  this->flag = this->flag | outflags->gateway_teleop_report;
  if(this->counter == ROUND) {
    out->gateway_teleop_report->flag = this->flag;
    outflags->gateway_teleop_report = 0;
    this->counter == 0;
  }
}
