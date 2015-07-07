#ifndef GATEWAY_TELEOP_WRAPPER_H
#define GATEWAY_TELEOP_WRAPPER_H

#include "gateway_9.h"


class GatewayTeleopWrapper{
  private:
    identity_9 _node;
    int counter;
    int flag;
  public:
    GatewayTeleopWrapper();
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);

};

#endif
