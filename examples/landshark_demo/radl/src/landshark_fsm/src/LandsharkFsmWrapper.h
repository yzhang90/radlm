#ifndef LANDSHARK_FSM_WRAPPER_H
#define LANDSHARK_FSM_WRAPPER_H

#include "fsm.h"


class LandsharkFsmWrapper{
  private:
    FSM _node;
    int counter;
    int flag;
  public:
    LandsharkFsmWrapper();
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);

};

#endif
