#ifndef LANDSHARK_Base_WRAPPER_H
#define LANDSHARK_Base_WRAPPER_H

#include "LandsharkBase.h"


class LandsharkBaseWrapper{
  private:
    LandsharkBase _node;
    int counter;
    int flag;
  public:
    LandsharkBaseWrapper();
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);

};

#endif
