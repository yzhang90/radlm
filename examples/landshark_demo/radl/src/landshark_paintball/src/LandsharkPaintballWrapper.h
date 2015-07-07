#ifndef LANDSHARK_PAINTBALL_WRAPPER_H
#define LANDSHARK_PAINTBALL_WRAPPER_H

#include "LandsharkPaintball.h"


class LandsharkPaintballWrapper{
  private:
    LandsharkPaintball _node;
  public:
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);

};

#endif
