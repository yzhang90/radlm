#ifndef LANDSHARK_JOYSTICK_WRAPPER_H
#define LANDSHARK_JOYSTICK_WRAPPER_H

#include "LandsharkJoystick.h"


class LandsharkJoystickWrapper{
  private:
    LandsharkJoystick _node;
  public:
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);

};

#endif
