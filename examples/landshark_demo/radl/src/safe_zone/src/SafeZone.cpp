#include "SafeZone.h"

SafeZone::SafeZone() {
  this->center_x = 0;
  this->center_y = 0;
  this->radius = 0;
  this->state = 0;
}

void SafeZone::step(const radl_in_t *in, const radl_in_flags_t *inflags,
                    radl_out_t *out, radl_out_flags_t *outflags) {
  //monitor variables
  x_dist = in->navsatfix->latitude - center_x;
  y_dist = in->navsatfix->longitude - center_y;
  in_zone = x_dist*x_dist + y_dist*y_dist <= radius*radius ? true : false;
  
  //fsm
  switch(state) {
    case 0: {
      if(in_zone) {
        state = 1;
      }
      break;         
    }
    case 1: {
      if(!in_zone) {
        state = 0;
      }
      break;      
    }
    default: {
      state = 0;
      break;
    }
  }

  out->safezone_state->state = this->state;

}
