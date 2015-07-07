#include "subscriber.h"
#include <stdio.h>

Subscriber::Subscriber() {
  this->leak_rate = 0.1;
  this->interval = 0.02;
  this->temp = 70.0;
}

void Subscriber::step(const radl_in_t * in, const radl_in_flags_t* iflags,
                 radl_out_t * out, radl_out_flags_t * oflags) {

//  if (in->pub_state->state) {
//    out->sub_state->state = true;
//  } else {
//    out->sub_state->state = false;
//  }

//  printf("out->sub_state->state = %d\n",out->sub_state->state);

  printf("in->pub_state->state = %d\n",in->pub_state->state);

  printf("this->temp = %f\n",this->temp);

}


