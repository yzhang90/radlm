#include "publisher.h"
#include <stdio.h>

void Publisher::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                  radl_out_t * out, radl_out_flags_t* outflags) {
  if (in->pub_switch->switch_on) {
    out->pub_state->state = true;
  } else {
    out->pub_state->state = false;
  }

  printf("out->pub_state->state = %d\n",out->pub_state->state);

}


