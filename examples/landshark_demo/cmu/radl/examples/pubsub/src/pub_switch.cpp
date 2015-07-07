#include "pub_switch.h"
#include <stdio.h>

void PublisherSwitch::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                  radl_out_t * out, radl_out_flags_t* outflags) {
  
  out->pub_switch->switch_on = true;

  printf("out->pub_switch->switch_on = %d\n",out->pub_switch->switch_on);
}


