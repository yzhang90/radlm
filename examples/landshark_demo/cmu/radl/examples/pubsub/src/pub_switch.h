#include "radl__pub_switch.h"
#include <stdio.h>

class PublisherSwitch {
 public:
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};
