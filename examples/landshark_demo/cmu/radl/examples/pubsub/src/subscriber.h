#include "radl__subscriber.h"
#include <stdio.h>

class Subscriber {
 private:
  float temp;
  float leak_rate;
  float interval;
 public:
  Subscriber();
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};



