#include "radl__pubLandsharkNav.h"
#include <stdio.h>

class PubLandsharkNav {
 private:
  int cnt;
  double latitude_origin;
  double longitude_origin;
  double freq;
  double dt;
  double pi;
  int NUM_REPEAT;
 public:
  PubLandsharkNav();
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};
