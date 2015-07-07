#include "radl__pubLandsharkGpsImu.h"
#include <stdio.h>

class PubLandsharkGpsImu {
 private:
  int cnt;
  double latitude_origin;
  double longitude_origin;
  double freq;
  double dt;
  double gps_radius;
  double pi;
 public:
  PubLandsharkGpsImu();
  void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);
};
