#include "pubLandsharkNav.h"
#include <stdio.h>
#include <math.h>

PubLandsharkNav::PubLandsharkNav() {
  this->cnt = 1;
  this->latitude_origin = 37.4571014863;
  this->longitude_origin = -122.173496991;
  this->pi=3.14159265359;
  this->freq=100; 
  this->dt=1.0/freq;
  this->NUM_REPEAT=100;
}

void PubLandsharkNav::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                  radl_out_t * out, radl_out_flags_t* outflags) {
  this->cnt++;
  printf("cnt = %d\n", this->cnt);
  out->landshark_nav_waypoint->x = 10.0;
  out->landshark_nav_waypoint->y = 10.0;
  out->landshark_nav_initiate->status=false;
//pub periodically for observation
  if ( cnt++ > this->NUM_REPEAT )
  {
    printf("sending landshark_2dnav initiate\n");
    out->landshark_nav_initiate->status=true;
  }

}
