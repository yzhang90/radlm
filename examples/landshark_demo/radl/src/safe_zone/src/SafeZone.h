#ifndef SAFE_ZONE_H
#define SAFE_ZONE_H

#include RADL_HEADER

class SafeZone {
  private:
    double center_x;
    double center_y;
    double x_dist;
    double y_dist;
    double radius;
    bool in_zone;
    int counter;
    int state;
  public:
    SafeZone();
    void step( const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t* );
};

#endif
