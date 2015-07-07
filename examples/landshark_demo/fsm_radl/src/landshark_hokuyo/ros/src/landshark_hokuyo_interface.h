#ifndef __LANDSHARK_HOKUYO_INTERFACE_H__
#define __LANDSHARK_HOKUYO_INTERFACE_H__

#include <sensor_msgs/LaserScan.h>

#include "radl__flags.h"

struct radl_in_t {
  radl_in_t()
  {
  }
};

struct radl_in_flags_t {
  radl_in_flags_t() 
    {
    }
};

struct radl_out_flags_t {
  radl_out_flags_t() 
    : scan( 0 )
    {
    }
  radl::flags_t scan;
};

struct radl_out_t {
  radl_out_t() 
    : imu( new sensor_msgs::LaserScan() )
  {
  }
  sensor_msgs::LaserScan *imu;
};

#endif // __LANDSHARK_HOKUYO_INTERFACE_H__

