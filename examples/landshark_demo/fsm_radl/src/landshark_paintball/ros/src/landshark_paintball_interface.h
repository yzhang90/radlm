#ifndef __LANDSHARK_PAINTBALL_INTERFACE_H__
#define __LANDSHARK_PAINTBALL_INTERFACE_H__

#include <std_msgs/UInt8.h>

#include "radl__flags.h"

struct radl_in_t {
  radl_in_t()
    : trigger( new std_msgs::UInt8() )
  {
  }
  std_msgs::UInt8 *trigger;
};

struct radl_in_flags_t {
  radl_in_flags_t() 
    : trigger( 0 )
  {
  }
  radl::flags_t trigger;
};

struct radl_out_flags_t {
  radl_out_flags_t() 
    {
    }
};

struct radl_out_t {
  radl_out_t() 
  {
  }
};

#endif // __LANDSHARK_PAINTBALL_INTERFACE_H__

