#ifndef __LANDSHARK_MOOG_PTZ_INTERFACE_H__
#define __LANDSHARK_MOOG_PTZ_INTERFACE_H__

#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>

#include "radl__flags.h"

struct radl_in_t {
  radl_in_t()
    : pan( new std_msgs::Float64() )
    , tilt( new std_msgs::Float64() )
    , zoom( new std_msgs::Float64() )
  {
  }
  std_msgs::Float64 *pan;
  std_msgs::Float64 *tilt;
  std_msgs::Float64 *zoom;
};

struct radl_in_flags_t {
  radl_in_flags_t() 
    : pan( 0 )
    , tilt( 0 )
    , zoom( 0 )
  {
  }
  radl::flags_t pan;
  radl::flags_t tilt;
  radl::flags_t zoom;
};

struct radl_out_flags_t {
  radl_out_flags_t() 
    : status_pan( 0 )
    , status_tilt( 0 )
    , status_zoom( 0 )
    {
    }
  radl::flags_t status_pan;
  radl::flags_t status_tilt;
  radl::flags_t status_zoom;
};

struct radl_out_t {
  radl_out_t() 
    : status_pan( new std_msgs::Float64() )
    , status_tilt( new std_msgs::Float64() )
    , status_zoom( new std_msgs::Float64() )
  {
  }
  std_msgs::Float64 *status_pan;
  std_msgs::Float64 *status_tilt;
  std_msgs::Float64 *status_zoom;
};

#endif // __LANDSHARK_MOOG_PTZ_INTERFACE_H__

