#ifndef __JOYSTICK_INTERFACE_H__
#define __JOYSTICK_INTERFACE_H__

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

#include "radl__flags.h"

struct radl_in_t {
};

struct radl_in_flags_t {
};

struct radl_out_flags_t {
  radl_out_flags_t() 
    : base( 0 )
    , deadman( 0 )
    , over_ride( 0 )
    , turret_pan( 0 )
    , turret_tilt( 0 )
    , moog_pan( 0 )
    , moog_tilt( 0 )
    , moog_zoom( 0 )
    , trigger( 0 )
  {
  }
  radl::flags_t base;
  radl::flags_t deadman;
  radl::flags_t over_ride;
  radl::flags_t turret_pan;
  radl::flags_t turret_tilt;
  radl::flags_t moog_pan;
  radl::flags_t moog_tilt;
  radl::flags_t moog_zoom;
  radl::flags_t trigger;
};

struct radl_out_t {
  radl_out_t() 
    : base( new geometry_msgs::Twist() )
    , deadman( new std_msgs::Bool() )
    , over_ride( new std_msgs::Bool() )
    , turret_pan( new std_msgs::Float64() )
    , turret_tilt( new std_msgs::Float64() )
    , moog_pan( new std_msgs::Float64() )
    , moog_tilt( new std_msgs::Float64() )
    , moog_zoom( new std_msgs::Float64() )
    , trigger( new std_msgs::UInt8() )
  {
  }

  geometry_msgs::Twist *base;
  std_msgs::Bool *deadman;
  std_msgs::Bool *over_ride;
  std_msgs::Float64 *turret_pan;
  std_msgs::Float64 *turret_tilt;
  std_msgs::Float64 *moog_pan;
  std_msgs::Float64 *moog_tilt;
  std_msgs::Float64 *moog_zoom;
  std_msgs::UInt8 *trigger;
};

#endif // __JOYSTICK_INTERFACE_H__


