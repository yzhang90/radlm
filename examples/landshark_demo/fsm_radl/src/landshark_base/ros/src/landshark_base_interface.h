#ifndef __LANDSHARK_BASE_INTERFACE_H__
#define __LANDSHARK_BASE_INTERFACE_H__

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <landshark_msgs/ActuatorInput.h>
#include <landshark_msgs/WheelEncoder.h>

#include "radl__flags.h"

struct radl_in_t {
  radl_in_t()
    : base( new geometry_msgs::Twist() )
    , deadman( new std_msgs::Bool() )
    , estop( new std_msgs::UInt8() )
  {
  }
  geometry_msgs::Twist *base;
  std_msgs::Bool *deadman;
  std_msgs::UInt8 *estop;
};

struct radl_in_flags_t {
  radl_in_flags_t()
    : base( 0 )
    , deadman( 0 )
    , estop( 0 )
  {
  }
  radl::flags_t base;
  radl::flags_t deadman;
  radl::flags_t estop;
};

struct radl_out_flags_t {
  radl_out_flags_t()
    : battery( 0 )
    , status( 0 )
    , encoder( 0 )
    , actuator( 0 )
    {
    }
  radl::flags_t battery;
  radl::flags_t status;
  radl::flags_t encoder;
  radl::flags_t actuator;
};

struct radl_out_t {
  radl_out_t()
    : battery( new std_msgs::Float32() )
    , status( new std_msgs::UInt8() )
    , encoder( new landshark_msgs::WheelEncoder() )
    , actuator( new landshark_msgs::ActuatorInput() )
  {
  }
  std_msgs::Float32 *battery;
  std_msgs::UInt8 *status;
  landshark_msgs::WheelEncoder *encoder;
  landshark_msgs::ActuatorInput *actuator;
};

#endif // __LANDSHARK_BASE_INTERFACE_H__


