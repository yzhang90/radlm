#ifndef __LANDSHARK_IMU_INTERFACE_H__
#define __LANDSHARK_IMU_INTERFACE_H__

#include <landshark_msgs/Vector3Stamped.h>
#include <landshark_msgs/Imu.h>
#include <std_msgs/UInt8.h>

#include "radl__flags.h"

struct radl_in_t {
  radl_in_t()
    //: calibrate( new std_msgs::UInt8() )
  {
  }
  //std_msgs::UInt8 *calibrate;
};

struct radl_in_flags_t {
  radl_in_flags_t() 
    //: calibrate( 0 )
    {
    }
  //radl::flags_t calibrate;
};

struct radl_out_flags_t {
  radl_out_flags_t() 
    : imu( 0 )
    , magnetometer( 0 )
    {
    }
  radl::flags_t imu;
  radl::flags_t magnetometer;
};

struct radl_out_t {
  radl_out_t() 
    : imu( new landshark_msgs::Imu() )
    , magnetometer( new landshark_msgs::Vector3Stamped() )
  {
  }
  landshark_msgs::Imu *imu;
  landshark_msgs::Vector3Stamped *magnetometer;
};

#endif // __LANDSHARK_IMU_INTERFACE_H__

