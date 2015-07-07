#ifndef __LANDSHARK_GPS_INTERFACE_H__
#define __LANDSHARK_GPS_INTERFACE_H__

#include <landshark_msgs/Twist.h>
#include <landshark_msgs/NavSatFix.h>
#include <landshark_msgs/TimeReference.h>

#include "radl__flags.h"

struct radl_in_t {
  radl_in_t()
  {
  }
};

struct radl_in_flags_t {
};

struct radl_out_flags_t {
  radl_out_flags_t() 
    : navsatfix( 0 )
    , twist( 0 )
    , timeref( 0 )
    {
    }
  radl::flags_t navsatfix;
  radl::flags_t timeref;
  radl::flags_t twist;
};

struct radl_out_t {
  radl_out_t() 
    : navsatfix( new landshark_msgs::NavSatFix() )
    , timeref( new landshark_msgs::TimeReference() )
    , twist( new landshark_msgs::Twist() )
  {
  }
  landshark_msgs::NavSatFix *navsatfix;
  landshark_msgs::TimeReference *timeref;
  landshark_msgs::Twist *twist;
};

#endif // __LANDSHARK_GPS_INTERFACE_H__

