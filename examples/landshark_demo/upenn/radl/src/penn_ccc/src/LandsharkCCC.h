#ifndef __LANDSHARK_CCC_H__
#define __LANDSHARK_CCC_H__

#include <assert.h>
#include <math.h>
#include <iostream>

#include <ros/ros.h>
#include <cstdlib>
#include <cstdio>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include RADL_HEADER

class LandsharkCCC
{
  public:
    LandsharkCCC();
    ~LandsharkCCC();
    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );

  private:
};

#endif // __LANDSHARK_CCC_H__
