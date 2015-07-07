#ifndef __LANDSHARK_StateEstimator_H__
#define __LANDSHARK_StateEstimator_H__

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

class LandsharkStateEstimator
{
  public:
    LandsharkStateEstimator();
    ~LandsharkStateEstimator();
    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );

  private:
    int left_enc_previous, right_enc_previous, enc_time_previous;
    int gps_age, encoder_age, rse_status, rse_wait;
    double v_enc_left_previous, v_enc_right_previous, v_gps_previous;
    double act_left_previous, act_right_previous;
};

#endif // __LANDSHARK_StateEstimator_H__
