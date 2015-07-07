#ifndef __LANDSHARK_IMU_H__
#define __LANDSHARK_IMU_H__

#include <assert.h>
#include <math.h>
#include <iostream>

#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "3dmgx2.h"

#include RADL_HEADER

struct ImuReading 
{
  bool fresh;
  uint64_t time;
  int64_t ctime;
  double accel[3];
  double angrate[3];
  double mag[3];
  double orientation[9];
};

class RADL__NODE_NAME
{
  public:
    RADL__NODE_NAME();
    ~RADL__NODE_NAME();

    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );

  private:
    int open();
    int close();
    void communication_process();
    void getData();
    std::string getID();

    inline bool GetRunning() {
      boost::mutex::scoped_lock lock( running_mutex_ );
      return m_IsCommunnicationRunning;
    }

    inline void SetRunning( bool value ) {
      boost::mutex::scoped_lock lock( running_mutex_ );
      m_IsCommunnicationRunning = value;
    }


  private:
    microstrain_3dmgx2_imu::IMU imu;
    microstrain_3dmgx2_imu::IMU::cmd cmd_;

    std::string device_;
    ImuReading value_;

    bool open_;
    bool m_IsCommunnicationRunning;
    boost::mutex running_mutex_;
    boost::mutex data_mutex_;
    boost::thread thread_;

    bool calibrated_;
    double offset_;
    double bias_x_;
    double bias_y_;
    double bias_z_;

    double angular_velocity_stdev_, angular_velocity_covariance_;
    double linear_acceleration_covariance_, linear_acceleration_stdev_;
    double orientation_covariance_, orientation_stdev_;

    double max_drift_rate_;
    double m_DesiredProcessPeriod_us;
};

#endif // __LANDSHARK_IMU_H__
