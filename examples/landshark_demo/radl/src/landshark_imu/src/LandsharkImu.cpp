/*
 * Software License Agreement (BSD License)
 *
 *  Microstrain 3DM-GX2 node
 *  Copyright (c) 2008-2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "LandsharkImu.h"
#include "bullet/LinearMath/btMatrix3x3.h"

#ifdef PUBLISH_SENSOR_MSGS
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Vector3Stamped.h"
#endif // PUBLISH_SENSOR_MSGS

using namespace std;

RADL_NODE_NAME::RADL_NODE_NAME()
  : device_( "/dev/ttyACM1" )
  , max_drift_rate_( 0.000100 )
  , offset_( 0.0 )
  , linear_acceleration_stdev_( 0.098)
  , orientation_stdev_( 0.035)
  , angular_velocity_stdev_( 0.012)
  , open_( false )
  , m_IsCommunnicationRunning( false )
  , m_DesiredProcessPeriod_us( 1e4 )
{
  device_ = *RADL_THIS->device;
  m_DesiredProcessPeriod_us = radl_to_nsec( *RADL_THIS->period ) * 1e-3;

  std::cout << "[imu] device: " << device_ << std::endl;
  cmd_ = microstrain_3dmgx2_imu::IMU::CMD_ACCEL_ANGRATE_MAG_ORIENT;

  bias_x_ = bias_y_ = bias_z_ = 0;

  angular_velocity_covariance_ = angular_velocity_stdev_ * angular_velocity_stdev_;
  orientation_covariance_ = orientation_stdev_ * orientation_stdev_;
  linear_acceleration_covariance_ = linear_acceleration_stdev_ * linear_acceleration_stdev_;

  SetRunning( true );
  // Start thread
  thread_ = boost::thread( &RADL_NODE_NAME::communication_process, this );
}

RADL_NODE_NAME::~RADL_NODE_NAME()
{
  SetRunning( false );
  thread_.join();
  close();
}

int RADL_NODE_NAME::open()
{
  if ( open_ ) {
    close();
    open_ = false;
  }

  try {
    imu.openPort(device_.c_str());
    getID();

    std::cout << "[imu] Not calibrating the IMU sensor. " << std::endl;
    std::cout << "[imu] Initializing IMU time with offset " << offset_ << std::endl;
    imu.initTime(offset_);

    std::cout << "[imu] IMU sensor initialized." << std::endl;

    imu.setContinuous( cmd_ );

    open_ = true;
  }
  catch (microstrain_3dmgx2_imu::Exception& e) {
    std::cerr << "  Exception thrown while starting IMU: " << e.what() << std::endl;
    std::cerr << "  This sometimes happens if you are not connected to an IMU\n"
      << "  or if another process is trying to access the IMU device. \n";
    std::cerr << "  You may try 'lsof|grep " << device_ << "' to see if\n"
      << "  other processes have the device open." << std::endl;
    return -1;
  }

  return 0;
}

std::string RADL_NODE_NAME::getID()
{
  char val[64];
  imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_DEVICE_NAME, val );
  std::string dev_name( val );
  imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_MODEL_NUMBER, val );
  std::string dev_model_num( val );
  imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_SERIAL_NUMBER, val );
  std::string dev_serial_num( val );
  imu.getDeviceIdentifierString(microstrain_3dmgx2_imu::IMU::ID_DEVICE_OPTIONS, val );
  std::string dev_opt( val );

  std::cout << "[imu] IMU:   " << dev_name << std::endl;
  std::cout << "[imu] model: " << dev_model_num << std::endl;
  std::cout << "[imu] SN:    " << dev_serial_num << std::endl;
  std::cout << "[imu] opt:   " << dev_opt << std::endl;

  return (boost::format("%s_%s-%s")%dev_name.c_str()%dev_model_num.c_str()%dev_serial_num.c_str()).str();
}

int RADL_NODE_NAME::close()
{
  if (open_) {
    try {
      imu.closePort();
    }
    catch (microstrain_3dmgx2_imu::Exception& e) {
      throw std::string("Exception thrown while stopping IMU. %s", e.what());
    }
    open_ = false;
  }

  return 0;
}

void RADL_NODE_NAME::communication_process()
{
  std::cout << "[imu] starting thread" << std::endl;
  //boost::posix_time::time_duration processPeriod;
  //boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();
  struct radl_timer_state timer;
  radl_duration_t loop_period( m_DesiredProcessPeriod_us * 1000 );
  radl_timer_init( &timer, loop_period );

  while ( GetRunning() ) {
    std::cout << "[imu] Starting IMU..." << std::endl;
    if ( open() == 0 ) {
      try {
        std::cout << "[imu] Processing IMU data..." << std::endl;
        while( GetRunning() ) {
#if 1
          radl_timer_wait( &timer );
#else
          processPeriod = boost::posix_time::microsec_clock::local_time() - startTime;
          long timeToSleep = m_DesiredProcessPeriod_us - processPeriod.total_microseconds();
          if (timeToSleep > 0) {
            boost::this_thread::sleep(boost::posix_time::microseconds(timeToSleep));
          }
          startTime = boost::posix_time::microsec_clock::local_time();
#endif 
          getData();
        }
      }
      catch ( microstrain_3dmgx2_imu::Exception& e ) {
        std::cerr << "[imu] error: " << e.what() << "... trying again..." << std::endl;
        usleep( 1e5 );
      }
      std::cout << "[imu] Stopping IMU..." << std::endl;
      close();
    }
    else {
      std::cout << "[imu] Failed to open device..." << std::endl;
      std::stringstream ss;
      ss << "Cannot open IMU device: " << device_;
      throw std::string( ss.str() );
    }
  }
  std::cout << "[imu] stopping thread" << std::endl;
}


void RADL_NODE_NAME::getData()
{
  boost::mutex::scoped_lock lock( data_mutex_ );
  imu.receiveAccelAngrateMagOrientation( &value_.time, value_.accel, value_.angrate, value_.mag, value_.orientation);
  value_.ctime = radl_gettime();
  value_.fresh = true;
}

/**
 * The calibrate flag can be done automatically.
 * The flags should be set to indicate the sensor is calibrating and the values are not
 */

int RADL_NODE_NAME::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags )
{
  for ( size_t i = 0; i < 9; i++ ) {
    out->imu->linear_acceleration_covariance[i] = 0;
    out->imu->angular_velocity_covariance[i] = 0;
    out->imu->orientation_covariance[i] = 0;
  }

  out->imu->linear_acceleration_covariance[0] = linear_acceleration_covariance_;
  out->imu->linear_acceleration_covariance[4] = linear_acceleration_covariance_;
  out->imu->linear_acceleration_covariance[8] = linear_acceleration_covariance_;

  out->imu->angular_velocity_covariance[0] = angular_velocity_covariance_;
  out->imu->angular_velocity_covariance[4] = angular_velocity_covariance_;
  out->imu->angular_velocity_covariance[8] = angular_velocity_covariance_;

  out->imu->orientation_covariance[0] = orientation_covariance_;
  out->imu->orientation_covariance[4] = orientation_covariance_;
  out->imu->orientation_covariance[8] = orientation_covariance_;

  boost::mutex::scoped_lock lock( data_mutex_ );

  out->imu->stamp = value_.ctime;
  out->magnetometer->stamp = value_.ctime;

  out->imu->linear_acceleration.x = value_.accel[0];
  out->imu->linear_acceleration.y = value_.accel[1];
  out->imu->linear_acceleration.z = value_.accel[2];

  out->imu->angular_velocity.x = value_.angrate[0];
  out->imu->angular_velocity.y = value_.angrate[1];
  out->imu->angular_velocity.z = value_.angrate[2];

  btQuaternion q;
  (btMatrix3x3(-1,0,0,
                 0,1,0,
                 0,0,-1)*
   btMatrix3x3( value_.orientation[0], value_.orientation[3], value_.orientation[6],
     value_.orientation[1], value_.orientation[4], value_.orientation[7],
     value_.orientation[2], value_.orientation[5], value_.orientation[8])).getRotation(q);

  out->imu->orientation.x = q[0];
  out->imu->orientation.y = q[1];
  out->imu->orientation.z = q[2];
  out->imu->orientation.w = q[3];

  out->magnetometer->vector.x = value_.mag[0];
  out->magnetometer->vector.y = value_.mag[1];
  out->magnetometer->vector.z = value_.mag[2];

  if ( !value_.fresh ) {
    radl_turn_on( radl_STALE, &out_flags->imu );
    radl_turn_on( radl_STALE, &out_flags->magnetometer );
  }
  value_.fresh = false;

#ifdef PUBLISH_SENSOR_MSGS
  {
    static ros::NodeHandle nh( "~" );
    static ros::Publisher p1 = nh.advertise<sensor_msgs::Imu>("imu", 100);
    static ros::Publisher p2 = nh.advertise<geometry_msgs::Vector3Stamped>("magnetometer", 100);
    static sensor_msgs::Imu msg1;
    static geometry_msgs::Vector3Stamped msg2;

    msg1.header.stamp = ros::Time::now();
    msg1.header.frame_id = "imu";
    msg1.orientation.x = out->imu->orientation.x;
    msg1.orientation.y = out->imu->orientation.y;
    msg1.orientation.z = out->imu->orientation.z;
    msg1.orientation.w = out->imu->orientation.w;
    msg1.angular_velocity.x = out->imu->angular_velocity.x;
    msg1.angular_velocity.y = out->imu->angular_velocity.y;
    msg1.angular_velocity.z = out->imu->angular_velocity.z;
    msg1.linear_acceleration.x = out->imu->linear_acceleration.x;
    msg1.linear_acceleration.y = out->imu->linear_acceleration.y;
    msg1.linear_acceleration.z = out->imu->linear_acceleration.z;
    for ( size_t i = 0; i < 9; i++ ) {
      msg1.linear_acceleration_covariance[i] = out->imu->linear_acceleration_covariance[i];
      msg1.angular_velocity_covariance[i] = out->imu->angular_velocity_covariance[i];
      msg1.orientation_covariance[i] = out->imu->orientation_covariance[i];
    }
    p1.publish( msg1 );
    msg2.vector.x = out->magnetometer->vector.x;
    msg2.vector.y = out->magnetometer->vector.z;
    msg2.vector.z = out->magnetometer->vector.z;
    msg2.header = msg1.header;
    p2.publish( msg2 );
  }
#endif // PUBLISH_SENSOR_MSGS

}

