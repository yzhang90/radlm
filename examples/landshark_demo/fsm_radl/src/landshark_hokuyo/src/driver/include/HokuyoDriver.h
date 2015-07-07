/*********************************************************************
* Software License Agreement (BSD License)
*
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
*********************************************************************/

/**
 * Modified by Thomas de Candia <thomas.decandia@sri.com>
 *             Aravind Sundaresan <aravind@ai.sri.com>
 */
#pragma once

#include <assert.h>
#include <math.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include "hokuyo.h" // change header location

#include "radl_time.h"

using namespace std;

class HokuyoDriver 
{
  static const uint8_t CLOSED = 0; // Not connected to the hardware.
  static const uint8_t OPENED = 1; // Connected to the hardware, ready to start streaming.
  static const uint8_t RUNNING = 2; // Streaming data.

  boost::shared_ptr<boost::thread> scan_thread_;

  std::string device_status_;
  std::string device_id_;
  std::string last_seen_device_id_;
public:
  std::string port_;
  double min_ang_;
  double max_ang_;
  bool calibrate_time_;
  int cluster_;
  bool skip_;
  bool intensity_;
  std::string frame_id_;

private:
  boost::mutex state_mutex_;
  uint8_t state_;
  bool first_scan_;

  std::string vendor_name_;
  std::string product_name_;
  std::string protocol_version_;
  std::string firmware_version_;
  
  std::string connect_fail_;
  
  hokuyo::Laser laser_;
  hokuyo::LaserConfig laser_config_;

  bool calibrated_;
  int lost_scan_thread_count_;
  int corrupted_scan_count_;

public:
  boost::mutex scan_mutex_;
  hokuyo::LaserScan scan_;
  int64_t stamp_;
  bool fresh_;
  uint32_t period_us_;

  HokuyoDriver()
    : port_( "/dev/hokuyo/H1321015" )
    , min_ang_( -M_PI/2 )
    , max_ang_( M_PI/2 )
    , cluster_( 1 )
    , skip_( 0 )
    , calibrate_time_( true )
    , calibrated_( false )
    , lost_scan_thread_count_( 0 )
    , corrupted_scan_count_( 0 )
    , fresh_( false )
    , period_us_( 1e6 / 40 )
  {
    std::cout << "[hokuyo] Expected period of scan: " << period_us_ / 1e3 << "ms" << std::endl;
  }

  void doOpen()
  {
    std::cout << "[hokuyo] Opening device: " << port_ << std::endl;
    try
    {
      std::string old_device_id = device_id_;
      device_id_ = "unknown";
      device_status_ =  "unknown";
      setFirstScan( true );
      
      laser_.open(port_.c_str());
      
      device_id_ = getID();
      vendor_name_ = laser_.getVendorName();
      firmware_version_ = laser_.getFirmwareVersion();
      product_name_ = laser_.getProductName();
      protocol_version_ = laser_.getProtocolVersion();

      device_status_ = laser_.getStatus();
      if (device_status_ != std::string("Sensor works well."))
      {
        doClose();
        std::cerr << "Laser returned abnormal status message, aborting: " << device_status_ << std::endl;
        return;
      }
      
      if (old_device_id != device_id_)
      {
        std::cout << "[hokuyo] Connected to device with ID: " << device_id_ << std::endl;
        
        if (last_seen_device_id_ != device_id_)
        {
          // Recalibrate when the device changes.
          last_seen_device_id_ = device_id_;
          calibrated_ = false;
        }

        // Do this elaborate retry circuis if we were just plugged in.
        for (int retries = 10;; retries--)
          try {
            laser_.laserOn();
            break;
          }
        catch (hokuyo::Exception &e)
        { 
          if (!retries)
            throw e; // After trying for 10 seconds, give up and throw the exception.
          else if (retries == 10)
            std::cout << "[hokuyo] Could not turn on laser. This may happen just after the device is plugged in. Will retry for 10 seconds." << std::endl;
          usleep(1e9);
        }
      }
      else {
        laser_.laserOn(); // Otherwise, it should just work, so no tolerance.
      }

      if (calibrate_time_ && !calibrated_)
      {
        std::cout << "[hokuyo] Starting calibration. This will take up a few seconds." << std::endl;
        double latency = laser_.calcLatency(false, min_ang_, max_ang_, cluster_, skip_) * 1e-9;
        calibrated_ = true; // This is a slow step that we only want to do once.
        std::cout << "[hokuyo] Calibration finished. Latency is: " << latency << "sec" <<std::endl;
      }
      else
      {
        calibrated_ = false;
        laser_.clearLatency();
      }

      std::cout << "[hokuyo] Device opened successfully (" << port_ << ")" << std::endl;
      laser_.getConfig(laser_config_);
      
      setState( OPENED );
    } 
    catch (hokuyo::Exception& e) 
    {
      std::cerr << "Exception thrown while opening Hokuyo: " << e.what() << std::endl;
      doClose();
      return;
    }
  }

  void doClose()
  {
    try
    {
      laser_.close();
      std::cout << "Device closed successfully." << std::endl;
    } catch (hokuyo::Exception& e) {
      std::cout << "Exception thrown while trying to close: " << e.what() << std::endl;
    }
    
    setState( CLOSED ); // If we can't close, we are done for anyways.
  }

  void doStart()
  {
    try
    {
      laser_.laserOn();
      
      std::cout << "[hokuyo] Requesting scans in ["  << min_ang_ << ", " << max_ang_ << "]" << std::endl;
      int status = laser_.requestScans(intensity_, min_ang_, max_ang_, cluster_, skip_);

      if (status != 0) {
        std::cout << "Failed to request scans from device.  Status: " << status << std::endl;
        corrupted_scan_count_++;
        return;
      }
    
      std::cout << "[hokuyo] Waiting for first scan." << std::endl;
      setState( RUNNING );
      scan_thread_.reset(new boost::thread(boost::bind(&HokuyoDriver::scanThread, this)));
    } 
    catch (hokuyo::Exception& e) 
    {
      doClose();
      std::cout << "Exception thrown while starting Hokuyo: " << e.what() << std::endl;
      connect_fail_ = e.what();
      return;
    }
  }

  void doStop()
  {
    if (state_ != RUNNING) // RUNNING can exit asynchronously.
      return;

    setState( OPENED );

    if (scan_thread_ && !scan_thread_->timed_join((boost::posix_time::milliseconds) 2000))
    {
      std::cerr << "scan_thread_ did not die after two seconds. Pretending that it did. This is probably a bad sign." << std::endl;
      lost_scan_thread_count_++;
    }
    scan_thread_.reset();
    
    std::cout << "Stopped." << std::endl;
  }

  virtual std::string getID()
  {
    std::string id = laser_.getID();
    if (id == std::string("H0000000"))
      return "unknown";
    return id;
  }

  void scanThread()
  {
    int timeout_ms_( period_us_ / 1e3 );
    int sleep_us( 0.8 * period_us_ );
    while ( getState() == RUNNING) {
      try {
        boost::mutex::scoped_lock lock( scan_mutex_ );
        int status = laser_.serviceScan( scan_, timeout_ms_ );
        stamp_ = radl_gettime();
        if (status == 0) {
          fresh_ = true;
        }
        else {
          std::cerr << "[hokuyo] Error getting scan: " << status << std::endl;
          break;
        }
      } 
      catch (hokuyo::TimeoutException &e) {
        std::cerr << "Timeout: " << e.what() << std::endl;
        continue;
      } 
      catch (hokuyo::CorruptedDataException &e) {
        std::cerr << "Skipping corrupted data" << std::endl;
        continue;
      } 
      catch (hokuyo::Exception& e) {
        std::cerr << "Exception thrown while trying to get scan.\n" << e.what() << std::endl;
        doClose();
        return;
      }
      if ( getFirstScan() ) {
        setFirstScan( false );
        std::cout << "[hokuyo] Received first scan (streaming data)." << std::endl;
      }

      usleep( sleep_us );
    }

    try
    {
      laser_.stopScanning(); // This actually just calls laser Off internally.
    } catch (hokuyo::Exception &e)
    {
      std::cerr << "Exception thrown while trying to stop scan.\n" << e.what() << std::endl;
    }
    setState( OPENED );
  }

  void setState( uint8_t value ) {
    boost::mutex::scoped_lock lock( state_mutex_ );
    state_ = value;
  }

  uint8_t getState( ) {
    boost::mutex::scoped_lock lock( state_mutex_ );
    return state_;
  }

  void setFirstScan( bool value ) {
    boost::mutex::scoped_lock lock( state_mutex_ );
    first_scan_ = value;
  }

  bool getFirstScan( ) {
    boost::mutex::scoped_lock lock( state_mutex_ );
    return first_scan_;
  }


};
