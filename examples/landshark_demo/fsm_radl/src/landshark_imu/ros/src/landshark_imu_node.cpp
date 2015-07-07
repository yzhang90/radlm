/*
 * Copyright (C) 2012, SRI International (R)
 *
 * The material contained in this release is copyrighted. It may not be copied,
 * reproduced, translated, reverse engineered, modified or reduced to any electronic
 * medium or machine-readable form without the prior written consent of
 * SRI International (R).
 *
 * Portions of files in this release may be unpublished work
 * containing SRI International (R) CONFIDENTIAL AND PROPRIETARY INFORMATION.
 * Disclosure, use, reverse engineering, modification, or reproduction without
 * written authorization of SRI International (R) is likewise prohibited.
 *
 * Author(s): Thomas de Candia (thomasd@ai.sri.com)
 *            Aravind Sundaresan (aravind@ai.sri.com)
 *
 */


#include <ros/ros.h>
#include <cstdlib>
#include <cstdio>
#include <LandsharkImu.h>

class LandsharkImuRadl {
  public:
    LandsharkImuRadl( ros::NodeHandle& nh ) 
      : nh_( nh )
    {
      // Get parameters from server (for testing)
      ros::NodeHandle nh2( "~" );
      nh2.param<std::string>( "imu_topic", imu_topic_, "/imu/imu" );
      nh2.param<std::string>( "magnetometer_topic", magnetometer_topic_, "/imu/magnetometer" );

      // Subscribe / Publish
      std::cout << "Publishing to " << imu_topic_ << std::endl;
      imu_pub_ = nh_.advertise<landshark_msgs::Imu>( imu_topic_, 1 );
      std::cout << "Publishing to " << magnetometer_topic_ << std::endl;
      magnetometer_pub_ = nh_.advertise<landshark_msgs::Vector3Stamped>( magnetometer_topic_, 1 );
    }

    void publish( )
    {
      imu_pub_.publish( *out_.imu );
      magnetometer_pub_.publish( *out_.magnetometer );
    }

  public:
    radl_in_t in_; 
    radl_in_flags_t in_flags_;
    radl_out_flags_t out_flags_;
    radl_out_t out_;

  private:
    ros::NodeHandle nh_; 

    std::string imu_topic_;
    std::string magnetometer_topic_;

    ros::Publisher imu_pub_;
    ros::Publisher magnetometer_pub_;
};

int main(int argc, char **argv)
{
  ros::init( argc, argv, "landshark_imu_node" );
  ros::NodeHandle nh;

  LandsharkImuRadl radl( nh );

  double rate_hz;
  ros::NodeHandle nh2( "~" );
  nh2.param<double>( "rate", rate_hz, 10 );
  std::cout << "imu.rate: " << rate_hz << std::endl;
  ros::Rate rate( rate_hz );

  try {
    LandsharkImu imu;
    while (nh.ok()) {
      imu.step( &radl.in_, &radl.in_flags_, &radl.out_, &radl.out_flags_ );
      radl.publish( );
      ros::spinOnce();
      rate.sleep();
    }
  }
  catch ( std::runtime_error e ) {
    std::cerr << "!!! Error: " << e.what() << " imu driver has quit!" << std::endl;
#define STAY_ALIVE
#ifdef STAY_ALIVE
    std::cerr << "Sleeping" << std::endl;
    while (nh.ok() ) {
      ros::spinOnce();
      rate.sleep();
    }
#endif // STAY_ALIVE
  }


  return 0;
}


