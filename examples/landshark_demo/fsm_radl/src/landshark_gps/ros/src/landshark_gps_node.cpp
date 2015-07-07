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
#include <LandsharkGpsRadl.h>

class LandsharkGpsRos {
  public:
    LandsharkGpsRos( ros::NodeHandle& nh ) 
      : nh_( nh )
    {
      // Get parameters from server (for testing)
      ros::NodeHandle nh2( "~" );
      nh2.param<std::string>( "navsatfix_topic", navsatfix_topic_, "/gps/navsatfix" );
      nh2.param<std::string>( "timeref_topic", timeref_topic_, "/gps/timeref" );
      nh2.param<std::string>( "twist_topic", twist_topic_, "/gps/twist" );

      // Subscribe / Publish
      std::cout << "Publishing to " << navsatfix_topic_ << std::endl;
      navsatfix_pub_ = nh_.advertise<landshark_msgs::NavSatFix>( navsatfix_topic_, 1 );
      std::cout << "Publishing to " << timeref_topic_ << std::endl;
      timeref_pub_ = nh_.advertise<landshark_msgs::TimeReference>( timeref_topic_, 1 );
      std::cout << "Publishing to " << twist_topic_ << std::endl;
      twist_pub_ = nh_.advertise<landshark_msgs::Twist>( twist_topic_, 1 );
    }

    void publish( )
    {
      navsatfix_pub_.publish( *out_.navsatfix );
      timeref_pub_.publish( *out_.timeref );
      twist_pub_.publish( *out_.twist );
    }

  public:
    radl_in_t in_; 
    radl_in_flags_t in_flags_;
    radl_out_flags_t out_flags_;
    radl_out_t out_;

  private:
    ros::NodeHandle nh_; 

    std::string navsatfix_topic_;
    std::string timeref_topic_;
    std::string twist_topic_;

    ros::Publisher navsatfix_pub_;
    ros::Publisher timeref_pub_;
    ros::Publisher twist_pub_;
};

int main(int argc, char **argv)
{
  ros::init( argc, argv, "landshark_gps_node" );
  ros::NodeHandle nh;

  LandsharkGpsRos radl( nh );

  double rate_hz;
  ros::NodeHandle nh2( "~" );
  nh2.param<double>( "rate", rate_hz, 10 );
  std::cout << "base.rate: " << rate_hz << std::endl;
  ros::Rate rate( rate_hz );

  try {
    LandsharkGpsRadl gps;
    while (nh.ok()) {
      gps.step( &radl.in_, &radl.in_flags_, &radl.out_, &radl.out_flags_ );
      radl.publish( );
      ros::spinOnce();
      rate.sleep();
    }
  }
  catch ( std::runtime_error e ) {
    std::cerr << "!!! Error: " << e.what() << " gps driver has quit!" << std::endl;
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


