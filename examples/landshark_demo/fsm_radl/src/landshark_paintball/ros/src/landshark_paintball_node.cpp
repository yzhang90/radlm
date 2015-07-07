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
#include <LandsharkPaintball.h>

class LandsharkPaintballRadl {
  public:
    LandsharkPaintballRadl( ros::NodeHandle& nh ) 
      : nh_( nh )
    {
      // Get parameters from server (for testing)
      ros::NodeHandle nh2( "~" );
      nh2.param<std::string>( "paintball_trigger_topic", paintball_trigger_topic_, "/paintball/trigger" );

      // Subscribe 
      std::cout << "Subscribing to " << paintball_trigger_topic_ << std::endl;
      paintball_trigger_sub_ = nh_.subscribe<std_msgs::UInt8>( paintball_trigger_topic_, 1, &LandsharkPaintballRadl::trigger_cb, this );
    }

    void publish( )
    {
    }

    void trigger_cb( const std_msgs::UInt8::ConstPtr& msg )
    {
      in_.trigger->data = msg->data;
    }

  public:
    radl_in_t in_; 
    radl_in_flags_t in_flags_;
    radl_out_flags_t out_flags_;
    radl_out_t out_;

  private:
    ros::NodeHandle nh_; 
    std::string paintball_trigger_topic_;
    ros::Subscriber paintball_trigger_sub_;
};

int main(int argc, char **argv)
{
  ros::init( argc, argv, "landshark_paintball_node" );
  ros::NodeHandle nh;

  LandsharkPaintballRadl radl( nh );

  double rate_hz;
  ros::NodeHandle nh2( "~" );
  nh2.param<double>( "rate", rate_hz, 10 );
  std::cout << "paintball.rate: " << rate_hz << std::endl;
  ros::Rate rate( rate_hz );

  try {
    LandsharkPaintball paintball;
    while (nh.ok()) {
      paintball.step( &radl.in_, &radl.in_flags_, &radl.out_, &radl.out_flags_ );
      radl.publish( );
      ros::spinOnce();
      rate.sleep();
    }
  }
  catch ( std::runtime_error e ) {
    std::cerr << "!!! Error: " << e.what() << " paintball driver has quit!" << std::endl;
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


