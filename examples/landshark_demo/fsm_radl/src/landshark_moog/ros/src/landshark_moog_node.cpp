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
#include <LandsharkMoog.h>

class LandsharkMoogRadl {
  public:
    LandsharkMoogRadl( ros::NodeHandle& nh ) 
      : nh_( nh )
    {
      // Get parameters from server (for testing)
      ros::NodeHandle nh2( "~" );
      nh2.param<std::string>( "moog_pan_topic", moog_pan_topic_, "/moog/pan" );
      nh2.param<std::string>( "moog_tilt_topic", moog_tilt_topic_, "/moog/tilt" );
      nh2.param<std::string>( "moog_zoom_topic", moog_zoom_topic_, "/moog/zoom" );
      nh2.param<std::string>( "moog_pan_status_topic", moog_pan_status_topic_, "/moog/status/pan" );
      nh2.param<std::string>( "moog_tilt_status_topic", moog_tilt_status_topic_, "/moog/status/tilt" );
      nh2.param<std::string>( "moog_zoom_status_topic", moog_zoom_status_topic_, "/moog/status/zoom" );

      // Subscribe 
      std::cout << "Subscribing to " << moog_pan_topic_ << std::endl;
      moog_pan_sub_ = nh_.subscribe<std_msgs::Float64>( moog_pan_topic_, 1, &LandsharkMoogRadl::pan_cb, this );
      std::cout << "Subscribing to " << moog_tilt_topic_ << std::endl;
      moog_tilt_sub_ = nh_.subscribe<std_msgs::Float64>( moog_tilt_topic_, 1, &LandsharkMoogRadl::tilt_cb, this );
      std::cout << "Subscribing to " << moog_zoom_topic_ << std::endl;
      moog_zoom_sub_ = nh_.subscribe<std_msgs::Float64>( moog_zoom_topic_, 1, &LandsharkMoogRadl::zoom_cb, this );
      // Publish
      std::cout << "Publishing to " << moog_pan_status_topic_ << std::endl;
      moog_pan_status_pub_ = nh_.advertise<std_msgs::Float64>( moog_pan_status_topic_, 1 );
      std::cout << "Publishing to " << moog_tilt_status_topic_ << std::endl;
      moog_tilt_status_pub_ = nh_.advertise<std_msgs::Float64>( moog_tilt_status_topic_, 1 );
      std::cout << "Publishing to " << moog_zoom_status_topic_ << std::endl;
      moog_zoom_status_pub_ = nh_.advertise<std_msgs::Float64>( moog_zoom_status_topic_, 1 );
    }

    void publish( )
    {
      moog_pan_status_pub_.publish( *out_.status_pan );
      moog_tilt_status_pub_.publish( *out_.status_tilt );
      moog_zoom_status_pub_.publish( *out_.status_zoom );
    }

    void pan_cb( const std_msgs::Float64::ConstPtr& msg )
    {
      in_.pan->data = msg->data;
    }

    void tilt_cb( const std_msgs::Float64::ConstPtr& msg )
    {
      in_.tilt->data = msg->data;
    }

    void zoom_cb( const std_msgs::Float64::ConstPtr& msg )
    {
      in_.zoom->data = msg->data;
    }

  public:
    radl_in_t in_; 
    radl_in_flags_t in_flags_;
    radl_out_flags_t out_flags_;
    radl_out_t out_;

  private:
    ros::NodeHandle nh_; 

    std::string moog_pan_topic_;
    std::string moog_tilt_topic_;
    std::string moog_zoom_topic_;
    std::string moog_pan_status_topic_;
    std::string moog_tilt_status_topic_;
    std::string moog_zoom_status_topic_;

    ros::Subscriber moog_pan_sub_;
    ros::Subscriber moog_tilt_sub_;
    ros::Subscriber moog_zoom_sub_;
    ros::Publisher moog_pan_status_pub_;
    ros::Publisher moog_tilt_status_pub_;
    ros::Publisher moog_zoom_status_pub_;
};

int main(int argc, char **argv)
{
  ros::init( argc, argv, "landshark_moog_node" );
  ros::NodeHandle nh;

  LandsharkMoogRadl radl( nh );

  double rate_hz;
  ros::NodeHandle nh2( "~" );
  nh2.param<double>( "rate", rate_hz, 10 );
  std::cout << "moog.rate: " << rate_hz << std::endl;
  ros::Rate rate( rate_hz );

  try {
    LandsharkMoog moog;
    while (nh.ok()) {
      moog.step( &radl.in_, &radl.in_flags_, &radl.out_, &radl.out_flags_ );
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


