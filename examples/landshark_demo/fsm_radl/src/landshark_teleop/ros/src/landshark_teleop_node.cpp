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
#include <Joystick.h>
#include <LandsharkJoystick.h>

class LandsharkTeleopRadl {
  public:
    LandsharkTeleopRadl( ros::NodeHandle& nh ) 
      : nh_( nh )
    {
      // Get parameters from server (for testing)
      ros::NodeHandle nh2( "~" );
      nh2.param<std::string>( "base_topic", base_topic_, "/base" );
      nh2.param<std::string>( "deadman_topic", deadman_topic_, "/deadman" );
      nh2.param<std::string>( "over_ride_topic", over_ride_topic_, "/over_ride" );
      nh2.param<std::string>( "moog_pan_topic",  moog_pan_topic_,  "/moog/pan" );
      nh2.param<std::string>( "moog_tilt_topic", moog_tilt_topic_, "/moog/tilt" );
      nh2.param<std::string>( "moog_zoom_topic", moog_zoom_topic_, "/moog/zoom" );
      nh2.param<std::string>( "turret_pan_topic",  turret_pan_topic_,  "/turret/pan" );
      nh2.param<std::string>( "turret_tilt_topic", turret_tilt_topic_, "/turret/tilt" );
      nh2.param<std::string>( "paintball_trigger_topic", paintball_trigger_topic_, "/paintball/trigger" );
      
      std::cout << "[joy] Publishing to " << base_topic_ << std::endl;
      base_pub_ = nh_.advertise<geometry_msgs::Twist>( base_topic_, 1 );

      std::cout << "[joy] Publishing to " << deadman_topic_ << std::endl;
      deadman_pub_ = nh_.advertise<std_msgs::Bool>( deadman_topic_, 1 );

      std::cout << "[joy] Publishing to " << over_ride_topic_ << std::endl;
      over_ride_pub_ = nh_.advertise<std_msgs::Bool>( over_ride_topic_, 1 );

      std::cout << "[joy] Publishing to " << turret_pan_topic_ << std::endl;
      turret_pan_pub_ = nh_.advertise<std_msgs::Float64>( turret_pan_topic_, 1 );

      std::cout << "[joy] Publishing to " << turret_tilt_topic_ << std::endl;
      turret_tilt_pub_ = nh_.advertise<std_msgs::Float64>( turret_tilt_topic_, 1 );

      std::cout << "[joy] Publishing to " << moog_pan_topic_ << std::endl;
      moog_pan_pub_ = nh_.advertise<std_msgs::Float64>( moog_pan_topic_, 1 );

      std::cout << "[joy] Publishing to " << moog_tilt_topic_ << std::endl;
      moog_tilt_pub_ = nh_.advertise<std_msgs::Float64>( moog_tilt_topic_, 1 );

      std::cout << "[joy] Publishing to " << moog_zoom_topic_ << std::endl;
      moog_zoom_pub_ = nh_.advertise<std_msgs::Float64>( moog_zoom_topic_, 1 );

      std::cout << "[joy] Publishing to " << paintball_trigger_topic_ << std::endl;
      paintball_trigger_pub_ = nh_.advertise<std_msgs::UInt8>( paintball_trigger_topic_, 1 );
    }

    void publish( )
    {
      base_pub_.publish( *out_.base );
      deadman_pub_.publish( *out_.deadman );
      over_ride_pub_.publish( *out_.over_ride );
      moog_pan_pub_.publish( *out_.moog_pan );
      moog_tilt_pub_.publish( *out_.moog_tilt );
      moog_zoom_pub_.publish( *out_.moog_zoom );
      turret_pan_pub_.publish( *out_.turret_pan );
      turret_tilt_pub_.publish( *out_.turret_tilt );
      paintball_trigger_pub_.publish( *out_.trigger );
    }


  public:
    radl_in_t in_; 
    radl_in_flags_t in_flags_;
    radl_out_flags_t out_flags_;
    radl_out_t out_;

  private:
    ros::NodeHandle nh_; 

    ros::Publisher base_pub_;
    ros::Publisher deadman_pub_;
    ros::Publisher over_ride_pub_;
    ros::Publisher turret_pan_pub_;
    ros::Publisher turret_tilt_pub_;
    ros::Publisher moog_pan_pub_;
    ros::Publisher moog_tilt_pub_;
    ros::Publisher moog_zoom_pub_;
    ros::Publisher paintball_trigger_pub_;

    std::string base_topic_;
    std::string deadman_topic_;
    std::string over_ride_topic_;
    std::string turret_pan_topic_;
    std::string turret_tilt_topic_;
    std::string moog_pan_topic_;
    std::string moog_tilt_topic_;
    std::string moog_zoom_topic_;
    std::string paintball_trigger_topic_;
};

int main(int argc, char **argv)
{
  ros::init( argc, argv, "landshark_teleop" );
  ros::NodeHandle nh;

  // Get parameters from server (for testing)
  std::string dev;
  double rate_hz;
  ros::NodeHandle nh2( "~" );
  nh2.param<std::string>( "dev", dev, "/dev/input/js0" );
  nh2.param<double>( "rate", rate_hz, 100 );
  std::cout << "joystick.dev: " << dev << std::endl;
  std::cout << "joystick.rate: " << rate_hz << std::endl;

  LandsharkTeleopRadl radl( nh );

  LandsharkJoystick joy( dev );

  ros::Rate rate( rate_hz );
  while (nh.ok()) {
    joy.step( &radl.in_, &radl.in_flags_, &radl.out_, &radl.out_flags_ );
    radl.publish( );
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

