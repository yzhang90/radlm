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
#include "LandsharkBase.h"

class LandsharkBaseRadl {
  public:
    LandsharkBaseRadl( ros::NodeHandle& nh )
      : nh_( nh )
    {
      // Get parameters from server (for testing)
      ros::NodeHandle nh2( "~" );
      nh2.param<std::string>( "base_topic", base_topic_, "/base" );
      nh2.param<std::string>( "deadman_topic", deadman_topic_, "/deadman" );
      nh2.param<std::string>( "estop_topic", estop_topic_, "/estop" );
      nh2.param<std::string>( "status_topic", status_topic_, "/base_status" );
      nh2.param<std::string>( "battery_topic", battery_topic_, "/battery" );
      nh2.param<std::string>( "encoder_topic", encoder_topic_, "/encoder" );
      nh2.param<std::string>( "actuator_topic", actuator_topic_, "/actuator" );

      // Subscribe / Publish
      std::cout << "Subscribing to " << base_topic_ << std::endl;
      base_sub_ = nh_.subscribe<geometry_msgs::Twist>( base_topic_, 1, &LandsharkBaseRadl::base_callback, this );

      std::cout << "Subscribing to " << deadman_topic_ << std::endl;
      deadman_sub_ = nh_.subscribe<std_msgs::Bool>( deadman_topic_, 1, &LandsharkBaseRadl::deadman_callback, this );

      std::cout << "Subscribing to " << estop_topic_ << std::endl;
      estop_sub_ = nh_.subscribe<std_msgs::UInt8>( estop_topic_, 1, &LandsharkBaseRadl::estop_callback, this );

      std::cout << "Publishing to " << battery_topic_ << std::endl;
      battery_pub_ = nh_.advertise<std_msgs::Float32>( battery_topic_, 1 );
      std::cout << "Publishing to " << encoder_topic_ << std::endl;
      encoder_pub_ = nh_.advertise<landshark_msgs::WheelEncoder>( encoder_topic_, 1 );
      std::cout << "Publishing to " << actuator_topic_ << std::endl;
      actuator_pub_ = nh_.advertise<landshark_msgs::ActuatorInput>( actuator_topic_, 1 );

      std::cout << "Publishing to " << status_topic_ << std::endl;
      status_pub_ = nh_.advertise<std_msgs::UInt8>( status_topic_, 1 );
    }

    void base_callback( const geometry_msgs::Twist::ConstPtr& msg )
    {
      in_.base->angular = msg->angular;
      in_.base->linear = msg->linear;
    }

    void deadman_callback( const std_msgs::Bool::ConstPtr& msg )
    {
      in_.deadman->data = msg->data;
    }

    void estop_callback( const std_msgs::UInt8::ConstPtr& msg )
    {
      in_.estop->data = msg->data;
    }

    void publish( )
    {
      battery_pub_.publish( *out_.battery );
      encoder_pub_.publish( *out_.encoder );
      actuator_pub_.publish( *out_.actuator );
      status_pub_.publish( *out_.status );
    }

  public:
    radl_in_t in_;
    radl_in_flags_t in_flags_;
    radl_out_flags_t out_flags_;
    radl_out_t out_;

  private:
    ros::NodeHandle nh_;

    std::string deadman_topic_;
    std::string base_topic_;
    std::string estop_topic_;
    std::string status_topic_;
    std::string battery_topic_;
    std::string encoder_topic_;
    std::string actuator_topic_;

    ros::Subscriber base_sub_;
    ros::Subscriber deadman_sub_;
    ros::Subscriber estop_sub_;
    ros::Publisher status_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher encoder_pub_;
    ros::Publisher actuator_pub_;
};

int main(int argc, char **argv)
{
  ros::init( argc, argv, "landshark_base_node" );
  ros::NodeHandle nh;

  LandsharkBaseRadl radl( nh );

  double rate_hz;
  ros::NodeHandle nh2( "~" );
  nh2.param<double>( "rate", rate_hz, 100 );
  std::cout << "base.rate: " << rate_hz << std::endl;
  ros::Rate rate( rate_hz );

  try {
    LandsharkBase base;
    while (nh.ok()) {
      base.step( &radl.in_, &radl.in_flags_, &radl.out_, &radl.out_flags_ );
      radl.publish( );
      ros::spinOnce();
      rate.sleep();
    }
  }
  catch ( std::runtime_error e ) {
    std::cerr << "!!! Error: " << e.what() << " base driver has quit!" << std::endl;
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


