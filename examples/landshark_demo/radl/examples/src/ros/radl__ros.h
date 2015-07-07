#pragma once
#include "ros/ros.h"
#include RADL_HEADER

#define STR_VALUE(arg)      #arg
#define FUNCTION_NAME(name) STR_VALUE(name)
#define NODE_NAME FUNCTION_NAME(RADL_NODE_NAME)

struct radl__ros {
  inline radl__ros( const std::string name = NODE_NAME )
  {
    int argc = 0;
    std::cout << "Initializing ROS node: " << name << " ...";
    ros::init( argc, NULL, name.c_str(), ros::init_options::AnonymousName );
    std::cout << "done!" << std::endl;
  };
};

