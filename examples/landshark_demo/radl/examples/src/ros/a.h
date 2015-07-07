#pragma once
#include RADL_HEADER
#include "radl__ros.h"

#include "ros/ros.h"
#include "std_msgs/UInt8.h"

class NodeA {
  public:
    inline NodeA()
    {
      pub = nh.advertise<std_msgs::UInt8>("/topic_a", 2); 
    }

    inline void step(const radl_in_t* in, const radl_in_flags_t* i_f,
        radl_out_t* out, radl_out_flags_t* o_f) 
    {
      static uint8_t i( 0 );
      static std_msgs::UInt8 msg;
      msg.data = i++;
      pub.publish( msg );
    }

  private:
    // radl__ros should be the first variable declared before other ROS variables!
    radl__ros r; 
    ros::NodeHandle nh;
    ros::Publisher pub;
};

