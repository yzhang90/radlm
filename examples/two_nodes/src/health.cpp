#include "health.h"
#include "ros/ros.h"

void Health::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                  radl_out_t * out, radl_out_flags_t* outflags) {
  if(radl_is_timeout(inflags->a_report)) {
    ROS_INFO("node a is dead");
  } else {
    if(radl_is_timeout(in->a_report->flag)) {
      ROS_INFO("node a received timeout messages");
    }
  }

  if(radl_is_timeout(inflags->c_report)) {
    ROS_INFO("node c is dead");
  } else {
    if(radl_is_timeout(in->c_report->flag)) {
      ROS_INFO("node c received timeout messages");
    }  
  }

}
