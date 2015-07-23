#include "HealthMonitor.h"
#include "ros/ros.h"

void HealthMonitor::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                         radl_out_t * out, radl_out_flags_t* outflags) {

  if(radl_is_timeout(inflags->two_nodes_a_report)) {
    ROS_INFO("node two_nodes.a is dead");
  } else {
    if(radl_is_timeout(in->two_nodes_a_report->flag)) {
      ROS_INFO("node two_nodes.a received timeout messages");
    }
  }

  if(radl_is_timeout(inflags->two_nodes_c_report)) {
    ROS_INFO("node two_nodes.c is dead");
  } else {
    if(radl_is_timeout(in->two_nodes_c_report->flag)) {
      ROS_INFO("node two_nodes.c received timeout messages");
    }
  }

}
