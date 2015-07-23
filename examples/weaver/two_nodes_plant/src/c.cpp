#include "c.h"
#include "ros/ros.h"

void C::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                  radl_out_t * out, radl_out_flags_t* outflags) {
    ROS_INFO("received from a: %d, flags: %d", in->a1->t_a, inflags->a1);
}


