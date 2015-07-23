#include "a.h"
#include "ros/ros.h"

A::A() {
  this->counter = 0;
}

void A::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                  radl_out_t * out, radl_out_flags_t* outflags) {
    counter++;
    counter = counter%10000;
    out->a1->t_a = counter;
    ROS_INFO("sending: %d", counter);
}
