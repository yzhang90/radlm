#include "ai.h"
#include "ros/ros.h"

AI::AI() {
}

void AI::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                  radl_out_t * out, radl_out_flags_t* outflags) {
    _node.step(in, inflags, out, outflags);
    out->a_report->flag = outflags->a_report;
    outflags->a_report = radl_STALE;
}
