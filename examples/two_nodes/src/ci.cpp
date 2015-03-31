#include "ci.h"
#include "ros/ros.h"

CI::CI() {
}

void CI::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                  radl_out_t * out, radl_out_flags_t* outflags) {
    _node.step(in, inflags, out, outflags);
    out->c_report->flag = outflags->c_report;
    outflags->c_report = radl_STALE;
}
