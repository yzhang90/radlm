#include "TwoNodesA_Wrapper.h"
#include "ros/ros.h"

void TwoNodesA_Wrapper::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                           radl_out_t * out, radl_out_flags_t* outflags) {
   _node.step(in, inflags, out, outflags);
   out->two_nodes_a_report->flag = outflags->two_nodes_a_report;
   outflags->two_nodes_a_report = radl_STALE;
}
