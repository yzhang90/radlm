#include "LandsharkPaintballWrapper.h"

void LandsharkPaintballWrapper::step(const radl_in_t * in, const radl_in_flags_t* inflags,
                                     radl_out_t * out, radl_out_flags_t* outflags) {
   radl_in_t m_in;
   radlt_6_topics_27_t__teleop_paintball_trigger m_trigger;
   radlt_7_mtopics_17_t__safezone_state m_safezone_state;

   m_trigger = *(in->trigger);
   m_safezone_state = *(in->safezone_state);
   
   m_in.trigger = &m_trigger;
   m_in.safezone_state = &m_safezone_state;

   if(in->safezone_state->state == 0){
       m_trigger.data = 0;
   }

   _node.step(&m_in, inflags, out, outflags);
}
