#ifndef HEALTHMONITOR_H
#define HEALTHMONITOR_H

#include RADL_HEALDER

class HealthMonitor {
  private:
    int ocu_teleop_state;
    int gateway_teleop_state;
    int landshark_fsm_state;
    int landshark_base_state;
  public:
    HealthMonitor();
    void step(const radl_in_t*, const radl_in_flags_t*, radl_out_t*, radl_out_flags_t*);

};

#endif
