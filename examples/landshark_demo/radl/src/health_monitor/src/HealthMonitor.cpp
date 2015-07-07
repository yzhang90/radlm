#include "HealthMonitor.h"

HealthMonitor::HealthMonitor() {
  this->ocu_teleop_state = 0;
  this->gateway_teleop_state = 0;
  this->landshark_fsm_state = 0;
  this->landshark_base_state = 0;
}

void HealthMonitor::step(const radl_in_t *in, const radl_in_flags_t *inflags,
                         radl_out_t *out, radl_out_flags_t *outflags) {
  if(radl_is_timeout(inflags->ocu_teleop_report)) {
    ocu_teleop_state = ocu_teleop_state | 1;
  } else {
    if(radl_is_timeout(in->ocu_teleop_report->flag)) {
      ocu_teleop_state = ocu_teleop_state | 2;
    } else {
      ocu_teleop_state = 0;
    }
  }

  if(radl_is_timeout(inflags->gateway_teleop_report)) {
    gateway_teleop_state = gateway_teleop_state | 1;
  } else {
    if(radl_is_timeout(in->gateway_teleop_report->flag)) {
      gateway_teleop_state = gateway_teleop_state | 2;
    } else {
      gateway_teleop_state = 0;
    }
  }

  if(radl_is_timeout(inflags->landshark_fsm_report)) {
    lanshark_fsm_state = landshark_fsm_state | 1;
  } else {
    if(radl_is_timeout(in->landshark_fsm_report->flag)) {
      landshark_fsm_state = landshark_fsm_state | 2;
    } else {
      landshark_fsm_state = 0;
    }
  }

  if(radl_is_timeout(inflags->landshark_base_report)) {
    lanshark_base_state = landshark_base_state | 1;
  } else {
    if(radl_is_timeout(in->landshark_base_report->flag)) {
      landshark_base_state = landshark_base_state | 2;
    } else {
      landshark_base_state = 0;
    }
  }
  
  out->healthmonitor_state->ocu_teleop_state = this->ocu_teleop_state;
  out->healthmonitor_state->gateway_teleop_state = this->gateway_teleop_state;
  out->healthmonitor_state->landshark_fsm_state = this->landshark_fsm_state;
  out->healthmonitor_state->landshark_base_state = this->landshark_base_state;

}
