#include "ccc.h"

CCC::CCC() {}

int CCC::step(const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags) {
  radl_turn_off(radl_STALE, &out_flags->ccc_base);
  radl_turn_off(radl_TIMEOUT, &out_flags->ccc_base);

  radl_turn_off(radl_STALE, &out_flags->ccc_status);
  radl_turn_off(radl_TIMEOUT, &out_flags->ccc_status);
  
  out->ccc_base->linear.x = 0;
  out->ccc_base->linear.y = 0;
  out->ccc_base->linear.z = 0;
  out->ccc_base->angular.x = 0;
  out->ccc_base->angular.y = 0;
  out->ccc_base->angular.z = 0;

  out->ccc_status->data = radlast_constants()->CCCStatus->ENGAGED_NO_GUARANTEE;

  return 0;
}
