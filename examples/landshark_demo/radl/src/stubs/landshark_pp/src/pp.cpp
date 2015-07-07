#include "pp.h"

PP::PP() 
{
  //ROS_INFO( "testing" );
}

int PP::step(const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags) {
  static uint8_t status( radlast_constants()->PPStatus->ON );
  static radl_duration_t engage_time = radl_gettime();

  radl_turn_off(radl_STALE, &out_flags->pp_base);
  radl_turn_off(radl_TIMEOUT, &out_flags->pp_base);
  radl_turn_off(radl_STALE, &out_flags->pp_status);
  radl_turn_off(radl_TIMEOUT, &out_flags->pp_status);
  radl_turn_off(radl_STALE, &out_flags->estop);
  radl_turn_off(radl_TIMEOUT, &out_flags->estop);

  if ( in->pp_request->data == radlast_constants()->PPRequest->ENGAGE ) {
    status = radlast_constants()->PPStatus->ENGAGED;
    engage_time = radl_gettime();
  }
  else if ( in->pp_request->data == radlast_constants()->PPRequest->DISENGAGE ) {
    status = radlast_constants()->PPStatus->ON;
  }
  radl_duration_t time = radl_gettime();
  int64_t sec;
  uint32_t nsec;
  radl_to_secnsec( radl_timesub( engage_time, time ), &sec, &nsec );
  if ( sec > 10 ) {
    status = radlast_constants()->PPStatus->GOAL_SUCCESS;
  }
  
  out->pp_base->linear.x = 0;
  out->pp_base->linear.y = 0;
  out->pp_base->linear.z = 0;
  out->pp_base->angular.x = 0;
  out->pp_base->angular.y = 0;
  out->pp_base->angular.z = 0;

  out->pp_status->data = status;
  out->estop->data = radlast_constants()->EStop->NONE;

  return 0;
}
