#include "fsm.h"
#include <iostream>

using namespace std;

FSM::FSM(): 
  state_(STATE_JOY),
  ccc_wait_cycle_(0),
  pp_wait_cycle_(0),
  estop_wait_cycle_(0),
  ccc2joy_wait_cycle_(0),
  pp2joy_wait_cycle_(0),
  estop2joy_wait_cycle_(0)
{
}


int FSM::step(const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags) {
  // default output
  // data
  out->base->angular = in->joy_base->angular;
  out->base->linear = in->joy_base->linear;
  out->base->stamp = in->joy_base->stamp;
  out->base_estop->data = radlast_constants()->EStop->NONE;
  out->ccc_request->data = radlast_constants()->CCCRequest->NONE;
  out->pp_request->data = radlast_constants()->PPRequest->NONE;
  out->controller_status->data = radlast_constants()->ControllerStatus->JOY;
  // flags
  out_flags->ccc_request = 0;
  out_flags->pp_request = 0;
  out_flags->controller_status = 0;
  out_flags->base_estop = in_flags->teleop_estop; // ignore the flags from monitor_estop for now
  out_flags->base = in_flags->joy_base;

  switch (state_) { // Mealy semantics
  case STATE_JOY: // joystick
    if (in->teleop_estop->data==radlast_constants()->EStop->SET) { // need to estop
      state_ = STATE_2ESTOP;
      // send estop signal
      out->base_estop->data = radlast_constants()->EStop->SET;
    } else {    
      if (in->select->data==radlast_constants()->ControllerSelect->CCC) {
	state_ = STATE_JOY2CCC;
	// send request to CCC
	out->ccc_request->data = radlast_constants()->CCCRequest->ENGAGE;
      }
      if (in->select->data==radlast_constants()->ControllerSelect->PP) {
	state_ = STATE_JOY2PP;
	// send request to PP
	out->pp_request->data = radlast_constants()->PPRequest->ENGAGE;
      }
    }
    break;
  case STATE_JOY2CCC: // transitioning from joystick to CCC
    if (in->ccc_status->data==radlast_constants()->CCCStatus->ENGAGED_GUARANTEE or in->ccc_status->data==radlast_constants()->CCCStatus->ENGAGED_NO_GUARANTEE) {
      state_ = STATE_CCC;
      // control values from CCC
      out->base->angular = in->ccc_base->angular;
      out->base->linear = in->ccc_base->linear;
      out->base->stamp = in->ccc_base->stamp;
      out_flags->base = in_flags->ccc_base;
      out->controller_status->data = radlast_constants()->ControllerStatus->CCC;
      ccc_wait_cycle_ = 0;
    } else {
      if (ccc_wait_cycle_ >= radlast_constants()->MaxWaitCycle->CCC) {
	state_ = STATE_CCC2JOY;
	ccc_wait_cycle_ = 0;
	// send disengage signal
	out->ccc_request->data = radlast_constants()->CCCRequest->DISENGAGE;
      } else { // keep trying to engage
	ccc_wait_cycle_++;
	out->ccc_request->data = radlast_constants()->CCCRequest->ENGAGE;
      }
    }
    break;
  case STATE_CCC: // constant cruise control
    if (in->select->data==radlast_constants()->ControllerSelect->JOY || in->over_ride->data) { // back to joystick
      state_ = STATE_CCC2JOY;
      // send disengage signal
      out->ccc_request->data = radlast_constants()->CCCRequest->DISENGAGE;
    } else if (in->teleop_estop->data==radlast_constants()->EStop->SET) { // need to estop
      state_ = STATE_2ESTOP;
      // send estop signal
      out->base_estop->data = radlast_constants()->EStop->SET;
      // send disengage signal
      out->ccc_request->data = radlast_constants()->CCCRequest->DISENGAGE;
    } else { // stay in ccc mode
      // control values from CCC
      out->base->angular = in->ccc_base->angular;
      out->base->linear = in->ccc_base->linear;
      out->base->stamp = in->ccc_base->stamp;
      out_flags->base = in_flags->ccc_base;
      out->controller_status->data = radlast_constants()->ControllerStatus->CCC;
    }
    break;
  case STATE_CCC2JOY: // transitioning from CCC back to joystick
    if (ccc2joy_wait_cycle_ >= radlast_constants()->MaxWaitCycle->CCC2JOY) {
      // back to joystick
      state_ = STATE_JOY;
      ccc2joy_wait_cycle_ = 0;
    } else {
      ccc2joy_wait_cycle_++;
      // keep sending disengage
      out->ccc_request->data = radlast_constants()->CCCRequest->DISENGAGE;
    }
    break;
  case STATE_JOY2PP: // transitioning from joystick to PP
    if (in->pp_status->data==radlast_constants()->PPStatus->ENGAGED) {
      state_ = STATE_PP;
      // control values from PP
      out->base->angular = in->pp_base->angular;
      out->base->linear = in->pp_base->linear;
      out->base->stamp = in->pp_base->stamp;
      out_flags->base = in_flags->pp_base;
      out->controller_status->data = radlast_constants()->ControllerStatus->PP;
      pp_wait_cycle_ = 0;
    } else {
      if (pp_wait_cycle_ >= radlast_constants()->MaxWaitCycle->PP) {
	state_ = STATE_PP2JOY;
	pp_wait_cycle_ = 0;
	// send disengage signal
	out->pp_request->data = radlast_constants()->PPRequest->DISENGAGE;
      } else { // keep trying to engage
	pp_wait_cycle_++;
	out->pp_request->data = radlast_constants()->PPRequest->ENGAGE;
      }
    }
    break;
  case STATE_PP: // path planner
    if (in->select->data==radlast_constants()->ControllerSelect->JOY || in->over_ride->data) { // back to joystick
      state_ = STATE_PP2JOY;
      // send disengage signal
      out->pp_request->data = radlast_constants()->PPRequest->DISENGAGE;
    } else if (in->teleop_estop->data==radlast_constants()->EStop->SET || in->monitor_estop->data==radlast_constants()->EStop->SET) { // need to estop
      state_ = STATE_2ESTOP;
      // send estop signal
      out->base_estop->data = radlast_constants()->EStop->SET;
      // send disengage signal
      out->pp_request->data = radlast_constants()->PPRequest->DISENGAGE;
    } else { // stay in pp mode
      // control values from PP
      out->base->angular = in->pp_base->angular;
      out->base->linear = in->pp_base->linear;
      out->base->stamp = in->pp_base->stamp;
      out_flags->base = in_flags->pp_base;
      out->controller_status->data = radlast_constants()->ControllerStatus->PP;
    }
    break;
  case STATE_PP2JOY: // transitioning from PP back to joystick
    if (pp2joy_wait_cycle_ >= radlast_constants()->MaxWaitCycle->PP2JOY) {
      // back to joystick
      state_ = STATE_JOY;
      pp2joy_wait_cycle_ = 0;
    } else {
      pp2joy_wait_cycle_++;
      // keep sending disengage
      out->pp_request->data = radlast_constants()->PPRequest->DISENGAGE;
    }
    break;
  case STATE_2ESTOP: // transitioning to ESTOP
    if (in->base_status->data==radlast_constants()->BaseStatus->ESTOP) {
      state_ = STATE_ESTOP;
      // control values shouldn't matter
      out->controller_status->data = radlast_constants()->ControllerStatus->ESTOP;
      estop_wait_cycle_ = 0;
    } else {
      if (estop_wait_cycle_ >= radlast_constants()->MaxWaitCycle->ESTOP) {      
      state_ = STATE_ESTOP2JOY;
      estop_wait_cycle_ = 0;
      // send reset signal
      out->base_estop->data = radlast_constants()->EStop->RESET;
      } else { // keep trying to set the base to estop mode
      estop_wait_cycle_++;
      out->base_estop->data = radlast_constants()->EStop->SET;
      // send disengage signal to both PP and CCC
      out->pp_request->data = radlast_constants()->PPRequest->DISENGAGE;
      out->ccc_request->data = radlast_constants()->CCCRequest->DISENGAGE;
      }
    }
    break;
  case STATE_ESTOP: // estop 
    if (in->select->data==radlast_constants()->ControllerSelect->JOY || in->over_ride->data) { // back to joystick
      state_ = STATE_ESTOP2JOY;
      // send reset signal
      out->base_estop->data = radlast_constants()->EStop->RESET;
    } else { // stay in estop mode
      // control values shouldn't matter
      out->controller_status->data = radlast_constants()->ControllerStatus->ESTOP;
    }
    break;
  case STATE_ESTOP2JOY: // transitioning from ESTOP back to joystick
    if (estop2joy_wait_cycle_ >= radlast_constants()->MaxWaitCycle->ESTOP2JOY) {
      // back to joystick
      state_ = STATE_JOY;
      estop2joy_wait_cycle_ = 0;
    } else {
      estop2joy_wait_cycle_++;
      // keep sending reset
      out->base_estop->data = radlast_constants()->EStop->RESET;
    }
    break;
  default:
    cerr << "Invalid state" << endl;
  }
  return 0;
}

