#include "fsm.h"
#include <iostream>

using namespace std;

FSM::FSM(): 
  state_(STATE_JOY),
  ccc_wait_cycle_(0),
  pp_wait_cycle_(0)
{
}


int FSM::step(const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags) {
  // default out
  out->base->angular = in->joy_base->angular;
  out->base->linear = in->joy_base->linear;
  out->ccc_request->data = radlast_constants()->CCCRequest->NONE;
  out->pp_request->data = radlast_constants()->PPRequest->NONE;
  out->base_estop->data = in->estop->data;
  // handle estop first
  if (in->estop->data==radlast_constants()->EStop->SET) { // encounter estop
    if (state_==STATE_PP || state_==STATE_CCC) { // go to ESTOP state only in auto-mode (PP or CCC)
      state_ = STATE_ESTOP;
    }	
  } else if (in->select->data==radlast_constants()->ControllerSelect->JOY) { // can go back to manual control from any state
    if (state_==STATE_ESTOP) { // reset ESTOP to the base
      out->base_estop->data = radlast_constants()->EStop->RESET;
    }
    state_ = STATE_JOY;
    out->ccc_request->data = radlast_constants()->CCCRequest->DISENGAGE;
    out->pp_request->data = radlast_constants()->PPRequest->DISENGAGE;
  } else {
    switch (state_) {
      case STATE_JOY: // joystick 				
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
      case STATE_JOY2CCC: // transitioning from joystick to CCC
        if (in->ccc_status->data==radlast_constants()->CCCStatus->ENGAGED_GUARANTEE or in->ccc_status->data==radlast_constants()->CCCStatus->ENGAGED_NO_GUARANTEE) {
          state_ = STATE_CCC;
          ccc_wait_cycle_ = 0;
        } else if (in->ccc_status->data==radlast_constants()->CCCStatus->FAILURE) {
          state_ = STATE_JOY;
          ccc_wait_cycle_ = 0;
          // send disengage request
          out->ccc_request->data = radlast_constants()->CCCRequest->DISENGAGE;
        } else if (in->ccc_status->data==radlast_constants()->CCCStatus->OFF) {
          state_ = STATE_JOY;
          ccc_wait_cycle_ = 0;
          cerr << "CCC is OFF." << endl;
        } else {
          // ON or TRYING
          if (ccc_wait_cycle_ >= radlast_constants()->MaxWaitCycle->CCC) {
            state_ = STATE_JOY;
            ccc_wait_cycle_ = 0;
            // send disengage signal
            out->ccc_request->data = radlast_constants()->CCCRequest->DISENGAGE;
          } else {
            ccc_wait_cycle_++;
            // keep engaging
            out->ccc_request->data = radlast_constants()->CCCRequest->ENGAGE;
          }
        }
      case STATE_CCC:
        // control values from CCC
        out->base->angular = in->ccc_base->angular;
        out->base->linear = in->ccc_base->linear;
      case STATE_JOY2PP: // transitioning from joystick to PP
        if (in->pp_status->data==radlast_constants()->PPStatus->ENGAGED) {
          state_ = STATE_PP;
          pp_wait_cycle_ = 0;
        } else if (in->pp_status->data==radlast_constants()->PPStatus->FAILURE) {
          state_ = STATE_JOY;
          pp_wait_cycle_ = 0;
          // send disengage request
          out->pp_request->data = radlast_constants()->PPRequest->DISENGAGE;
        } else if (in->pp_status->data==radlast_constants()->PPStatus->OFF) {
          state_ = STATE_JOY;
          pp_wait_cycle_ = 0;
          cerr << "PP is OFF." << endl;
        } else {
          // ON or TRYING
          if (pp_wait_cycle_ >= radlast_constants()->MaxWaitCycle->PP) {
            state_ = STATE_JOY;
            pp_wait_cycle_ = 0;
            // send disengage signal
            out->pp_request->data = radlast_constants()->PPRequest->DISENGAGE;
          } else {
            pp_wait_cycle_++;
            // keep engaging
            out->pp_request->data = radlast_constants()->PPRequest->ENGAGE;
          }
        }
      case STATE_PP:
        // control values from PP
        out->base->angular = in->pp_base->angular;
        out->base->linear = in->pp_base->linear;
        // TODO: handle GOAL_SUCCESS and GOAL_FAILURE in some special way
      case STATE_ESTOP:                        
      default:
        cerr << "Invalid state" << endl;
    }
  }
  return 0;
}

void FSM::printState() {
  cout << state_ << endl;
}

