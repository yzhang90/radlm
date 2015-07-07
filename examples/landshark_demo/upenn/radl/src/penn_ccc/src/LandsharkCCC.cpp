#include "LandsharkCCC.h"

extern "C"
{
#include "CCC.h"
#include "rtwtypes.h"
}
using namespace std;

LandsharkCCC::LandsharkCCC() 
{
  CCC_initialize();
}

LandsharkCCC::~LandsharkCCC()
{
  CCC_terminate();
}

int LandsharkCCC::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags )
{
  CCC_U.v_estim = in->est_velocity->linear.x; 
  CCC_U.ref = in->ref_velocity->data;
  if(CCC_U.ref > 2.0) CCC_U.ref = 2.0;
  if(CCC_U.ref < 0.0) CCC_U.ref = 0.0;
  CCC_U.RESET = 0.0;

  static int counter = 0;
  static bool engFlag =  false; //true means it is still in engaing process

  out->ccc_status->data = radlast_constants()->CCCStatus->ON; //default status is ON

  if(counter == 0 && in->ccc_request->data == radlast_constants()->CCCRequest->ENGAGE){
    CCC_U.RESET = 1.0;
    engFlag = true;
  }

  if(engFlag){
   out->ccc_status->data = radlast_constants()->CCCStatus->ENGAGED_NO_GUARANTEE;
   ++counter;
    if(counter == 50){
      engFlag = false;
    }
  }
  if(in->ccc_request->data == radlast_constants()->CCCRequest->DISENGAGE){
    counter = 0;
    engFlag = false;
    out->ccc_status->data = radlast_constants()->CCCStatus->ON;
  }

  if(counter == 50){  //when counter == 50 means it is engaged. Counter is only reset to 0 when request disengage
    out->ccc_status->data = radlast_constants()->CCCStatus->ENGAGED_GUARANTEE;
  }
  if(out->ccc_status->data == radlast_constants()->CCCStatus->ENGAGED_GUARANTEE || out->ccc_status->data == radlast_constants()->CCCStatus->ENGAGED_NO_GUARANTEE){

  }
    CCC_step();

  out->base->linear.x = CCC_Y.ctrl_u;

//  out_flags->base = 0;
//  out_flags->ccc_status = 0;

  return 0;
}
