#include "LandsharkStateEstimator.h"
#include <math.h>
#include <stdio.h>

extern "C"
{
#include "RSE.h"
#include "rtwtypes.h"
}

using namespace std;

LandsharkStateEstimator::LandsharkStateEstimator() 
{
  RSE_initialize();
  enc_time_previous = 0;
  left_enc_previous = 0;
  right_enc_previous = 0;

  gps_age = 0;
  encoder_age = 0;
  rse_status = radlast_constants()->RSEStatus->INIT;
  rse_wait = 0;

  v_enc_left_previous = 0;
  v_enc_right_previous = 0;
  v_gps_previous = 0;
  act_left_previous = 0;
  act_right_previous = 0;
}

LandsharkStateEstimator::~LandsharkStateEstimator()
{
  RSE_terminate();
}

int LandsharkStateEstimator::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags )
{
  int left_enc, right_enc, delta_left_enc, delta_right_enc, enc_time, delta_t;
  double displacement_left, displacement_right, divisor, v_enc_left, v_enc_right;

  int encoder_stale = 0; // 0 ==> not, 1 ==> stale
  int gps_stale = 0;

// Sensor message: Encoder
  if (in_flags->encoder & 1) encoder_stale = 1; // ROS_INFO("Encoder message is stale.");
  if (in_flags->encoder & 16); // ROS_INFO("Encoder message is timeout."); // We don't care about timeout for now.

  left_enc = in->encoder->left;
  right_enc = in->encoder->right;
  enc_time = in->encoder->stamp;

//Calculate each side velocity independently, like they are 2 different measurements

  delta_left_enc = left_enc - left_enc_previous;
  delta_right_enc = right_enc - right_enc_previous;
  displacement_left = 0.001371 * delta_left_enc;
  displacement_right = 0.001371 * delta_right_enc;
  delta_t = enc_time - enc_time_previous;
  if (delta_t > 0)
    {
      divisor = ( 1000000000.0 / delta_t );
      v_enc_left = divisor * displacement_left;
      v_enc_right = divisor * displacement_right;

      enc_time_previous = enc_time;
      left_enc_previous = left_enc;
      right_enc_previous = right_enc;
      v_enc_left_previous = v_enc_left;
      v_enc_right_previous = v_enc_right;

      encoder_age = 0;
    }
  else encoder_stale = 1; // This case is equivalent to 'stale'. //ROS_INFO("problem in the time stamp of the encoder message"); 

  // enc_timeROS = ros::Time::now().toSec();
  //mp// ROS_INFO(" ENCODER: time = %f; left_enc_velocity = %f, right_enc_velocity = %f", enc_time, v_enc_left, v_enc_right);


// Sensor message: GPS
  if (in_flags->gps_velocity & 1) gps_stale = 1; // ROS_INFO("GPS velocity message is stale.");
  if (in_flags->gps_velocity & 16); // ROS_INFO("GPS velocity message is timeout."); // We don't care about timeout for now.

  if (in->gps_velocity->linear.x == NAN)
    gps_stale = 1; // This case is equivalent to 'stale'. // ROS_INFO("gps velocity is NAN"); 

  if (gps_stale == 0) 
  {
    gps_age = 0;
    v_gps_previous = in->gps_velocity->linear.x;
  }

  RSE_U.y[0] = v_enc_left_previous;
  RSE_U.y[1] = v_enc_right_previous;
  RSE_U.y[2] = v_gps_previous;
  //RSE_U.y[2] = (v_enc_left_previous + v_enc_right_previous) / 2.0;

  int i;
  for(i=0; i<3; i++) {
    if (RSE_U.y[i] < 0) RSE_U.y[i] = 0;
    if (RSE_U.y[i] > 2) RSE_U.y[i] = 2;
  }

  // Control input
  if (in_flags->actuator & 1); // We don't care about this for now. // ROS_INFO("Base message is stale");
  if (in_flags->actuator & 16); // We don't care about this for now. // ROS_INFO("Base message is timeout");
  if (in->actuator->left == NAN)
    ; // ROS_INFO("act_left is NAN");
  else
    act_left_previous = in->actuator->left;

  if (in->actuator->right == NAN)
    ; // ROS_INFO("act_left is NAN");
  else
    act_right_previous = in->actuator->right;

  RSE_U.u[0] = act_left_previous;
  RSE_U.u[1] = act_right_previous;

  RSE_step();
  if (RSE_Y.v_estim < 0) RSE_Y.v_estim = 0;
  if (RSE_Y.v_estim > 2) RSE_Y.v_estim = 2;
  out->est_velocity->linear.x = RSE_Y.v_estim;

  //out_flags->est_velocity = 0; 

  // RSE Status
  // rse_status==0 ==> INIT
  // rse_status==1 ==> GOOD
  // rse_status==2 ==> BAD

  const int t_rse = 4; // time required to initialize RSE

  int SA_flag = 1; // The inital value 1 means that the assumption is NOT violated.
  if(encoder_age >= 2) SA_flag = 0;
  //gps_age >= 15

  if (rse_status == radlast_constants()->RSEStatus->INIT) { // INIT
    rse_wait++;
    if (SA_flag) {
      if (rse_wait > t_rse) {
        rse_wait = 0;
        rse_status = radlast_constants()->RSEStatus->GOOD; // Goto GOOD
      }
    }
    else rse_status = radlast_constants()->RSEStatus->BAD; // Goto BAD
  }
  else if(rse_status == radlast_constants()->RSEStatus->GOOD) { // GOOD
    if (!SA_flag) rse_status = radlast_constants()->RSEStatus->BAD; // Goto BAD
  }
  else if(rse_status == radlast_constants()->RSEStatus->BAD) { // BAD
    if (SA_flag) rse_status = radlast_constants()->RSEStatus->INIT; // Goto INIT
  }

  out->rse_status->data = rse_status;

  encoder_age++;
  gps_age++;
  return 0;
}
