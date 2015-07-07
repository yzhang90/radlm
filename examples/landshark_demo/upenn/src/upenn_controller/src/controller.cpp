/*
 Copyright (c) 2013, PRECISE Center, University of Pennsylvania
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 Neither the name of the University of Pennsylvania nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 Author(s): Nicola Bezzo (nicbezzo@seas.upenn.edu)
 Miroslav Pajic (pajic@seas.upenn.edu)
 */

#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <landshark_msgs/BatteryState.h>
#include <landshark_msgs/WheelEncoder.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <landshark_msgs/JointControl.h>
#include <landshark_msgs/PaintballTrigger.h>
#include <landshark_msgs/BoolStamped.h>

//a
// ...................................................includes for Simulink section
#include <landshark_simulatorN/solver.h>
#include <landshark_simulatorN/PID_d.h>
#include <landshark_simulatorN/PID_d_private.h>

#include <../src/ldl.c>
#include <../src/matrix_support.c>
#include <../src/util.c>
#include <../src/solver.c>

#define PI 3.14159265

//using namespace hacms;

// Define controller state: Active or Idle
landshark_msgs::BoolStamped gIsControllerActiveMsg;

// Define unique id for velocity commands
std::string gVelocitCommandId("UpennCruiseController");

Vars vars;
Params params;
Workspace work;
Settings settings;

double v_tot;
double velocity_imu;
double velocity;
double est_speed;
double number;
double output_s = 0.0;
double auto_trim = 0.0;
double trim = 0.0;
double autotrim_gain = 0.001;
//b

float v_x;
float v_y;
float v_z;
float p_x;
float p_y;
float p_ref;
float y_error;

float w_x;
float w_y;
float w_z;
float a_x;
float a_y;
float a_z;
float o_x;
float o_y;
float o_z;
float o_w;

float lat;
float lat_prev = 0.0;
float lon_prev = 0.0;
float delta_lat;
float delta_lon;
float lon;
float alt;
float lat_circun;
float x_gps;
float y_gps;
float x_gps_prev;
float y_gps_prev;

float gps_vel_x;
float gps_vel_y;
float gps_vel;

float battery_level;

int left_enc;
int right_enc;
int delta_left;
int cumulative_left = 0;
int cumulative_right = 0;
int delta_right;
int delta_left_enc;
int delta_right_enc;
double left_enc_previous = 0.0;
double right_enc_previous = 0.0;
double displacement_left;
double displacement_right;
double delta_t;
double divisor;
double enc_time_previous;
float v_enc_left;
float v_enc_right;

double bat_timeROS;
double odom_timeROS;
double enc_timeROS;
double imu_timeROS;
double gps_timeROS;
double cmd_timeROS;
double gps_vel_time;
double gps_vel_timeROS;

double odom_time;
double enc_time;
double imu_time;
double imu_time_prev = 0.0;
double imu_delta_t;

ros::Time gps_time;
double gps_time_prev = 0.0;
double gps_delta_t;

int attacked = 0;
int sensattacked = 0;
int resilient = 0;
float setspeed = 0.8;
float proportional_gain = 0.05; //0.04
float integrative_gain = 0.2; //0.03
float settrim = 0.0;

double cmd_time;

int counter = 0;
int attack_controlled = 0;
int count_enc = 1;
int count_odom = 0;
int count_attack = 0;
int count_attack2 = 0;
int count_attack_l = 0;
int count_attack2_l = 0;
int count_attack_r = 0;
int count_attack2_r = 0;
int noise = 1;

ros::Time enc_time_stamp;

// ------------------------------- ATTACK CALLBACK --------------------------------

void AttackCallback(const std_msgs::Int32::ConstPtr& attack_msg)
{
  attacked = attack_msg->data;
}

void SensorAttackCallback(const std_msgs::Int32::ConstPtr& sensattack_msg)
{
  sensattacked = sensattack_msg->data; // 0,1, ... , 8
}

// ------------------------------- RESILIENT CALLBACK -----------------------------

void ResilientCallback(const std_msgs::Int32::ConstPtr& resilient_msg)
{
  resilient = resilient_msg->data;
}

// ------------------------------- Trim CALLBACK -----------------------------

void TrimCallback(const std_msgs::Float32::ConstPtr& trim_msg)
{
  settrim = trim_msg->data;
}

// ------------------------------- AUTOTrim CALLBACK -----------------------------

void AutoTrimCallback(const std_msgs::Float32::ConstPtr& autotrim_msg)
{
  autotrim_gain = autotrim_msg->data;
}

// ------------------------------- SET SPEED CALLBACK -----------------------------

void SetSpeedCallback(const std_msgs::Float32::ConstPtr& setspeed_msg)
{
  setspeed = setspeed_msg->data;
}

// ------------------------------- SET KP CALLBACK -----------------------------

void KPCallback(const std_msgs::Float32::ConstPtr& kp_msg)
{
  proportional_gain = kp_msg->data;
}

// ------------------------------- SET KI CALLBACK -----------------------------

void KICallback(const std_msgs::Float32::ConstPtr& ki_msg)
{
  integrative_gain = ki_msg->data;
}

// ---------------------------------------------------------------- VELCOCITY COMMAND ID CHECKER
void checkVelocityCommandId(const geometry_msgs::TwistStampedConstPtr& pVelocityCommand)
{
  if (pVelocityCommand->header.frame_id != gVelocitCommandId)
  {
    // Preemptive, become idle
    gIsControllerActiveMsg.data = false;
    setspeed = 0;
  }
}

// ---------------------------------------------------------------- BATTERY CALLBACK

void BatteryCallback(const landshark_msgs::BatteryState::ConstPtr& battery_msg)
{
  battery_level = battery_msg->level;
  bat_timeROS = ros::Time::now().toSec();
  //ROS_INFO(" time = %f ---------------BATTERY: level = %f", bat_timeROS, battery_level);
}

// ---------------------------------------------------------------- ENCODER CALLBACK

void EncoderCallback(const landshark_msgs::WheelEncoder::ConstPtr& encoder_msg)
{

  left_enc = encoder_msg->left_encoder;
  right_enc = encoder_msg->right_encoder;
  enc_time = encoder_msg->header.stamp.toSec();
  enc_time_stamp = encoder_msg->header.stamp;
//Calculate each side velocity independently, like they are 2 different measurements

  delta_left_enc = left_enc - left_enc_previous;
  delta_right_enc = right_enc - right_enc_previous;
  displacement_left = 0.001371 * delta_left_enc;
  displacement_right = 0.001371 * delta_right_enc;
  delta_t = enc_time - enc_time_previous;
  divisor = (1 / delta_t);
  v_enc_left = divisor * displacement_left;
  v_enc_right = divisor * displacement_right;

  enc_timeROS = ros::Time::now().toSec();
//mp// ROS_INFO(" ENCODER: time = %f; left_enc_velocity = %f, right_enc_velocity = %f", enc_time, v_enc_left, v_enc_right);

  enc_time_previous = enc_time;
  left_enc_previous = left_enc;
  right_enc_previous = right_enc;
}

// ---------------------------------------------------------------- ODOMETRY CALLBACK
void OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg)
{
  v_x = odometry_msg->twist.twist.linear.x;
  v_y = odometry_msg->twist.twist.linear.y;
  v_z = odometry_msg->twist.twist.angular.z;
  p_x = odometry_msg->pose.pose.position.x;
  p_y = odometry_msg->pose.pose.position.y;

  odom_time = odometry_msg->header.stamp.toSec();

  odom_timeROS = ros::Time::now().toSec();
//mp// ROS_INFO(" ODOM: time = %f; v_x= %f, v_y= %f, w_z= %f", odom_time, v_x, v_y, v_z);

//a
  v_tot = odometry_msg->twist.twist.linear.x;
//b

}

// ---------------------------------------------------------------- IMU CALLBACK

void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  a_x = imu_msg->linear_acceleration.x;
  a_y = imu_msg->linear_acceleration.y;
  a_z = imu_msg->linear_acceleration.z;
  w_x = imu_msg->angular_velocity.x;
  w_y = imu_msg->angular_velocity.y;
  w_z = imu_msg->angular_velocity.z;
  o_x = imu_msg->orientation.x;
  o_y = imu_msg->orientation.y;
  o_z = imu_msg->orientation.z;
  o_w = imu_msg->orientation.w;
  imu_time = imu_msg->header.stamp.toSec();

  imu_delta_t = imu_time - imu_time_prev;
  imu_timeROS = ros::Time::now().toSec();

//ROS_INFO(" time = %f ---------------IMU: acc_x= %f acc_y= %f acc_z= %f    w_x= %f w_y= %f w_z= %f", imu_time, a_x, a_y, a_z, w_x, w_y, w_z);

//a
  velocity_imu = a_x * imu_delta_t;
//b
  imu_time_prev = imu_time;
}

// ---------------------------------------------------------------- GPS CALLBACK
void GpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{
  lat = gps_msg->latitude;
  lon = gps_msg->longitude;
  alt = gps_msg->altitude;
  gps_time = gps_msg->header.stamp;

  gps_timeROS = ros::Time::now().toSec();
//ROS_INFO(" time = %f ---------------GPS: latitude= %f longitude= %f altitude= %f", gps_time, lat, lon, alt);

}

void GpsVelCallback(const geometry_msgs::TwistStamped::ConstPtr& gps_vel_msg)
{
  gps_vel_x = gps_vel_msg->twist.linear.x;
  gps_vel_y = gps_vel_msg->twist.linear.y;
  gps_vel_time = gps_vel_msg->header.stamp.toSec();

  gps_vel = sqrt((gps_vel_x * gps_vel_x) + (gps_vel_y * gps_vel_y));
  gps_vel_timeROS = ros::Time::now().toSec();
//ROS_INFO(" GPS: time = %f; gps_vel= %f", gps_vel_time, gps_vel);

}

void ControllerStateCallback(const landshark_msgs::BoolStamped::ConstPtr& rp_state_msg)
{
  gIsControllerActiveMsg = *rp_state_msg;
}

//a
/* Block states (auto storage) */// ------------------------------------------from SIMULINK
D_Work_PID_d PID_d_DWork;

/* External inputs (root inport signals with auto storage) */
ExternalInputs_PID_d PID_d_U;

/* External outputs (root outports fed by signals with auto storage) */
ExternalOutputs_PID_d PID_d_Y;

/* Real-time model */
RT_MODEL_PID_d PID_d_M_;
RT_MODEL_PID_d * const PID_d_M = &PID_d_M_;
int32_T div_s32_floor(int32_T numerator, int32_T denominator)
{
  int32_T quotient;
  uint32_T absNumerator;
  uint32_T absDenominator;
  uint32_T tempAbsQuotient;
  uint32_T quotientNeedsNegation;
  if (denominator == 0)
  {
    quotient = numerator >= 0 ? MAX_int32_T : MIN_int32_T;

    /* Divide by zero handler */
  }
  else
  {
    absNumerator = (uint32_T)(numerator >= 0 ? numerator : -numerator);
    absDenominator = (uint32_T)(denominator >= 0 ? denominator : -denominator);
    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absDenominator == 0 ? MAX_uint32_T : absNumerator / absDenominator;
    if (quotientNeedsNegation)
    {
      absNumerator %= absDenominator;
      if (absNumerator > 0)
      {
        tempAbsQuotient++;
      }
    }

    quotient = quotientNeedsNegation ? -(int32_T)tempAbsQuotient : (int32_T)tempAbsQuotient;
  }

  return quotient;
}

void PID_d_initialize(void) //------------------------------------------from SIMULINK....INITIALIZE PID
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(PID_d_M, (NULL));

  /* states (dwork) */
  (void)memset((void *)&PID_d_DWork, 0, sizeof(D_Work_PID_d));

  /* external inputs */
  (void)memset((void *)&PID_d_U, 0, sizeof(ExternalInputs_PID_d));

  /* external outputs */
  PID_d_Y.u = 0.0;

  /* Start for DataStoreMemory: '<S1>/Data Store Memory1' */
  memcpy(&PID_d_DWork.Y[0], &PID_d_P.DataStoreMemory1_InitialValue[0], 12U * sizeof(real_T));

  /* Start for DataStoreMemory: '<S1>/Data Store Memory6' */
  memcpy(&PID_d_DWork.U[0], &PID_d_P.DataStoreMemory6_InitialValue[0], 21U * sizeof(real_T));

  /* InitializeConditions for Delay: '<S1>/Delay1' */
  PID_d_DWork.Delay1_DSTATE = PID_d_P.Delay1_InitialCondition;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Integrator1' */
  PID_d_DWork.Integrator1_DSTATE = PID_d_P.Integrator1_IC;

  /* InitializeConditions for DiscreteIntegrator: '<S1>/Filter' */
  PID_d_DWork.Filter_DSTATE = PID_d_P.Filter_IC;
}

//b

// ---------------------------------------------------------------- MAIN
int main(int argc, char **argv)
{
  gIsControllerActiveMsg.data = false;

  //ROS_INFO("HELLO");
  ros::init(argc, argv, "UpennCruiseController");
  ros::NodeHandle node;
  ros::NodeHandle privateNodehandle("~");

  gVelocitCommandId = std::string(ros::this_node::getName());

  std::string controllerStateControlTopic, controllerStateStatusTopic;

  privateNodehandle.param("controller_state_control_topic", controllerStateControlTopic,
                          std::string("/landshark/upenn_controller_state/control"));
  privateNodehandle.param("controller_state_status_topic", controllerStateStatusTopic,
                          std::string("/landshark/upenn_controller_state/status"));

  ros::Publisher vel_pub;
  vel_pub = node.advertise<geometry_msgs::TwistStamped>("/landshark_control/base_velocity", 1);

  ros::Publisher estimated_vel_pub;
  estimated_vel_pub = node.advertise<geometry_msgs::TwistStamped>("/landshark_control/estimated_velocity", 1);

  ros::Publisher ref_pub;
  ref_pub = node.advertise<geometry_msgs::TwistStamped>("/landshark_control/reference_velocity", 1);

  ros::Publisher enc_left_pub;
  enc_left_pub = node.advertise<geometry_msgs::TwistStamped>("/landshark/left_encoder_velocity", 1);

  ros::Publisher enc_right_pub;
  enc_right_pub = node.advertise<geometry_msgs::TwistStamped>("/landshark/right_encoder_velocity", 1);

  ros::Publisher gps_vel_pub;
  gps_vel_pub = node.advertise<geometry_msgs::TwistStamped>("/landshark/gps_velocity_2", 1);

  ros::Publisher controller_state_pub = node.advertise<landshark_msgs::BoolStamped>(controllerStateStatusTopic, 1);

  ros::Subscriber velocity_command_checker_subscriber = node.subscribe<geometry_msgs::TwistStamped>("/landshark_control/base_velocity", 10, checkVelocityCommandId);

  ros::Subscriber battery_subscriber;
  battery_subscriber = node.subscribe<landshark_msgs::BatteryState>("/landshark/battery", 1, BatteryCallback);

  ros::Subscriber encoder_subscriber;
  encoder_subscriber = node.subscribe<landshark_msgs::WheelEncoder>("/landshark/wheel_encoder", 1, EncoderCallback);

  ros::Subscriber odom_subscriber;
  odom_subscriber = node.subscribe<nav_msgs::Odometry>("/landshark/odom", 1, OdometryCallback);

  ros::Subscriber imu_subscriber;
  imu_subscriber = node.subscribe<sensor_msgs::Imu>("/landshark/imu", 1, ImuCallback);

  ros::Subscriber gps_subscriber;
  gps_subscriber = node.subscribe<sensor_msgs::NavSatFix>("/landshark/gps", 1, GpsCallback);

  ros::Subscriber gps_vel_subscriber;
  gps_vel_subscriber = node.subscribe<geometry_msgs::TwistStamped>("/landshark/gps_velocity", 1, GpsVelCallback);

  ros::Subscriber attack_subscriber;
  attack_subscriber = node.subscribe<std_msgs::Int32>("/landshark_demo/run_attack", 1, AttackCallback);

  ros::Subscriber sensattack_subscriber;
  sensattack_subscriber = node.subscribe<std_msgs::Int32>("/landshark_demo/sensor_attack", 1, SensorAttackCallback);

  ros::Subscriber resilient_subscriber;
  resilient_subscriber = node.subscribe<std_msgs::Int32>("/landshark_demo/run_rc", 1, ResilientCallback);

  ros::Subscriber setspeed_subscriber;
  setspeed_subscriber = node.subscribe<std_msgs::Float32>("/landshark_demo/desired_speed", 1, SetSpeedCallback);

  ros::Subscriber trim_subscriber;
  trim_subscriber = node.subscribe<std_msgs::Float32>("/landshark_demo/trim", 1, TrimCallback);

  ros::Subscriber autotrim_subscriber;
  autotrim_subscriber = node.subscribe<std_msgs::Float32>("/landshark_demo/autotrim", 1, AutoTrimCallback);

  ros::Subscriber kp_subscriber;
  kp_subscriber = node.subscribe<std_msgs::Float32>("/landshark_demo/kp", 1, KPCallback);

  ros::Subscriber ki_subscriber;
  ki_subscriber = node.subscribe<std_msgs::Float32>("/landshark_demo/ki", 1, KICallback);

  ros::Subscriber controller_state_subscriber = node.subscribe<landshark_msgs::BoolStamped>(controllerStateControlTopic,
                                                                                            1, ControllerStateCallback);

  geometry_msgs::TwistStamped command;
  geometry_msgs::TwistStamped left_enc;
  geometry_msgs::TwistStamped right_enc;
  geometry_msgs::TwistStamped reference_vel;
  geometry_msgs::TwistStamped estimated_speed;
  geometry_msgs::TwistStamped gps_2;
  ros::Rate r(50.0); //----------------------------------------------------------------------------------------RATE

  int count = 0;

// ----------------------------------------------------------------------------------------------------------------- WHILE
  while (ros::ok())
  {
    ros::spinOnce();

    if (gIsControllerActiveMsg.data)
    {

      //geometry_msgs::TwistStamped command;
      //a
      PID_d_U.v[0] = v_enc_left; //-----------------> measured velocity from ENCODER RIGHT
      PID_d_U.v[1] = v_enc_right; //----------------> measured velocity from ENCODER LEFT
      PID_d_U.v[2] = gps_vel; //--------------------> measured velocity from GPS

// -----------------------------------------------------------------------------------ATTACK
      if (sensattacked == 4 || sensattacked == 5 || sensattacked == 6 || sensattacked == 7)
      {
        if (attacked == 1)
        {
          PID_d_U.v[2] = gps_vel + 3.0; //v_tot
        }
        else if (attacked == 2)
        {
          if ((count_attack2 >= 0) && (count_attack2 < 100))
          {
            //noise = rand() % 100 + 1;
            PID_d_U.v[2] = gps_vel - 3.0 + (0.06 * noise);
          }
          else if (count_attack2 >= 100)
          {
            count_attack2 = 0;
            noise = rand() % 100 + 1;
            PID_d_U.v[2] = gps_vel - 3.0 + (0.06 * noise);

          }
          count_attack2++;
        }
        else if (attacked == 3)
        {
          if ((count_attack >= 0) && (count_attack < 100))
          {
            PID_d_U.v[2] = gps_vel + 1.5;
            count_attack++;
          }
          else if ((count_attack >= 100) && (count_attack < 200))
          {
            PID_d_U.v[2] = gps_vel - 1.5;
            count_attack++;
          }
          else if (count_attack >= 200)
          {
            PID_d_U.v[2] = gps_vel + 1.5;
            count_attack = 1;
          }
        }
      }

      if (sensattacked == 2 || sensattacked == 3 || sensattacked == 6 || sensattacked == 7)
      {
        if (attacked == 1)
        {
          PID_d_U.v[0] = v_enc_left + 3.0;
        }
        else if (attacked == 2)
        {
          if ((count_attack2_l >= 0) && (count_attack2_l < 100))
          {
            //noise = rand() % 100 + 1;
            PID_d_U.v[0] = v_enc_left - 3.0 + (0.06 * noise);
          }
          else if (count_attack2_l >= 100)
          {
            count_attack2_l = 0;
            noise = rand() % 100 + 1;
            PID_d_U.v[0] = v_enc_left - 3.0 + (0.06 * noise);

          }
          count_attack2_l++;
        }
        else if (attacked == 3)
        {
          if ((count_attack_l >= 0) && (count_attack_l < 100))
          {
            PID_d_U.v[0] = v_enc_left + 1.5;
            count_attack_l++;
          }
          else if ((count_attack_l >= 100) && (count_attack_l < 200))
          {
            PID_d_U.v[0] = v_enc_left - 1.5;
            count_attack_l++;
          }
          else if (count_attack_l >= 200)
          {
            PID_d_U.v[0] = v_enc_left + 1.5;
            count_attack_l = 1;
          }
        }
      }

      if (sensattacked == 1 || sensattacked == 3 || sensattacked == 5 || sensattacked == 7)
      {
        if (attacked == 1)
        {
          PID_d_U.v[1] = v_enc_right + 3.0;
        }
        else if (attacked == 2)
        {
          if ((count_attack2_r >= 0) && (count_attack2_r < 100))
          {
            //noise = rand() % 100 + 1;
            PID_d_U.v[1] = v_enc_right - 3.0 + (0.06 * noise);
          }
          else if (count_attack2_r >= 100)
          {
            count_attack2_r = 0;
            noise = rand() % 100 + 1;
            PID_d_U.v[1] = v_enc_right - 3.0 + (0.06 * noise);

          }
          count_attack2_r++;
        }
        else if (attacked == 3)
        {
          if ((count_attack_r >= 0) && (count_attack_r < 100))
          {
            PID_d_U.v[1] = v_enc_right + 1.5;
            count_attack_r++;
          }
          else if ((count_attack_r >= 100) && (count_attack_r < 200))
          {
            PID_d_U.v[1] = v_enc_right - 1.5;
            count_attack_r++;
          }
          else if (count_attack_r >= 200)
          {
            PID_d_U.v[1] = v_enc_right + 1.5;
            count_attack_r = 1;
          }
        }
      }

      left_enc.header.stamp = enc_time_stamp;
      left_enc.twist.linear.x = PID_d_U.v[0];
      right_enc.header.stamp = enc_time_stamp;
      right_enc.twist.linear.x = PID_d_U.v[1];
      enc_left_pub.publish(left_enc);
      enc_right_pub.publish(right_enc);

      gps_2.header.stamp = gps_time;
      gps_2.twist.linear.x = PID_d_U.v[2];
      gps_vel_pub.publish(gps_2);

// ******************************************************************************************************************* SIMULINK PART
// *******************************************************************************************************************

      /* Model step function */

      /* local block i/o variables */
      real_T rtb_yout[3];
      real_T Ytil[12];
      int32_T b;
      int32_T kk;
      int32_T j;
      int32_T m;
      real_T y[7];
      int32_T ar;
      int32_T ia;
      real_T rtb_Sum2;
      real_T rtb_FilterCoefficient;
      int32_T i;
      real_T tmp[49];
      real_T tmp_0[49];
      real_T tmp_1[49];
      real_T tmp_data[21];
      int32_T tmp_sizes;
      real_T tmp_data_0[63];
      real_T temp_idx;

      real_T InputOffset = -0.60579294300116315; //MP
      real_T SatInWO_offset; //MP

      /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
       *  Delay: '<S1>/Delay1'
       *  Inport: '<Root>/v'
       *  SignalConversion: '<S2>/TmpSignal ConversionAt SFunction Inport2'
       */
      /* MATLAB Function 'PID_d/MATLAB Function': '<S2>:1' */
      if (2.0 > PID_d_P.SFunction_p5)
      {
        ar = 0;
        b = 0;
      }
      else
      {
        ar = 1;
        b = (int32_T)PID_d_P.SFunction_p5;
      }

      /* '<S2>:1:12' */
      ia = b - ar;
      for (i = 0; i <= (b - ar) - 1; i++)
      {
        Ytil[3 * i] = PID_d_DWork.Y[(ar + i) * 3];
        Ytil[1 + 3 * i] = PID_d_DWork.Y[(ar + i) * 3 + 1];
        Ytil[2 + 3 * i] = PID_d_DWork.Y[(ar + i) * 3 + 2];
      }

      for (i = 0; i <= ia - 1; i++)
      {
        PID_d_DWork.Y[3 * i] = Ytil[3 * i];
        PID_d_DWork.Y[1 + 3 * i] = Ytil[3 * i + 1];
        PID_d_DWork.Y[2 + 3 * i] = Ytil[3 * i + 2];
      }

      /* '<S2>:1:13' */
      i = (int32_T)PID_d_P.SFunction_p5;
      PID_d_DWork.Y[3 * (i - 1)] = PID_d_U.v[0];
      PID_d_DWork.Y[1 + 3 * (i - 1)] = PID_d_U.v[1];
      PID_d_DWork.Y[2 + 3 * (i - 1)] = PID_d_U.v[2];
      rtb_Sum2 = (PID_d_P.SFunction_p5 - 2.0) * 7.0;
      if (8.0 > (PID_d_P.SFunction_p5 - 1.0) * 7.0)
      {
        ar = 0;
      }
      else
      {
        ar = 7;
      }

      /* '<S2>:1:14' */
      if (1.0 > rtb_Sum2)
      {
        i = 0;
      }
      else
      {
        i = (int32_T)rtb_Sum2;
      }

      tmp_sizes = i - 1;
      for (ia = 0; ia <= i - 1; ia++)
      {
        tmp_data[ia] = PID_d_DWork.U[ia];
      }

      for (i = 0; i <= tmp_sizes; i++)
      {
        PID_d_DWork.U[ar + i] = tmp_data[i];
      }

      //manualy modified by MP //MP
      /* '<S2>:1:15' */
      /*
       for (i = 0; i < 7; i++) {
       PID_d_DWork.U[i] = PID_d_P.SFunction_p2[i + 7] * PID_d_DWork.Delay1_DSTATE +
       PID_d_P.SFunction_p2[i] * PID_d_DWork.Delay1_DSTATE;
       }
       */

      SatInWO_offset = PID_d_DWork.Delay1_DSTATE + InputOffset;

//mp// printf(">>>>>>>>>>>>>>>>>>>>>satinwo=%f\n", SatInWO_offset);
      if (SatInWO_offset < 0)
        SatInWO_offset = 0;

      for (i = 0; i < 7; i++)
      {
        PID_d_DWork.U[i] = PID_d_P.SFunction_p2[i + 7] * SatInWO_offset + PID_d_P.SFunction_p2[i] * SatInWO_offset;
      }
      // end of the modification

      /* '<S2>:1:18' */
      memcpy(&Ytil[0], &PID_d_DWork.Y[0], 12U * sizeof(real_T));

      /* '<S2>:1:20' */
      for (kk = 0; kk <= (int32_T)(PID_d_P.SFunction_p5 - 1.0) - 1; kk++)
      {
        /* '<S2>:1:20' */
        j = (int32_T)((1.0 + (real_T)kk) * 7.0);
        rtb_Sum2 = ((PID_d_P.SFunction_p5 - 1.0) - (1.0 + (real_T)kk)) * 7.0;
        rtb_FilterCoefficient = (PID_d_P.SFunction_p5 - 1.0) * 7.0;
        if (rtb_Sum2 + 1.0 > rtb_FilterCoefficient)
        {
          m = 0;
          ia = 0;
        }
        else
        {
          m = (int32_T)(rtb_Sum2 + 1.0) - 1;
          ia = (int32_T)rtb_FilterCoefficient;
        }

        /* '<S2>:1:21' */
        if (ia - m == 1)
        {
          for (i = 0; i <= j - 1; i++)
          {
            tmp_data_0[3 * i] = PID_d_P.SFunction_p4[3 * i];
            tmp_data_0[1 + 3 * i] = PID_d_P.SFunction_p4[3 * i + 1];
            tmp_data_0[2 + 3 * i] = PID_d_P.SFunction_p4[3 * i + 2];
          }

          tmp_sizes = (ia - m) - 1;
          for (i = 0; i <= (ia - m) - 1; i++)
          {
            tmp_data[i] = PID_d_DWork.U[m + i];
          }

          rtb_Sum2 = 0.0;
          for (i = 0; i <= tmp_sizes; i++)
          {
            rtb_Sum2 += tmp_data_0[3 * i] * tmp_data[i];
          }

          rtb_FilterCoefficient = 0.0;
          for (i = 0; i <= tmp_sizes; i++)
          {
            rtb_FilterCoefficient += tmp_data_0[3 * i + 1] * tmp_data[i];
          }

          temp_idx = 0.0;
          for (i = 0; i <= tmp_sizes; i++)
          {
            temp_idx += tmp_data_0[3 * i + 2] * tmp_data[i];
          }
        }
        else
        {
          rtb_Sum2 = 0.0;
          rtb_FilterCoefficient = 0.0;
          temp_idx = 0.0;
          ar = -1;
          for (b = 0; b + 1 <= j; b++)
          {
            if (PID_d_DWork.U[m + b] != 0.0)
            {
              ia = ar + 1;
              tmp_sizes = div_s32_floor(ia, 3);
              rtb_Sum2 += PID_d_P.SFunction_p4[ia % 3 + 3 * tmp_sizes] * PID_d_DWork.U[m + b];
              ia++;
              tmp_sizes = div_s32_floor(ia, 3);
              rtb_FilterCoefficient += PID_d_P.SFunction_p4[ia % 3 + 3 * tmp_sizes] * PID_d_DWork.U[m + b];
              ia++;
              tmp_sizes = div_s32_floor(ia, 3);
              temp_idx += PID_d_P.SFunction_p4[ia % 3 + 3 * tmp_sizes] * PID_d_DWork.U[m + b];
            }

            ar += 3;
          }
        }

        /* '<S2>:1:22' */
        i = (int32_T)((1.0 + (real_T)kk) + 1.0);
        ia = (int32_T)((1.0 + (real_T)kk) + 1.0);
        Ytil[3 * (i - 1)] = PID_d_DWork.Y[(ia - 1) * 3] - rtb_Sum2;
        Ytil[1 + 3 * (i - 1)] = PID_d_DWork.Y[(ia - 1) * 3 + 1] - rtb_FilterCoefficient;
        Ytil[2 + 3 * (i - 1)] = PID_d_DWork.Y[(ia - 1) * 3 + 2] - temp_idx;

        /* '<S2>:1:20' */
      }

      /* '<S2>:1:28' */
      /* '<S2>:1:29' */
      /* '<S2>:1:30' */
      /* '<S2>:1:31' */
      /* '<S2>:1:33' */
      /* '<S2>:1:34' */
      /* '<S2>:1:35' */
      /* '<S2>:1:36' */

      params.y_0[0] = Ytil[0]; // ------------------------------------> update for the Ytil
      params.y_0[1] = Ytil[1];
      params.y_0[2] = Ytil[2];
      params.y_1[0] = Ytil[3];
      params.y_1[1] = Ytil[4];
      params.y_1[2] = Ytil[5];
      params.y_2[0] = Ytil[6];
      params.y_2[1] = Ytil[7];
      params.y_2[2] = Ytil[8];
      params.y_3[0] = Ytil[9];
      params.y_3[1] = Ytil[10];
      params.y_3[2] = Ytil[11];

///////////////////////printf("ytil_0=%f\n", Ytil[0]);

      /* %%%%%%%%%%%%%%%%%%%%%%%%%  0 */
      /*  % % % % % % % % % % % % % % state_x = csolve_mine(params); */
      /*  yout = Y(:,1); */
      /*  state_estim = vars.x; */
      /* '<S2>:1:49' */
      /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%% state_x(1); */
      /* '<S2>:1:52' */

//................................................................................................................................SOLVER
      int num_iters;
#if (NUMTESTS > 0)
      int i;
      double time;
      double time_per;
#endif
      set_defaults();
      setup_indexing();
      load_default_data();

      /* Solve problem instance for the record. */
      settings.verbose = 0;
      // num_iters = solve();

      solve();

      //  printf("solution: %f \n", vars.x[0]);

//................................................................................................................................

      rtb_Sum2 = vars.x[1]; //-------------------------------------------------------------  v_estim

      PID_d_U.r = setspeed;

      reference_vel.header.stamp = ros::Time::now();
      reference_vel.twist.linear.x = setspeed;
      ref_pub.publish(reference_vel);

      est_speed = rtb_Sum2;

      count++;

      rtb_Sum2 = PID_d_U.r - rtb_Sum2; //  ------------------------------------  reference  - v_estim
//mp// printf("reference value = %f   error = %f\n", PID_d_U.r, rtb_Sum2);

      if (resilient == 0)
      {
//  if (attacked == 1)
//      {
        rtb_Sum2 = (PID_d_U.v[0] + PID_d_U.v[1] + PID_d_U.v[2]) / 3;
        est_speed = rtb_Sum2;
        //  printf("<<<<<<ATTACK>>>>>> V_ESTIM=%f\n", rtb_Sum2);
        rtb_Sum2 = PID_d_U.r - rtb_Sum2; //  ------------------------------------  reference  - v_estim
        //  printf("<<<<<<ATTACK>>>>>> reference value = %f   error = %f\n", PID_d_U.r, rtb_Sum2);
//     }

      }

//.......................................................................................................................... PID
      /* Gain: '<S1>/Filter Coefficient' incorporates:
       *  DiscreteIntegrator: '<S1>/Filter'
       *  Gain: '<S1>/Derivative Gain'
       *  Sum: '<S1>/SumD'
       */

      PID_d_P.ProportionalGain_Gain = proportional_gain;
      PID_d_P.IntegralGain_Gain = integrative_gain;

      rtb_FilterCoefficient = (PID_d_P.DerivativeGain_Gain * rtb_Sum2 - PID_d_DWork.Filter_DSTATE)
          * PID_d_P.FilterCoefficient_Gain;

      /* Outport: '<Root>/u' incorporates:
       *  DiscreteIntegrator: '<S1>/Integrator1'
       *  Gain: '<S1>/Proportional Gain'
       *  Sum: '<S1>/Sum'
       */
      PID_d_Y.u = ((PID_d_P.ProportionalGain_Gain * rtb_Sum2 + PID_d_DWork.Integrator1_DSTATE) + rtb_FilterCoefficient); // + 0.05;

      if (PID_d_Y.u > 1)
      {
        PID_d_Y.u = 1;
      }
      if (PID_d_Y.u < 0)
      {
        PID_d_Y.u = 0;
      }

      /* Update for Delay: '<S1>/Delay1' */
      PID_d_DWork.Delay1_DSTATE = PID_d_Y.u;
      double outputpid = (PID_d_Y.u);
//printf("PID_output=%f\n", outputpid);

      /* Update for DiscreteIntegrator: '<S1>/Integrator1' incorporates:
       *  Gain: '<S1>/Integral Gain'
       */
      PID_d_DWork.Integrator1_DSTATE += PID_d_P.IntegralGain_Gain * rtb_Sum2 * PID_d_P.Integrator1_gainval;

      /* Update for DiscreteIntegrator: '<S1>/Filter' */
      PID_d_DWork.Filter_DSTATE += PID_d_P.Filter_gainval * rtb_FilterCoefficient;

// **********************************************************************************************************
// ********************************************************************************************************** END SIMULINK PART

      command.twist.linear.x = outputpid;        // 0.5;//0.5556; //(20/36); //0.65

      command.twist.angular.z = settrim; //;output_s; // auto_trim; // settrim;// auto_trim;
//mp// printf("output=%f\n", outputpid);

      estimated_speed.header.stamp = ros::Time::now();
      estimated_speed.twist.linear.x = est_speed;
      estimated_vel_pub.publish(estimated_speed);

//-------------------------------PUBLISH COMMAND

      cmd_time = command.header.stamp.toSec();

      cmd_timeROS = ros::Time::now().toSec();

      command.header.frame_id = gVelocitCommandId;

      vel_pub.publish(command);
      counter++;
      double time_sim = ros::Time::now().toSec();
    } // if (gIsControllerActiveMsg.data)

    controller_state_pub.publish(gIsControllerActiveMsg);

    r.sleep();
  } // while(ros::ok())

  return 0;

}

void load_default_data(void)
{

  params.CAs_0[0] = 0; //0.04331042079065206;
  params.CAs_0[1] = 0; // 1.5717878173906188;
  params.CAs_0[2] = 0; //1.5851723557337523;
  params.CAs_0[3] = 1; //-1.497658758144655;
  params.CAs_0[4] = 1; //-1.171028487447253;
  params.CAs_0[5] = 1; //-1.7941311867966805;
  params.CAs_0[6] = 0; //-0.23676062539745413;
  params.CAs_0[7] = 0; //-1.8804951564857322;
  params.CAs_0[8] = 0; //-0.17266710242115568;
  params.CAs_0[9] = 0; //0.596576190459043;
  params.CAs_0[10] = 0; //-0.8860508694080989;
  params.CAs_0[11] = 0; //0.7050196079205251;
  params.CAs_0[12] = 0; //0.3634512696654033;
  params.CAs_0[13] = 0; //-1.9040724704913385;
  params.CAs_0[14] = 0; //0.23541635196352795;
  params.CAs_0[15] = 0; //-0.9629902123701384;
  params.CAs_0[16] = 0; // -0.3395952119597214;
  params.CAs_0[17] = 0; //-0.865899672914725;
  params.CAs_0[18] = 0; //0.7725516732519853;
  params.CAs_0[19] = 0; //-0.23818512931704205;
  params.CAs_0[20] = 0; //-1.372529046100147
  //params.y_1[0] =  11.100335622518406; //0.17859607212737894;
  //params.y_1[1] =  9.990302060266565; //1.1212590580454682;
  //params.y_1[2] = 13.320402747022086; //-0.774545870495281;
  params.CAs_1[0] = 0;  //-1.1121684642712744;
  params.CAs_1[1] = 0; //-0.44811496977740495;
  params.CAs_1[2] = 0; //1.7455345994417217;
  params.CAs_1[3] = 0.94160905144045348; //1.9039816898917352;
  params.CAs_1[4] = 0.94160905144045348; //0.6895347036512547;
  params.CAs_1[5] = 0.94160905144045348; //1.6113364341535923;
  params.CAs_1[6] = 0; //1.383003485172717;
  params.CAs_1[7] = 0; //-0.48802383468444344;
  params.CAs_1[8] = 0; //-1.631131964513103;
  params.CAs_1[9] = 0; //0.6136436100941447;
  params.CAs_1[10] = 0; //0.2313630495538037;
  params.CAs_1[11] = 0; //-0.5537409477496875;
  params.CAs_1[12] = 0; //-1.0997819806406723;
  params.CAs_1[13] = 0; //-0.3739203344950055;
  params.CAs_1[14] = 0; //-0.12423900520332376;
  params.CAs_1[15] = 0; //-0.923057686995755;
  params.CAs_1[16] = 0; //-0.8328289030982696;
  params.CAs_1[17] = 0; //-0.16925440270808823;
  params.CAs_1[18] = 0; //1.442135651787706;
  params.CAs_1[19] = 0; //0.34501161787128565;
  params.CAs_1[20] = 0; //-0.8660485502711608;
  //params.y_2[0] = 11.100663516987090; //-0.8880899735055947;
  //params.y_2[1] = 9.990597165288381; //-0.1815116979122129;
  //params.y_2[2] = 13.320796220384509; //-1.17835862158005;
  params.CAs_2[0] = 0; //-1.1944851558277074;
  params.CAs_2[1] = 0; //0.05614023926976763;
  params.CAs_2[2] = 0; //-1.6510825248767813;
  params.CAs_2[3] = 0.88662760575459054; //-0.06565787059365391;
  params.CAs_2[4] = 0.88662760575459054; //-0.5512951504486665;
  params.CAs_2[5] = 0.88662760575459054; //0.8307464872626844;
  params.CAs_2[6] = 0; //0.9869848924080182;
  params.CAs_2[7] = 0; //0.7643716874230573;
  params.CAs_2[8] = 0; //0.7567216550196565;
  params.CAs_2[9] = 0; //-0.5055995034042868;
  params.CAs_2[10] = 0; //0.6725392189410702;
  params.CAs_2[11] = 0; //-0.6406053441727284;
  params.CAs_2[12] = 0; //0.29117547947550015;
  params.CAs_2[13] = 0; //-0.6967713677405021;
  params.CAs_2[14] = 0; //-0.21941980294587182;
  params.CAs_2[15] = 0; //-1.753884276680243;
  params.CAs_2[16] = 0; //-1.0292983112626475;
  params.CAs_2[17] = 0; //1.8864104246942706;
  params.CAs_2[18] = 0; //-1.077663182579704;
  params.CAs_2[19] = 0; //0.7659100437893209;
  params.CAs_2[20] = 0; //0.6019074328549583;
  //params.y_3[0] = 11.100983153348627; //0.8957565577499285;
  //params.y_3[1] = 9.990884838013765; //-0.09964555746227477;
  //params.y_3[2] = 13.321179784018353; //0.38665509840745127;
  params.CAs_3[0] = 0; //-1.7321223042686946;
  params.CAs_3[1] = 0; //-1.7097514487110663;
  params.CAs_3[2] = 0; //-1.2040958948116867;
  params.CAs_3[3] = 0.83485657883550035; //-1.3925560119658358;
  params.CAs_3[4] = 0.83485657883550035; //-1.5995826216742213;
  params.CAs_3[5] = 0.83485657883550035; //-1.4828245415645833;
  params.CAs_3[6] = 0; //0.21311092723061398;
  params.CAs_3[7] = 0; //-1.248740700304487;
  params.CAs_3[8] = 0; //1.808404972124833;
  params.CAs_3[9] = 0; //0.7264471152297065;
  params.CAs_3[10] = 0; //0.16407869343908477;
  params.CAs_3[11] = 0; //0.8287224032315907;
  params.CAs_3[12] = 0; //-0.9444533161899464;
  params.CAs_3[13] = 0; //1.7069027370149112;
  params.CAs_3[14] = 0; //1.3567722311998827;
  params.CAs_3[15] = 0; //0.9052779937121489;
  params.CAs_3[16] = 0; //-0.07904017565835986;
  params.CAs_3[17] = 0; //1.3684127435065871;
  params.CAs_3[18] = 0; //0.979009293697437;
  params.CAs_3[19] = 0; //0.6413036255984501;
  params.CAs_3[20] = 0; //1.6559010680237511;

}
