/*
 * Copyright (C) 2012, SRI International (R)
 *
 * The material contained in this release is copyrighted. It may not be copied,
 * reproduced, translated, reverse engineered, modified or reduced to any electronic
 * medium or machine-readable form without the prior written consent of
 * SRI International (R).
 *
 * Portions of files in this release may be unpublished work
 * containing SRI International (R) CONFIDENTIAL AND PROPRIETARY INFORMATION.
 * Disclosure, use, reverse engineering, modification, or reproduction without
 * written authorization of SRI International (R) is likewise prohibited.
 *
 * Author(s): Thomas de Candia (thomasd@ai.sri.com)
 *            Aravind Sundaresan (aravind@ai.sri.com)
 *
 */

#include "LandsharkBase.h"
#include <boost/interprocess/sync/posix/ptime_to_timespec.hpp>
using namespace boost::interprocess::ipcdetail;

#include <sstream>
#include <exception>
#include <cmath>
#include <stdlib.h>
#include <assert.h>

const double MAX_VELOCITY = 32.0;
const double MIN_VELOCITY = -MAX_VELOCITY;
const unsigned int MAX_PWM = 9999;
const int SIXTEEN_BIT_SIGNED_MAX_NBR = 32767;
const int SIXTEEN_BIT_SIGNED_MIN_NBR = -32768;
const int SIXTEEN_BIT_REP_NBR = 65536;
const double BATTERY_VOLTAGE_FACTOR = 50.0;
const double MAX_BATTERY_ADC_READING = 4095;
const double MAX_BATTERY_LEVEL = 38.2; // V
const double MIN_BATTERY_LEVEL = 37.24; // V
const size_t BATTERY_FILTER_SIZE = 100.0;
const double UNKNOWN_COV = 99999.0;
const double ODOM_COV_x = 0.0025;
const double ODOM_COV_y = 0.0025;
const double ODOM_COV_yaw = 0.09;

  LandsharkBase::LandsharkBase()
  : m_IsCommunicationRunning(false)
  , m_DesiredProcessPeriod_us(2e4)
  , m_CommandUpdateRate(COMMAND_UPDATE_RATE)
  , m_LedBlinkingRate(LED_BLINKING_RATE)
  , m_AdcUpdateRate(ADC_UPDATE_RATE)
  , m_MotorTimeOutRate(MOTOR_TIME_OUT_RATE)
  , m_ProcessVelocityCommand(false)
  , m_ProcessPwmCommand(false)
  , m_IsFirstEncoderReading(true)
  , m_WheelBase( 0.6477 )     // m
  , m_MaxMotorSpeed( 4.7 )    // m/s
  , m_MaxMotorInput( 50 )    // motor units
  , m_MotorRatio( m_MaxMotorInput / m_MaxMotorSpeed )
  , m_BatteryLevelArraySize( 10 ) 
  , estop( false )
  , pleft( 0 )
  , pright( 0 )
  , dleft( 0 )
    , dright( 0 )
{
}

LandsharkBase::~LandsharkBase()
{
}

/**
 * \brief convert twist to left and right wheel inputs
 *
 * theta = arc / r, where
 *   r = wheel_base/2 (radius of turn)
 *   arc is distance traveled by each wheel (arc on circle)
 *   theta is the angle turned in radians
 */ 

void LandsharkBase::to_left_right( const double t, const double a, double& left, double& right )
{
  double t2 = a * m_WheelBase / 2;
  left = m_MotorRatio * ( t + t2 );
  right = m_MotorRatio * ( t - t2 );
}

/**
 * \brief The step function takes in estop, twist commands
 *        
 * The landshark step() function performs the following.
 * 1. If in->estop is SET, it sets a local ESTOP state which sends zero 
 *    to the motors until a RESET is received.
 *
 * 2. If in->estop == RESET the local ESTOP flag is reset. 
 *
 * 3. if in->estop == NONE/RESET, the input twist command is relayed to the motors
 * 
 * Output
 * - status (ESTOP, NORMAL)
 * - battery
 * - encoder 
 * - last command sent
 */

int LandsharkBase::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags ) {
  // delta is the "decelerating" constant
  const double delta( 5 );
  uint8_t status( radlast_constants()->BaseStatus->NORMAL );

  if ( in->estop->data == radlast_constants()->EStop->SET ) {
    estop = true;
  }
  else if ( in->estop->data == radlast_constants()->EStop->RESET ) {
    estop = false;
  }

  // left and right wheel actuators
  double left( 0 );
  double right( 0 );

  if ( estop == true ) {
    left = copysign( std::max( 0.0, fabs( pleft ) - delta ), pleft );
    right = copysign( std::max( 0.0, fabs( pright ) - delta ), pright );
    status = radlast_constants()->BaseStatus->ESTOP;
  }
  else { 
    status = radlast_constants()->BaseStatus->NORMAL;
    if ( radl_is_failing( in_flags->base ) ) {
      left = 0;
      right = 0;
      status = radlast_constants()->BaseStatus->FAILURE;
      std::cout << "[base] RADL failure flags set!" << std::endl;
    }
    else {
      // compute motor input from twist after limiting to allowable range
      double t = in->base->linear.x;
      double a = in->base->angular.z;
      to_left_right( t, a, left, right );
    }
  }

  // set base driver status 
  out->status->data = status;

  pleft = left;
  pright = right;

  out->battery->data = 98;

  out->battery->data = 99;
  out->encoder->left = 0;
  out->encoder->right = 0;
  out->actuator->stamp = radl_gettime();
  out->actuator->left = left;
  out->actuator->right = right;
  return 0;
}

