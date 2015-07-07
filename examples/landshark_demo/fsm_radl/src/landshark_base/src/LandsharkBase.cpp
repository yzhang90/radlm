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

extern "C"
{
#include "usdigital.h"
}

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
  m_SerialNumber = *RADL_THIS->serial_number;
  m_PwmCommand.assign(PWM_CHANNEL_NBR, std::make_pair(false, 0));
  m_AdcReadings.assign(ADC_CHANNEL_NBR, 0);
  init(); 
}

LandsharkBase::~LandsharkBase()
{
  shutdown();
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
  else if ( radl_is_failing( in_flags->base ) ) {
    left = copysign( std::max( 0.0, fabs( pleft ) - delta ), pleft );
    right = copysign( std::max( 0.0, fabs( pright ) - delta ), pright );
    status = radlast_constants()->BaseStatus->FAILURE;
    std::cout << "[base] RADL failure flags set!" << std::endl;
  }
  else {
    // compute motor input from twist after limiting to allowable range
    double t = in->base->linear.x;
    double a = in->base->angular.z;
    to_left_right( t, a, left, right );
  }

  // set base driver status 
  out->status->data = status;

  pleft = left;
  pright = right;

  // get battery level as a percentage of the capacity
  if ( GetRunning() ) {
    // Set wheel velocity
    SetVelocity( left, right );
    
    // Get battery level
    m_BatteryLevel.push_back( GetBatteryLevel() );
    if ( m_BatteryLevel.size() > m_BatteryLevelArraySize ) {
      m_BatteryLevel.pop_front();
    }
    double battery = 0;
    for ( size_t i = 0; i < m_BatteryLevel.size(); i++ ) {
      battery += m_BatteryLevel[i];
    }
    battery /= static_cast<double>(m_BatteryLevel.size());
    battery = std::max( MIN_BATTERY_LEVEL, std::min( MAX_BATTERY_LEVEL, battery ) ) - MIN_BATTERY_LEVEL;
    battery *= ( 100.0 / MAX_BATTERY_LEVEL );
    out->battery->data = battery;

    // get wheel encoders
    boost::posix_time::ptime time;
    long enc_left;
    long enc_right;
    GetWheelEncoders( enc_left, enc_right, time );
    out->encoder->left = (int64_t) enc_left;
    out->encoder->right = (int64_t) enc_right;
    out->encoder->stamp = radl_from_timespec( ptime_to_timespec( time ) );
  }
  else {
    out->battery->data = 99;
    out->encoder->left = 0;
    out->encoder->right = 0;
  }
  // set last actuator command 
  // XXX this is not the last input to the motors (this may have been capped)
  out->actuator->stamp = radl_gettime();
  out->actuator->left = left;
  out->actuator->right = right;
  return 0;
}

inline int GetDirection(int speed)
{
  if (speed > 0)
  {
    return 1;
  }
  else if (speed < 0)
  {
    return -1;
  }

  return 0;
}

/**
 *  \brief Send the commands to the motor
 */

inline int SetBothMotorSpeeds(int left, int right)
{
  return Set_Both_Motor_Speeds(GetDirection(left), abs(left), GetDirection(right), abs(right));
}

void LandsharkBase::init()
{
  if ( GetRunning() ) {
    std::cerr << "[base] Process is already initialized: doing nothing." << std::endl;
    return;
  }

  // initialize the communication
  char* pDummy;
  if (init_landshark(pDummy, 0) != LANDSHARK_SUCCESS) {
    std::string error("Unable to initialize landshark base: no device?");
    throw std::runtime_error( error );
  }

  // set the speed to zero
  SetBothMotorSpeeds(0, 0);
  Set_Pwm(BASE_DEFAULT_PWM_CHANNEL, BASE_DEFAULT_PWM);

  if (Get_SerialNumber(&m_USB4DeviceSerialNumber) != LANDSHARK_SUCCESS) {
    std::string error( "Failed to retrieve USB4 serial number." );
    throw std::runtime_error( error );
  }
  if ( m_USB4DeviceSerialNumber != m_SerialNumber ) {
    std::cerr << "[base] Device serial number (" << m_USB4DeviceSerialNumber; 
    std::cerr << ") does not match specified serial_number: " << m_SerialNumber << std::endl;
  }

  SetRunning( true );
  // start communication process thread
  m_ThreadGroup.create_thread(boost::bind(&LandsharkBase::CommunicationProcess, this));
}

/**
 * 
 */

inline double nonlin( double val ) {
  const double scale( 12 );
  const double thresh( 11.5 );
  double a = std::min( thresh, std::max( -thresh, scale * val ) );
  return a + val;
}

/**
 * \brief Set the motor input (will cap to maximum allowed velocity)
 */

void LandsharkBase::SetVelocity(double l_wheel, double r_wheel)
{
  static bool first_time( true );
  if ( first_time ) {
    std::cout << "[base] Current will be limited to [" << (int)-MAX_VELOCITY << ", " << (int) MAX_VELOCITY  << "]" << std::endl;
    first_time = false;
  }
//#define __DEBUG__
  l_wheel = nonlin( l_wheel );
  r_wheel = nonlin( r_wheel );

  if ( l_wheel< MIN_VELOCITY || l_wheel > MAX_VELOCITY)
  {
#ifdef __DEBUG__
    std::cerr  << "Received invalid l_wheel velocity: " << l_wheel
      << " should be between " << MIN_VELOCITY << " and " << MAX_VELOCITY << std::endl;
#endif // __DEBUG__
    l_wheel = std::max( MIN_VELOCITY, std::min( MAX_VELOCITY, l_wheel ) );
  }

  if ( r_wheel< MIN_VELOCITY || r_wheel > MAX_VELOCITY)
  {
#ifdef __DEBUG__
    std::cerr  << "Received invalid r_wheel velocity: " << r_wheel
      << " should be between " << MIN_VELOCITY << " and " << MAX_VELOCITY << std::endl;
#endif // __DEBUG__
    r_wheel = std::max( MIN_VELOCITY, std::min( MAX_VELOCITY, r_wheel ) );
  }

#ifdef __DEBUG__
  static int count = 0;
  if ( ++count > 100) {
    std::cout << "wheel command, l_wheel: " << l_wheel << ", r_wheel: " << r_wheel << std::endl;
    count = 0;
  }
#endif // __DEBUG__

  boost::mutex::scoped_lock lock(m_VelocityCommandMutex);

  m_VelocityCommand.left = static_cast<int>(round( l_wheel ));
  m_VelocityCommand.right = static_cast<int>(round( r_wheel ));

  m_ProcessVelocityCommand = true;
}

void LandsharkBase::GetWheelEncoders(long& rLeftWheelEncoder, long& rRightWheelEncoder, boost::posix_time::ptime& rTime)
{
  boost::mutex::scoped_lock lock(m_EncodersMutex);

  rLeftWheelEncoder = m_Encoders.left;
  rRightWheelEncoder = m_Encoders.right;
  rTime = m_Encoders.time;
}

double LandsharkBase::GetBatteryLevel()
{
  boost::mutex::scoped_lock lock(m_AdcReadingMutex);

  assert(m_AdcReadings.size() > (BATTERY_ADC_IDX + 1));

  return static_cast<double>(m_AdcReadings[BATTERY_ADC_IDX]) * BATTERY_VOLTAGE_FACTOR / MAX_BATTERY_ADC_READING;
}

unsigned long LandsharkBase::GetUSB4SerialNumber()
{
  return m_USB4DeviceSerialNumber;
}

void LandsharkBase::shutdown()
{
  SetRunning( false );
  m_ThreadGroup.join_all();

  Stop_all_Motors();
  Set_Pwm(BASE_DEFAULT_PWM_CHANNEL, BASE_DEFAULT_PWM);
  shutdown_landshark();
}

void LandsharkBase::CommunicationProcess()
{
  std::cout << "[base] starting thread" << std::endl;
  int bytesRead;
  unsigned int encoders[ENCODER_NBR];
  int adcValues[ADC_CHANNEL_NBR];
  bool turnLedOn = false;
  int motorTimeOut;
  unsigned int loopCount = 0;
  long timeToSleep;
  boost::posix_time::time_duration processPeriod;
  boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();

  // initialize the encoders
  if (Read_Encoders(encoders) > 0)
  {
    boost::mutex::scoped_lock lock(m_EncodersMutex);

    m_Encoders.previousRawLeft = encoders[LEFT_WHEEL_ENCODER_IDX];
    m_Encoders.previousRawRight = encoders[RIGHT_WHEEL_ENCODER_IDX];
  }
  else
  {
    throw std::string( "Failed to initialize the wheel encoders." );
  }

  while ( GetRunning() )
  {
    processPeriod = boost::posix_time::microsec_clock::local_time() - startTime;
    timeToSleep = m_DesiredProcessPeriod_us - processPeriod.total_microseconds();
    if (timeToSleep > 0) {
      boost::this_thread::sleep(boost::posix_time::microseconds(timeToSleep));
    }
    startTime = boost::posix_time::microsec_clock::local_time();

    // process encoders
    bytesRead = Read_Encoders(encoders);
    if (bytesRead > 0)
    {
      UpdateEncoders(encoders[LEFT_WHEEL_ENCODER_IDX], encoders[RIGHT_WHEEL_ENCODER_IDX], startTime);
    }
    else if (bytesRead == SERIAL_READ_ERROR)
    {
      std::cerr << "Error reading the encoders";
      RestartCommunication();
    }

    // process commands
    if ((loopCount % m_CommandUpdateRate) == 0)
    {
      if (m_ProcessPwmCommand)
      {
        boost::mutex::scoped_lock lock(m_PwmCommandMutex);

        for (size_t i = 0; i < m_PwmCommand.size(); ++i)
        {
          if (m_PwmCommand[i].first)
          {
            Set_Pwm(static_cast<char>(i), static_cast<int>(m_PwmCommand[i].second));
            m_PwmCommand[i].first = false;
          }
        }

        m_ProcessPwmCommand = false;
      } // if (m_ProcessPwmCommand)

      if (m_ProcessVelocityCommand)
      {
        boost::mutex::scoped_lock lock(m_VelocityCommandMutex);

        SetBothMotorSpeeds(m_VelocityCommand.left, m_VelocityCommand.right);
        m_ProcessVelocityCommand = false;
      }
    } // if ((loopCount % m_CommandUpdateRate) == 0)

    // process blinking led
    if ((loopCount % m_LedBlinkingRate) == 0)
    {
      if (turnLedOn)
      {
        Set_High_Power_Output(LED_ON);
        turnLedOn = false;
      }
      else
      {
        Set_High_Power_Output(LED_OFF);
        turnLedOn = true;
      }
    }

    // process adc
    if ((loopCount % m_AdcUpdateRate) == 0)
    {
      bytesRead = Read_ADC_Values(adcValues);
      if (bytesRead > 0)
      {
        boost::mutex::scoped_lock lock(m_AdcReadingMutex);

        m_AdcReadings.assign(adcValues, adcValues + ADC_CHANNEL_NBR);
      }
      else if (bytesRead == SERIAL_READ_ERROR)
      {
        std::cerr << "Error reading adc channels";
        RestartCommunication();
      }
    }

    // process motor timeout
    if ((loopCount % m_MotorTimeOutRate) == 0)
    {
      Read_Timeout(&motorTimeOut);
      if (motorTimeOut <= 1)
      {
        Set_Motor_Timeout(DEFAULT_MOTOR_TIMEOUT);
      }
    }

    ++loopCount;
  } // while ( GetRunning() )
  std::cout << "[base] stopping thread" << std::endl;
}

void LandsharkBase::UpdateEncoders(int leftEncoder, int rightEncoder, boost::posix_time::ptime time)
{
  if (m_IsFirstEncoderReading == true)
  {
    m_Encoders.previousRawLeft = leftEncoder;
    m_Encoders.previousRawRight = rightEncoder;
    m_IsFirstEncoderReading = false;
  }


  boost::mutex::scoped_lock lock(m_EncodersMutex);

  int leftDelta = leftEncoder - m_Encoders.previousRawLeft;
  int rightDelta = rightEncoder - m_Encoders.previousRawRight;

  // From BlackI:
  /* check for encoder counter roll over */
  /* assuming encoder register is 16 bit=65535 */
  /* if encoder jumps more than 32768 then you have roll over */
  /* ie 10 - 65535 = -65515 (should be a +11 change (rollover upward)) */
  /* ie 65535 - 10 = 65525 (should be a -11 change (rollover downward))*/
  /* so subtract 65536 from + numbers and add 65536 to negative numbers */
  /* to get right encoder change value*/
  if (leftDelta > (SIXTEEN_BIT_SIGNED_MAX_NBR + 1))
  {
    leftDelta = leftDelta - SIXTEEN_BIT_REP_NBR;
  }
  if (leftDelta < -(SIXTEEN_BIT_SIGNED_MAX_NBR + 1))
  {
    leftDelta = leftDelta + SIXTEEN_BIT_REP_NBR;
  }
  if (rightDelta > (SIXTEEN_BIT_SIGNED_MAX_NBR + 1))
  {
    rightDelta = rightDelta - SIXTEEN_BIT_REP_NBR;
  }
  if (rightDelta < -(SIXTEEN_BIT_SIGNED_MAX_NBR + 1))
  {
    rightDelta = rightDelta + SIXTEEN_BIT_REP_NBR;
  }

  m_Encoders.left += leftDelta;
  m_Encoders.right += rightDelta;
  m_Encoders.previousRawLeft = leftEncoder;
  m_Encoders.previousRawRight = rightEncoder;
  m_Encoders.time = time;
}

void LandsharkBase::RestartCommunication()
{
  shutdown_landshark();
  boost::this_thread::sleep(boost::posix_time::microseconds(50000));

  char* pDummy;
  init_landshark(pDummy, 1);
}

