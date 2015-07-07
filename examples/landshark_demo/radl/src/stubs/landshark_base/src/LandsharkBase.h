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

#ifndef __LANDHARK_BASE_H__
#define __LANDHARK_BASE_H__

#include <cstdlib>
#include <cstdio>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include RADL_HEADER

class LandsharkBase {
  public:
    LandsharkBase();
    ~LandsharkBase();

    /**
     * @brief step function 
     */

    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );


  private:
    // transform from twist to motor inputs
    void to_left_right( const double t, const double a, double& left, double& right );

    bool volatile m_IsCommunicationRunning;
    boost::mutex m_RunningMutex;

    // Internal variables
    boost::thread_group m_ThreadGroup;
    unsigned int volatile m_DesiredProcessPeriod_us; // us
    unsigned int volatile m_CommandUpdateRate; // nbr of process cycles
    unsigned int volatile m_LedBlinkingRate; // nbr of process cycles
    unsigned int volatile m_AdcUpdateRate; // nbr of process cycles
    unsigned int volatile m_MotorTimeOutRate; // nbr of process cycles
    bool volatile m_ProcessVelocityCommand;
    bool volatile m_ProcessPwmCommand;
    bool m_IsFirstEncoderReading;
    std::vector< std::pair<bool, unsigned int> > m_PwmCommand;
    std::vector<int> m_AdcReadings;
    boost::mutex m_EncodersMutex;
    boost::mutex m_PwmCommandMutex;
    boost::mutex m_VelocityCommandMutex;
    boost::mutex m_AdcReadingMutex;
    unsigned long m_USB4DeviceSerialNumber;
    unsigned long m_SerialNumber;

    // Conversion from twist to motor input
    const double m_WheelBase;
    const double m_MaxMotorSpeed;
    const double m_MaxMotorInput;
    const double m_MotorRatio;

    std::deque<double> m_BatteryLevel;
    const size_t m_BatteryLevelArraySize;

    // variables for step function
    bool estop;
    double pleft;
    double pright;
    double dleft;
    double dright;

  private:
    // error definitions
    static const int SYSTEM_ERROR = -1;
    static const int SERIAL_READ_ERROR = -3;
    static const int LANDSHARK_SUCCESS = 1;
    // process period and rates
    static const unsigned int COMMAND_UPDATE_RATE = 2; // nbr of process cycles
    static const unsigned int LED_BLINKING_RATE = 50; // nbr of process cycles
    static const unsigned int ADC_UPDATE_RATE = 10; // nbr of process cycles
    static const unsigned int MOTOR_TIME_OUT_RATE = 25; // nbr of process cycles
    // default values
    static const int BASE_DEFAULT_PWM_CHANNEL = 0;
    static const int BASE_DEFAULT_PWM = 1500;
    static const size_t ENCODER_NBR = 4;
    static const size_t PWM_CHANNEL_NBR = 8;
    static const size_t ADC_CHANNEL_NBR = 15;
    static const int DEFAULT_MOTOR_TIMEOUT = 2; // s
    // static const int LED_OFF = 0x7F;
    static const int LED_OFF = 0x00;
    static const int LED_ON = 0x80;
    // indexes
    static const size_t BATTERY_ADC_IDX = 4;
    static const size_t LEFT_WHEEL_ENCODER_IDX = 3;
    static const size_t RIGHT_WHEEL_ENCODER_IDX = 2;
};

#endif // __LANDHARK_BASE_H__
