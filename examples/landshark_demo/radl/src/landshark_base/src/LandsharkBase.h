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

    /**
     * Initialize the communication with the interface
     * @throw std::runtime_error thrown when unable to initialize the communication with the motors
     */

    void init();

    /**
     * Set the Landshark velocity
     * @param leftWheel left wheel velocity, unit [?]
     * @param rightWheel right wheel velocity, unit [?]
     */
    void SetVelocity(double leftWheel, double rightWheel);

    /**
     * Get wheel encoder values
     * @param rLeftWheelEncoder left wheel encoder value
     * @param rRightWheelEncoder right wheel encoder value
     * @param rTime time of the encoder reading
     */
    void GetWheelEncoders(long& rLeftWheelEncoder, long& rRightWheelEncoder, radl_duration_t& rTime);

    /**
     * Get the battery level in Volts
     * @return battery level
     */
    double GetBatteryLevel();

    /**
     * Get the serial number of the USB4 device
     * @return serial number
     */
    unsigned long GetUSB4SerialNumber();

    /**
     * Shut down the Landshark
     */
    void shutdown();


  private:
    /**
     * This data structure represents encoders.
     */
    struct Encoders
    {
      long left;
      long right;
      int previousRawLeft;
      int previousRawRight;
      radl_duration_t time;
    };

    /**
     * This data structure represents a velocity command.
     */
    struct VelocityCommand
    {
      uint64_t seq;
      radl_duration_t stamp;
      int left;
      int right;

      inline VelocityCommand() 
        : seq( 0 )
        , stamp( 0 )
        , left( 0 )
        , right( 0 )
      {}
    };

    void CommunicationProcess();
    void UpdateEncoders(int leftEncoder, int rightEncoder, const radl_duration_t time);
    void RestartCommunication();

    // transform from twist to motor inputs
    void to_left_right( const double t, const double a, double& left, double& right );

    bool volatile m_IsCommunicationRunning;
    boost::mutex m_RunningMutex;
    inline bool GetRunning( )
    {
      boost::mutex::scoped_lock lock( m_RunningMutex );
      return m_IsCommunicationRunning;
    }

    void SetRunning( bool value ) 
    {
      boost::mutex::scoped_lock lock( m_RunningMutex );
      m_IsCommunicationRunning = value;
    }

    // Internal variables
    boost::thread_group m_ThreadGroup;
    unsigned int volatile m_DesiredProcessPeriod_us; // us
    unsigned int volatile m_CommandUpdateRate; // nbr of process cycles
    unsigned int volatile m_LedBlinkingRate; // nbr of process cycles
    unsigned int volatile m_AdcUpdateRate; // nbr of process cycles
    unsigned int volatile m_MotorTimeOutRate; // nbr of process cycles
    //bool volatile m_ProcessVelocityCommand;
    //bool volatile m_ProcessPwmCommand;
    Encoders m_Encoders;
    bool m_IsFirstEncoderReading;
    std::vector< std::pair<bool, unsigned int> > m_PwmCommand;
    VelocityCommand m_VelocityCommand;
    VelocityCommand m_VelocityStatus;
    std::vector<int> m_AdcReadings;
    uint64_t elapsed_radl_time;
    uint64_t elapsed_boost_time;
    boost::mutex m_EncodersMutex;
    boost::mutex m_PwmCommandMutex;
    boost::mutex m_VelocityCommandMutex;
    boost::mutex m_VelocityStatusMutex;
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
