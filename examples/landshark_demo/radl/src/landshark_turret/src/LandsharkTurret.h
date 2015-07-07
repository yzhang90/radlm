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
 *
 */

#ifndef LANDSHARKTURRET_H
#define LANDSHARKTURRET_H

#include <string>
#include <vector>
#include <map>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <dmclnx.h>

#include RADL_HEADER

/**
 * This data structure represents the data used to command and read the turret.
 */

const unsigned int TOTAL_CHANNELS( 2 );

struct TurretData
{
  std::map<unsigned int, int> jointStatus;
  std::map<unsigned int, double> measuredJointPositions;
  std::map<unsigned int, double> measuredJointVelocities;
  std::map<unsigned int, double> desiredJointPositions;
  std::map<unsigned int, double> desiredJointVelocities;
  unsigned int jointToHome;
};

/**
 * This enum describes the different control mode.
 */
enum TurretControlMode
{
  IDLE,
  POSITION,
  VELOCITY,
  HOME,
  REHOME,
  RESET,
  STOP
};

/**
 * This class provides an interface to control the turret of the Landshark.
 */
class LandsharkTurret
{
  public:
    /**
     * Constructor
     */
    LandsharkTurret();

    /**
     * Destructor
     */
    ~LandsharkTurret();

    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );

    /**
     * Initialize the communication with the ethernet interface
     * @param rIpAddress IP address of the turret (motion controller: Galil DMC 40)
     * @throw std::runtime_error thrown when unable to initialize the communication with the controller
     */
    void Initialize(const std::string& rIpAddress);

    /**
     * Set the position of the turret joints
     * @param rPositions list of joint positions
     */
    void SetJointPositions(const std::map<unsigned int, double>& rPositions);

    /**
     * Set the velocities of the turret joints
     * @param rVelocities list of joint velocities
     */
    void SetJointVelocities(const std::map<unsigned int, double>& rVelocities);

    /**
     * Set a joint to its home position
     * @param jointIdx specify the joint. If equal to CHANNEL_NBR, all the joints are going to be set to their home positions.
     */
    void SetJointToHomePosition(unsigned int jointIdx);

    /**
     * Rehome the joints.
     */
    void RehomeJoints();

    /**
     * Reset the galil parameters
     */
    void Reset();

    /**
     * Stop any joint motion
     */
    void StopJoints();

    /**
     * Get the position of the turret joints
     * @param rPositions list of joint positions
     */
    void GetJointPositions(std::map<unsigned int, double>& rPositions);

    /**
     * Get the velocities of the turret joints
     * @param rVelocities list of joint velocities
     */
    void GetJointVelocities(std::map<unsigned int, double>& rVelocities);

    /**
     * Shut down the turret
     */
    void ShutDown();

    inline void SetRunning( bool value )
    {
      boost::mutex::scoped_lock lock( m_RunningMutex );
      m_IsCommunicationRunning = value;
    }

    inline bool GetRunning()
    {
      boost::mutex::scoped_lock lock( m_RunningMutex );
      return m_IsCommunicationRunning;
    }

  private:
    void Initialize(CONTROLLERINFO& rControllerInfo);
    void CommunicationProcess();

    bool IsJointDataIdxWithinChannels(const std::map<unsigned int, double>& rJointData);
    bool IsJointDataWithinLimits(const std::map<unsigned int, double>& rJointData, const double* pMin, const double* pMax);

    template<class T> void SetupControlParameters(const std::string& rControlType, T* pParameters);
    bool ProcessSimpleCommand(const std::string& rCommand);
    bool ProcessReadingCommand(const std::string& rCommand, std::vector<int>& rResponse);
    bool ProcessReadingCommand(const std::string& rCommand, std::vector<long>& rResponse);
    bool ProcessReadingCommand(const std::string& rCommand, std::string& rResponse);
    void Idle();

    boost::thread_group m_ThreadGroup;
    unsigned int volatile m_DesiredProcessPeriod_us; // in us
    unsigned int volatile m_EncoderUpdateRate;
    unsigned int volatile m_VelocityReadingRate;
    unsigned int volatile m_JointStatusUpdateRate;

    boost::mutex m_RunningMutex;
    bool m_IsCommunicationRunning;

    // XXX volatile qualifier should be replaced by a mutex
    uint64_t m_Idle;
    bool volatile m_ShouldProcessCommand;
    TurretControlMode volatile m_ControlMode;
    TurretControlMode volatile m_PreviousControlMode;
    HANDLEDMC m_ControllerHandle;
    TurretData m_TurretData;
    boost::mutex m_TurretDataMutex;
    char m_pResponseBuffer[RESPONSE_BUFFER_SIZE];
    std::vector<double> m_TurretJointConstant;
    std::string m_DmcControllerIp;
};

#endif
