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
 */

#ifndef LANDSHARKMOOG_H
#define LANDSHARKMOOG_H

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include RADL_HEADER

/**
 * This data structure represents the pan, tilt and zoom velocities of the moog quickcset.
 */
struct MoogPtzData
{
  double pan;
  double tilt;
  double zoom;

  MoogPtzData() 
    : pan( 0 )
    , tilt( 0 )
    , zoom( 0 )
  {
  }
};

/**
 * This enum describes the different moog quickset control mode.
 */
enum MoogPtzControlMode
{
  IDLE,
  VELOCITY,
  HOME
};

/**
 * This class provides an interface to control the ptz of the moog quickset.
 */
class LandsharkMoog
{
  public:
    /**
     * Constructor
     */
    LandsharkMoog();

    /**
     * Destructor
     */
    ~LandsharkMoog();

    int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags );

  private:

    /**
     * Initialize the communication with the interface
     * @param rPort specify the serial port
     * @throw std::runtime_error thrown when unable to initialize the communication with the moog ptz
     */
    void Initialize(const std::string& rPort);

    /**
     * Set the camera pan velocity in percent
     * @param panVelocity pan velocity
     */
    void SetPanVelocity(double panVelocity);

    /**
     * Set the camera tilt velocity in percent
     * @param tiltVelocity tilt velocity
     */
    void SetTiltVelocity(double tiltVelocity);

    /**
     * Set the camera zoom velocity in percent
     * @param zoomVelocity zoom velocity
     */
    void SetZoomVelocity(double zoomVelocity);

    /**
     * Set the camera to its home position
     */
    void Home();

    /**
     * Get the pan position in radian
     * @return pan
     */
    double GetPan();

    /**
     * Get the tilt position in radian
     * @return tilt
     */
    double GetTilt();

    /**
     * Get the zoom position in percent
     * @return zoom
     */
    double GetZoom();

  private:
    void CommunicationProcess();
    void JogKeepAlive(double panVelocity, double tiltVelocity, double zoomVelocity);
    void SendHomeCommand();
    void GetStatus(double& rPanPosition, double& rTiltPosition, double& rZoom);
    void Reset();

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

    boost::thread_group m_ThreadGroup;
    boost::mutex m_RunningMutex;
    bool m_IsCommunicationRunning;
    int volatile m_PortFileDescriptor;
    unsigned int volatile m_DesiredProcessPeriod_us; 

    std::string m_SerialPort;

    // Single mutex for both control and reading MeasuredPosition
    boost::mutex m_ControlMutex;
    MoogPtzControlMode m_ControlMode;

    MoogPtzControlMode GetMode() {
      boost::mutex::scoped_lock lock(m_ControlMutex);
      return m_ControlMode;
    }

    void SetMode( MoogPtzControlMode val ) {
      boost::mutex::scoped_lock lock(m_ControlMutex);
      m_ControlMode = val;
    }

    // Single mutex for both control and reading MeasuredPosition
    boost::mutex m_DataMutex;
    MoogPtzData m_MoogPtzDesiredVelocity;
    MoogPtzData m_MoogPtzMeasuredPosition;
};

#endif // LANDSHARKMOOG_H
