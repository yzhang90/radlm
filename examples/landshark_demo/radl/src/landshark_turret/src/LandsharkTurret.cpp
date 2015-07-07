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

#include "LandsharkTurret.h"

#include <sstream>
#include <exception>
#include <stdexcept>
#include <stdlib.h>
#include <assert.h>

static const size_t CHANNEL_NBR = 8;
static const unsigned int ACTIVE_CHANNEL_NBR = 2;
static const int FORWARD_LIMIT_SWITCH_SET_BIT = 8;
static const int REVERSE_LIMIT_SWITCH_SET_BIT = 4;
// Time and period constants
static const unsigned long CONTROLLER_TIMEOUT = 1000; // us
static const unsigned long CONTROLLER_DELAY = 5; // us
static const unsigned int ENCODER_UPDATE_RATE = 2; // nbr of process cycles
static const unsigned int VELOCITY_READING_RATE = 2; // nbr of process cycles
static const unsigned int JOINT_STATUS_UPDATE_RATE = 2; // nbr of process cycles
static const long CONTROL_SLEEP = 50000;
static const long SHORT_CONTROL_SLEEP = 1000;
// JOINT CONSTANTS
// %speed to counts/sec conversion
// motors run from base up
// -----------------------
// JOINT 1
// base:
//	cui encoder connected on base = 2048 pul/rev
//      	2048*4(quadrature)=8192 counts/revolution
//	360/8192 =0.0439453125 deg per count
static const double TURRET_JOINT_1_CONSTANT = 8192.0 * (1.0 / (2.0 * M_PI) );
// JOINT 2
// tilt:
//	cui encoder connected on gun axis = 2048 pul/rev
//      	2048*4(quadrature)=8192 counts/revolution
//	30:15 gear ratio for belt connection from encoder to axis
//		16384 counts/rev of axis)
//	360/163848 = 0.02197265625 deg per count
static const double TURRET_JOINT_2_CONSTANT = 16384.0 * (1.0 / (2.0 * M_PI) );
static const double VELOCITY_DIVISOR = 3600.0;
static const double TORQUE_DIVISOR = 100.0;
// Galil DMC controller commands
static const std::string DMC_MODEL_CMD = "\x12\x16";
static const std::string DMC_SERIAL_NBR_CMD = "MG _BN";
static const std::string DMC_HOMEALL_CMD = "XQ #HOMEALL";
static const std::string DMC_STOP_HOMEALL_CMD = "ST;XQ #HOMEALL";
static const std::string DMC_HOME_A_CMD = "ST;XQ #HOME_A";
static const std::string DMC_HOME_B_CMD = "ST;XQ #HOME_B";
static const std::string DMC_POSITION_READING_CMD = "TP";
static const std::string DMC_VELOCITY_READING_CMD = "TV";
static const std::string DMC_STATUS_READING_CMD = "TS";
static const std::string DMC_STOP_MOTION_CMD = "ST";
static const std::string DMC_STOP_ALL_CMD  = "ST;";
static const std::string DMC_EXECUTE_PROGRAM_JTSPD_CMD = "XQ #JTSPD";
static const std::string DMC_EXECUTE_PROGRAM_JTSPD_0_CMD = "XQ #JTSPD_0";
static const std::string DMC_POSITION_ABSOLUTE_CMD = "PA";
static const std::string DMC_BEGIN_MOTION_CMD = "BG ";
static const std::string DMC_BEGIN_MOTION_ON_AB_CMD = "BG AB;";
static const std::string DMC_BEGIN_MOTION_ON_ABCDEFG_CMD = "BG ABCDEFG;";
static const std::string DMC_JOG_SPEED_CMD = "JG";
static const std::string DMC_SH_CMD = "SH";
static const std::string DMC_REHOME_CMD = "ST;XQ #SET_HOM";
static const std::string DMC_RESET_CMD = "ST;RS";
//Control parameter commands
static const std::string DMC_KP_CMD = "KP";
static const std::string DMC_KI_CMD = "KI";
static const std::string DMC_KD_CMD = "KD";
static const std::string DMC_SP_CMD = "SP";
static const std::string DMC_AC_CMD = "AC";
static const std::string DMC_DC_CMD = "DC";
static const std::string DMC_IT_CMD = "IT"; // smoothing function
static const std::string DMC_TRACKING_MODE_CMD = "PT";
static const std::string DMC_DP_CMD = "DP";
static const std::string DMC_SD_CMD = "SD";
static const std::string DMC_TL_CMD = "TL";
static const std::string DMC_TK_CMD = "TK";
static const std::string DMC_LD_CMD = "LD";
static const std::string DMC_ER_CMD = "ER";
static const std::string DMC_OE_CMD = "OE";
// Control parameters
// PID
static const int DMC_KP[] = {70, 150, 0, 0, 0, 0, 0, 0}; // pid proportiona gain {100, 350, 0, 0, 0, 0, 0, 0}
static const double DMC_KI[] = {1, 0, 0, 0, 0, 0, 0, 0};	// pid integral gain
static const int DMC_KD[] = {2895, 3000, 0, 0, 0, 0, 0, 0};	// pid derivative gain
// Max Sped
static const int DMC_SP[] = {2000, 500, 0, 0, 0, 0, 0, 0};	// axis max speed
// Max Acceleration
static const int DMC_AC[] = {1000, 8192, 0,0,0,0,0,0}; // axis acceleration {10000, 8192, 0,0,0,0,0,0} {1000, 8192, 0, 0, 0, 0, 0, 0}
static const int DMC_DC[] = {50000, 8192, 0,0,0,0,0,0};	// axis deceleration {50000, 8192, 0,0,0,0,0,0} {10000, 8192, 0, 0, 0, 0, 0, 0}
// Smoothing Function (Independent Time Constant)
static const double DMC_IT[] = {0.3, 1, 0, 0, 0, 0, 0, 0};
// Control Flages
static const int DMC_PT[] = {1, 1, 0, 0, 0, 0, 0, 0}; // position tracking enabled
static const int DMC_DP[] = {0, 0, 0, 0, 0, 0, 0, 0}; // unknown?
// Limits
static const int DMC_SD[] = {256000,256000,0,0,0,0,0,0}; // limit switch deceleration speed
static const double DMC_TL[] = {9.0, 9.0, 0, 0, 0, 0, 0, 0}; // torque limit
static const double DMC_TK[] = {9.9, 9.0, 0, 0, 0, 0, 0, 0}; // peek torque limit
static const int DMC_LD[] = {0, 0, 0, 0, 0, 0, 0, 0}; // limit switch disable
// Error
static const int DMC_ER[] = {300, 100, 0, 0, 0, 0, 0, 0}; // encoder error limit to shut down motors
static const int DMC_OE[] = {1, 1, 0, 0, 0, 0, 0, 0};	// enable error limiter

// Limit parameters
static const double JOINT_POSITION_MIN[] = {-2 * M_PI, -2 * M_PI, 0, 0, 0, 0, 0, 0};
static const double JOINT_POSITION_MAX[] = {2 * M_PI, 2* M_PI, 0, 0, 0, 0, 0, 0};
static const double JOINT_VELOCITY_MIN[] = {-2 * M_PI, -2 * M_PI, 0, 0, 0, 0, 0, 0};
static const double JOINT_VELOCITY_MAX[] = {2 * M_PI, 2 * M_PI, 0, 0, 0, 0, 0, 0};

LandsharkTurret::LandsharkTurret()
  : m_DesiredProcessPeriod_us(5e4)
  , m_EncoderUpdateRate(ENCODER_UPDATE_RATE)
  , m_VelocityReadingRate(VELOCITY_READING_RATE)
  , m_JointStatusUpdateRate(JOINT_STATUS_UPDATE_RATE)
  , m_IsCommunicationRunning(false)
  , m_ShouldProcessCommand(false)
  , m_Idle( 0 )
  , m_ControlMode(IDLE)
  , m_PreviousControlMode(POSITION)
  , m_ControllerHandle(-1)
  , m_DmcControllerIp( "192.168.15.77" )
{
  m_DmcControllerIp = *RADL_THIS->ip_address;

  std::cout << "[turret] IP addr: " << m_DmcControllerIp << std::endl;
  assert(ACTIVE_CHANNEL_NBR <= CHANNEL_NBR);
  m_TurretData.jointToHome = CHANNEL_NBR;
  for (unsigned int idx = 0; idx < ACTIVE_CHANNEL_NBR; ++idx) {
    m_TurretData.measuredJointPositions[idx] = 0.0;
    m_TurretData.measuredJointVelocities[idx] = 0.0;
    m_TurretData.jointStatus[idx] = 0;
  }

  m_TurretJointConstant.push_back(TURRET_JOINT_1_CONSTANT);
  m_TurretJointConstant.push_back(TURRET_JOINT_2_CONSTANT);

  Initialize( m_DmcControllerIp );

  SetRunning( true );
  m_ThreadGroup.create_thread(boost::bind(&LandsharkTurret::CommunicationProcess, this));
}

LandsharkTurret::~LandsharkTurret()
{
  ShutDown();
}



int LandsharkTurret::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags )
{
  std::map<unsigned int, double> velocities;
  if ( radl_is_failing( in_flags->pan ) || radl_is_failing( in_flags->tilt ) ) {
    velocities[0] = 0;
    velocities[1] = 0;
  }
  else {
    velocities[0] = in->pan->data / 2;
    velocities[1] = in->tilt->data / 2;
  }
  this->SetJointVelocities( velocities );

  std::map<unsigned int, double> positions;
  this->GetJointPositions(positions);
  out->status_pan->data = positions[0] ;
  out->status_tilt->data = positions[1];
}


void LandsharkTurret::Initialize(const std::string& rIpAddress)
{
  if (GetRunning()) {
    std::cout << "Process is already initialized, will return." << std::endl;
    return;
  }

  // turret Controller information structure
  CONTROLLERINFO controllerInfo;

  memset(&controllerInfo, 0, sizeof(controllerInfo));

  controllerInfo.cbSize = sizeof(controllerInfo);
  controllerInfo.usModelID = MODEL_2100;
  controllerInfo.fControllerType = ControllerTypeEthernet;
  controllerInfo.ulTimeout = CONTROLLER_TIMEOUT;
  controllerInfo.ulDelay = CONTROLLER_DELAY;

  strcpy(controllerInfo.hardwareinfo.socketinfo.szIPAddress, rIpAddress.c_str());

  controllerInfo.hardwareinfo.socketinfo.fProtocol = EthernetProtocolTCP;

  Initialize(controllerInfo);
}

void LandsharkTurret::SetJointPositions(const std::map<unsigned int, double>& rPositions)
{
  if (!IsJointDataIdxWithinChannels(rPositions))
  {
    throw std::invalid_argument("Bad joint index");
  }

  if (!IsJointDataWithinLimits(rPositions, JOINT_POSITION_MIN, JOINT_POSITION_MAX))
  {
    std::cout << "Desired joint position out of range" << std::endl;
    return;
  }

  boost::mutex::scoped_lock lock(m_TurretDataMutex);

  m_TurretData.desiredJointPositions = rPositions;

  m_ControlMode = POSITION;

  m_ShouldProcessCommand = true;
}

void LandsharkTurret::SetJointVelocities(const std::map<unsigned int, double>& rVelocities)
{
  if (!IsJointDataIdxWithinChannels(rVelocities))
  {
    throw std::invalid_argument("Bad joint index");
  }

  if (!IsJointDataWithinLimits(rVelocities, JOINT_VELOCITY_MIN, JOINT_VELOCITY_MAX))
  {
    std::cout << "Desired joint velocity out of range" << std::endl;
    return;
  }

  int notZero( 0 );
  //std::cout << "    vel= [ ";
  for ( std::map<unsigned int, double>::const_iterator it = rVelocities.begin(); it != rVelocities.end(); it++ ) {
    //std::cout << "  " << it->second;
    notZero += ( fabs( it->second ) > 0 ) ? 1 : 0;
  }
  m_Idle = notZero == 0 ? m_Idle + 1 : 0;
  //std::cout << "  (m_Idle= " << m_Idle << ", notZero= " << notZero << " )" << std::endl;

  boost::mutex::scoped_lock lock(m_TurretDataMutex);

  m_ControlMode = ( m_Idle > 2 ) ? IDLE : VELOCITY;
  m_TurretData.desiredJointVelocities = rVelocities;
  m_ShouldProcessCommand = true;
}


void LandsharkTurret::SetJointToHomePosition(unsigned int jointIdx)
{
  if (jointIdx > CHANNEL_NBR)
  {
    throw std::invalid_argument("Bad joint index");
  }

  boost::mutex::scoped_lock lock(m_TurretDataMutex);

  if (jointIdx >= ACTIVE_CHANNEL_NBR)
  {
    m_TurretData.jointToHome = CHANNEL_NBR;
  }
  else
  {
    m_TurretData.jointToHome = jointIdx;
  }

  m_ControlMode = HOME;

  m_ShouldProcessCommand = true;
}

void LandsharkTurret::RehomeJoints()
{
  m_ControlMode = REHOME;

  m_ShouldProcessCommand = true;
}

void LandsharkTurret::Reset()
{
  m_ControlMode = RESET;

  m_ShouldProcessCommand = true;
}

void LandsharkTurret::StopJoints()
{
  m_ControlMode = STOP;

  m_ShouldProcessCommand = true;
}

void LandsharkTurret::GetJointPositions(std::map<unsigned int, double>& rPositions)
{
  boost::mutex::scoped_lock lock(m_TurretDataMutex);

  rPositions = m_TurretData.measuredJointPositions;
}

void LandsharkTurret::GetJointVelocities(std::map<unsigned int, double>& rVelocities)
{
  boost::mutex::scoped_lock lock(m_TurretDataMutex);

  rVelocities = m_TurretData.measuredJointVelocities;
}

void LandsharkTurret::ShutDown()
{
  SetRunning( false );

  m_ThreadGroup.join_all();

  long resultDMCCommand = DMCClose(m_ControllerHandle);
  if (resultDMCCommand) {
    std::cerr << "Shutdown Error on DMClose, returned: " << resultDMCCommand << std::endl;
  }
}

void LandsharkTurret::Initialize(CONTROLLERINFO& rControllerInfo)
{
  // turret controller init
  DMCInitLibrary();

  // open the connection
  long resultDMCCommand = DMCOpen(&rControllerInfo, &m_ControllerHandle);
  if (resultDMCCommand)
  {
    std::stringstream stream;
    stream << "Unable to initialize the communication with the controller, open returned: " << resultDMCCommand;

    throw std::runtime_error(stream.str());
  }

  std::string response;
  std::string key( ":" );
  size_t pos;
  // model
  resultDMCCommand = ProcessReadingCommand(DMC_MODEL_CMD, response);
  if ( ( pos = response.rfind( key ) ) != std::string::npos ) {
    response.replace( pos, key.length(), "" );
  }
  std::cout << "[turret] Controller Model: " << response;

  // serial number
  resultDMCCommand = ProcessReadingCommand(DMC_SERIAL_NBR_CMD, response);
  if ( ( pos = response.rfind( key ) ) != std::string::npos ) {
    response.replace( pos, key.length(), "" );
  }
  std::cout << "[turret] Serial Number: " << response;

  // set up the acceleration parameters
  SetupControlParameters(DMC_AC_CMD, DMC_AC);
  SetupControlParameters(DMC_DC_CMD, DMC_DC);

  SetupControlParameters(DMC_KP_CMD, DMC_KP);

  SetupControlParameters(DMC_IT_CMD, DMC_IT);

#ifdef SET_PID_PAR
  // set up pid parameters for motors
  SetupControlParameters(DMC_KP_CMD, DMC_KP);
  SetupControlParameters(DMC_KI_CMD, DMC_KI);
  SetupControlParameters(DMC_KD_CMD, DMC_KD);

  // set up the speed parameters
  SetupControlParameters(DMC_SP_CMD, DMC_SP);

  // set up the acceleration parameters
  SetupControlParameters(DMC_AC_CMD, DMC_AC);
  SetupControlParameters(DMC_DC_CMD, DMC_DC);

  // set up the limits
  SetupControlParameters(DMC_SD_CMD, DMC_SD);
  SetupControlParameters(DMC_TL_CMD, DMC_TL);
  SetupControlParameters(DMC_TK_CMD, DMC_TK);
  SetupControlParameters(DMC_LD_CMD, DMC_LD);

  // set up some flags
  SetupControlParameters(DMC_TRACKING_MODE_CMD, DMC_PT);
  SetupControlParameters(DMC_DP_CMD, DMC_DP);

  // set up error parameters Er_turret
  SetupControlParameters(DMC_ER_CMD, DMC_ER);
  SetupControlParameters(DMC_OE_CMD, DMC_OE);
#endif

#ifdef MEEP2
  resultDMCCommand = DMCCommand(m_ControllerHandle, DMC_HOMEALL_CMD, m_pResponseBuffer, sizeof(m_pResponseBuffer));
  if (resultDMCCommand)
  {
    std::cerr << "Unable to set " << "XQ #HOMEALL" << " parameters of the controller, returned: " << resultDMCCommand << std::endl;
  }
#endif
}

void LandsharkTurret::CommunicationProcess()
{
  std::cout << "[turret] starting thread" << std::endl;
  assert(m_TurretJointConstant.size() >= m_TurretData.measuredJointPositions.size());

  unsigned int loopCount = 0;
  //boost::posix_time::time_duration processPeriod;
  //boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();
  struct radl_timer_state timer;
  radl_duration_t loop_period( m_DesiredProcessPeriod_us * 1000 );
  radl_timer_init( &timer, loop_period );


  while (GetRunning())
  {
#if 1
    radl_timer_wait( &timer );
#else 
    processPeriod = boost::posix_time::microsec_clock::local_time() - startTime;
    long timeToSleep = m_DesiredProcessPeriod_us - processPeriod.total_microseconds();
    if (timeToSleep > 0) {
      boost::this_thread::sleep(boost::posix_time::microseconds(timeToSleep));
    }
    startTime = boost::posix_time::microsec_clock::local_time();
#endif 

    // encoders
    if ((loopCount % m_EncoderUpdateRate) == 0)
    {
      std::vector<long> encoders;

      if (ProcessReadingCommand(DMC_POSITION_READING_CMD, encoders))
      {
        boost::mutex::scoped_lock lock(m_TurretDataMutex);
        for (unsigned int i = 0; i < ACTIVE_CHANNEL_NBR; ++i)
        {
          m_TurretData.measuredJointPositions[i] = static_cast<double>(encoders[i]) / m_TurretJointConstant[i];
        }
      }
    }

    // velocity
    if ((loopCount % m_VelocityReadingRate) == 0)
    {
      std::vector<long> velocities;

      if (ProcessReadingCommand(DMC_VELOCITY_READING_CMD, velocities))
      {
        boost::mutex::scoped_lock lock(m_TurretDataMutex);
        for (unsigned int i = 0; i < ACTIVE_CHANNEL_NBR; ++i)
        {
          m_TurretData.measuredJointVelocities[i] = 2 * M_PI * static_cast<double>(velocities[i]) / VELOCITY_DIVISOR;
        }
      }
    }

    // joint status
    if ((loopCount % m_JointStatusUpdateRate) == 0)
    {
      std::vector<int> status;

      if (ProcessReadingCommand(DMC_STATUS_READING_CMD, status))
      {
        boost::mutex::scoped_lock lock(m_TurretDataMutex);
        for (unsigned int i = 0; i < ACTIVE_CHANNEL_NBR; ++i)
        {
          m_TurretData.jointStatus[i] = status[i];
        }
      }
    }

    m_ShouldProcessCommand = false;

    // joint position, velocity and torque control ...
    switch (m_ControlMode)
    {
      case IDLE:
        {
          break;
        }
      case POSITION:
        {
          if (m_PreviousControlMode == VELOCITY)
          {
            boost::this_thread::sleep(boost::posix_time::microseconds(CONTROL_SLEEP));
            ProcessSimpleCommand(DMC_STOP_MOTION_CMD);
            boost::this_thread::sleep(boost::posix_time::microseconds(CONTROL_SLEEP));
            ProcessSimpleCommand(DMC_EXECUTE_PROGRAM_JTSPD_CMD);
          }

          // put into position tracking mode for position absolute moves
          SetupControlParameters(DMC_TRACKING_MODE_CMD, DMC_PT);
          boost::this_thread::sleep(boost::posix_time::microseconds(SHORT_CONTROL_SLEEP));

          std::stringstream positionCommand;
          positionCommand << DMC_POSITION_ABSOLUTE_CMD << " ";
          {
            boost::mutex::scoped_lock lock(m_TurretDataMutex);

            for (unsigned int i = 0; i < ACTIVE_CHANNEL_NBR; ++i)
            {
              std::map<unsigned int, double>::iterator it = m_TurretData.desiredJointPositions.find(i);

              if (it != m_TurretData.desiredJointPositions.end())
              {
                positionCommand << m_TurretJointConstant[it->first] * it->second;
              }
              else
              {
                positionCommand << ", ";
              }
            }
          }

          ProcessSimpleCommand(positionCommand.str());
          boost::this_thread::sleep(boost::posix_time::microseconds(SHORT_CONTROL_SLEEP));

          // ProcessSimpleCommand(DMC_BEGIN_MOTION_ON_AB_CMD);

          m_PreviousControlMode = POSITION;

          Idle();

          break;
        }
      case VELOCITY:
        {
          if (m_PreviousControlMode == POSITION)
          {
            boost::this_thread::sleep(boost::posix_time::microseconds(CONTROL_SLEEP));
            ProcessSimpleCommand(DMC_STOP_MOTION_CMD);
            boost::this_thread::sleep(boost::posix_time::microseconds(CONTROL_SLEEP));
            ProcessSimpleCommand(DMC_EXECUTE_PROGRAM_JTSPD_0_CMD);

            int axis[] = {0, 0, 0, 0, 0, 0, 0, 0};

            {
              boost::mutex::scoped_lock lock(m_TurretDataMutex);
              for (unsigned int i = 0; i < ACTIVE_CHANNEL_NBR; ++i)
              {
                if ((m_TurretData.jointStatus[i] & FORWARD_LIMIT_SWITCH_SET_BIT) == 0)
                {
                  axis[i] = -1;
                }
                if ((m_TurretData.jointStatus[i] & REVERSE_LIMIT_SWITCH_SET_BIT) == 0)
                {
                  axis[i] = 1;
                }
              }
            }

            SetupControlParameters(DMC_JOG_SPEED_CMD, axis);
            boost::this_thread::sleep(boost::posix_time::microseconds(CONTROL_SLEEP));
            ProcessSimpleCommand(DMC_BEGIN_MOTION_ON_AB_CMD);
            boost::this_thread::sleep(boost::posix_time::microseconds(CONTROL_SLEEP));

            int zeros[] = {0, 0, 0, 0, 0, 0, 0, 0};

            SetupControlParameters(DMC_JOG_SPEED_CMD, zeros);
          }

          int positionTracking[] = {0, 0, 0, 0, 0, 0, 0, 0};

          SetupControlParameters(DMC_TRACKING_MODE_CMD, positionTracking);
          ProcessSimpleCommand(DMC_SH_CMD);

          int velocities[] = {0, 0, 0, 0, 0, 0, 0, 0};
          {
            boost::mutex::scoped_lock lock(m_TurretDataMutex);
            for (std::map<unsigned int, double>::iterator it = m_TurretData.desiredJointVelocities.begin(); it != m_TurretData.desiredJointVelocities.end(); ++it)
            {
              velocities[it->first] = static_cast<int>(m_TurretJointConstant[it->first] * it->second);
            }
          }

          SetupControlParameters(DMC_JOG_SPEED_CMD, velocities);

          std::string command(DMC_BEGIN_MOTION_ON_ABCDEFG_CMD);
#define test22
#ifdef test22
          std::stringstream commandStream;
          commandStream << DMC_BEGIN_MOTION_CMD;

          {
            boost::mutex::scoped_lock lock(m_TurretDataMutex);
            for (unsigned int i = 0; i < ACTIVE_CHANNEL_NBR; ++i)
            {
              if ( (((m_TurretData.jointStatus[i] & FORWARD_LIMIT_SWITCH_SET_BIT) == 0)) && (velocities[i] >= 0) )
              {}
              else
              {
                commandStream << static_cast<char>('A' + i);
              }
            }
          }

          commandStream << ';';

          command = commandStream.str();
#endif

          ProcessSimpleCommand(command.c_str());

          m_PreviousControlMode = VELOCITY;

          Idle();

          break;
        }
      case HOME:
        {
          ProcessSimpleCommand(DMC_STOP_MOTION_CMD);
          boost::this_thread::sleep(boost::posix_time::microseconds(CONTROL_SLEEP));

          std::string command("");

          {
            boost::mutex::scoped_lock lock(m_TurretDataMutex);

            switch (m_TurretData.jointToHome)
            {
              case 0:
                {
                  command = DMC_HOME_A_CMD;
                  break;
                }
              case 1:
                {
                  command = DMC_HOME_B_CMD;
                  break;
                }
              case CHANNEL_NBR:
                {
                  command = DMC_STOP_HOMEALL_CMD;
                  break;
                }
              default:
                {
                  break;
                }
            }
          }

          if (command != "")
          {
            ProcessSimpleCommand(command);
          }

          m_PreviousControlMode = POSITION;

          Idle();

          break;
        }
      case REHOME:
        {
          ProcessSimpleCommand(DMC_REHOME_CMD);

          Idle();

          break;
        }
      case RESET:
        {
          ProcessSimpleCommand(DMC_RESET_CMD);

          Idle();

          break;
        }
      case STOP:
        {
          ProcessSimpleCommand(DMC_STOP_ALL_CMD);

          Idle();

          break;
        }
      default:
        {
          Idle();

          break;
        }
    }

    ++loopCount;
  } // while (GetRunning())

  ProcessSimpleCommand(DMC_STOP_ALL_CMD);

  std::cout << "[turret] stopping thread" << std::endl;
} // CommunicationProcess

bool LandsharkTurret::IsJointDataIdxWithinChannels(const std::map<unsigned int, double>& rJointData)
{
  for (std::map<unsigned int, double>::const_iterator it = rJointData.begin(); it != rJointData.end(); it++)
  {
    if (it->first >= CHANNEL_NBR)
    {
      return false;
    }
  }

  return true;
}

bool LandsharkTurret::IsJointDataWithinLimits(const std::map<unsigned int, double>& rJointData, const double* pMin, const double* pMax)
{
  assert(IsJointDataIdxWithinChannels(rJointData));

  for (std::map<unsigned int, double>::const_iterator it = rJointData.begin(); it != rJointData.end(); it++)
  {
    unsigned jointIndex = it->first;
    double jointValue = it->second;

    if (!(jointValue > pMin[jointIndex] && jointValue < pMax[jointIndex])) {
      return false;
    }
  }

  return true;
}

template<class T>
void LandsharkTurret::SetupControlParameters(const std::string& rControlType, T* pParameters)
{
  std::stringstream commandStream;
  commandStream << rControlType << " "  << pParameters[0] << "," << pParameters[1] << "," << pParameters[2] << "," << pParameters[3] << ","
    << pParameters[4] << "," << pParameters[5] << "," << pParameters[6] << "," << pParameters[7];

  char command[commandStream.str().size() + 1];

  strcpy(command, commandStream.str().c_str());

  long resultDMCCommand = DMCCommand(m_ControllerHandle, command, m_pResponseBuffer, sizeof(m_pResponseBuffer));
  if (resultDMCCommand)
  {
    std::cerr << "Unable to process " << commandStream.str() << ", returned: " << resultDMCCommand << std::endl;

    DMCError(m_ControllerHandle, resultDMCCommand, m_pResponseBuffer, sizeof(m_pResponseBuffer));

    std::cerr << "Message: " << m_pResponseBuffer << std::endl;
  }
}

bool LandsharkTurret::ProcessSimpleCommand(const std::string& rCommand)
{
  char command[rCommand.size() + 1];

  strcpy(command, rCommand.c_str());

  long resultDMCCommand = DMCCommand(m_ControllerHandle, command, m_pResponseBuffer, sizeof(m_pResponseBuffer));
  if (resultDMCCommand)
  {
    std::cerr << "Unable to process command " << rCommand << ", returned: " << resultDMCCommand << std::endl;

    DMCError(m_ControllerHandle, resultDMCCommand, m_pResponseBuffer, sizeof(m_pResponseBuffer));

    std::cerr << "Message: " << m_pResponseBuffer << std::endl;

    return false;
  }

  return true;
}

bool LandsharkTurret::ProcessReadingCommand(const std::string& rCommand, std::vector<int>& rResponse)
{
  char command[rCommand.size() + 1];

  strcpy(command, rCommand.c_str());

  long resultDMCCommand = DMCCommand(m_ControllerHandle, command, m_pResponseBuffer, sizeof(m_pResponseBuffer));

  if (resultDMCCommand)
  {
    std::cerr << "Unable to process command: " << rCommand << ", returned: " << resultDMCCommand << std::endl;

    DMCError(m_ControllerHandle, resultDMCCommand, m_pResponseBuffer, sizeof(m_pResponseBuffer));

    std::cerr << "Message: " << m_pResponseBuffer << std::endl;

    return false;
  }

  rResponse.resize(CHANNEL_NBR);

  sscanf(m_pResponseBuffer, "%d,%d,%d,%d,%d,%d,%d,%d\n:", &rResponse[0], &rResponse[1], &rResponse[2], &rResponse[3], &rResponse[4], &rResponse[5], &rResponse[6], &rResponse[7]);

  return true;
}

bool LandsharkTurret::ProcessReadingCommand(const std::string& rCommand, std::vector<long>& rResponse)
{
  char command[rCommand.size() + 1];

  strcpy(command, rCommand.c_str());

  long resultDMCCommand = DMCCommand(m_ControllerHandle, command, m_pResponseBuffer, sizeof(m_pResponseBuffer));

  if (resultDMCCommand)
  {
    std::cerr << "Unable to process command: " << rCommand << ", returned: " << resultDMCCommand << std::endl;

    DMCError(m_ControllerHandle, resultDMCCommand, m_pResponseBuffer, sizeof(m_pResponseBuffer));

    std::cerr << "Message: " << m_pResponseBuffer << std::endl;

    return false;
  }

  rResponse.resize(CHANNEL_NBR);

  sscanf(m_pResponseBuffer, "%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\n:", &rResponse[0], &rResponse[1], &rResponse[2], &rResponse[3], &rResponse[4], &rResponse[5], &rResponse[6], &rResponse[7]);

  return true;
}

bool LandsharkTurret::ProcessReadingCommand(const std::string& rCommand, std::string& rResponse)
{
  char command[rCommand.size() + 1];

  strcpy(command, rCommand.c_str());

  long resultDMCCommand = DMCCommand(m_ControllerHandle, command, m_pResponseBuffer, sizeof(m_pResponseBuffer));

  if (resultDMCCommand)
  {
    std::cerr << "Unable to process command: " << rCommand << ", returned: " << resultDMCCommand << std::endl;

    DMCError(m_ControllerHandle, resultDMCCommand, m_pResponseBuffer, sizeof(m_pResponseBuffer));

    std::cerr << "Message: " << m_pResponseBuffer << std::endl;

    return false;
  }

  rResponse = std::string(m_pResponseBuffer);

  return true;
}

void LandsharkTurret::Idle()
{
  if (!m_ShouldProcessCommand)
  {
    m_ControlMode = IDLE;
  }
}
