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

#include "LandsharkMoog.h"
#include <sstream>
#include <exception>
#include <vector>

extern "C"
{
#include "serial.h"
#include <termios.h>
#include <errno.h>
}


// error definitions
static const int BAD_FILE_DESCRIPTOR = -1;
static const int ERROR_GETTING_TERMINAL_FD_PARAMETERS = -1;
static const int ERROR_SETTING_TERMINAL_FD_PARAMETERS = -1;
static const int SUCCESS_INIT_PORT = 0;
static const int CHAR_NOT_FOUND = -1;
// process period and rates
// serial communication parameteres
static const speed_t SERIAL_SPEED = B9600;
static const size_t MAX_BYTES_READ = 2048;
static const int MIN_STATUS_KEEP_ALIVE_BYTES = 32;
// moog quickset control character
static const unsigned char START_TEXT = 0x02;
static const unsigned char END_TEXT = 0x03;
static const unsigned char ACKNOWLEDGE = 0x06;
static const unsigned char NOT_ACKNOWLEDGE = 0x15;
static const char ESC_CHAR = 0x1B;
static const char BIT_7 = 0x80;
static const char BIT_7_UNMASK = 0x7F;
// moog quickset commands
static const unsigned char GET_STATUS_JOG_CMD = 0x31;
static const unsigned char PAN_CLOCKWISE = 0x80;
static const unsigned char TILT_UP = 0x40;
static const unsigned char ZOOM_OUT = 0x01;
static const unsigned char ZOOM_IN_MASK = 0xFE;
static const unsigned char HOME_CMD = 0x35;
// miscellaneous parameters
static const double VELOCITY_LIMIT = 100.0;

#pragma pack(1)

struct JogCmd
{
  unsigned char cmdNum;
  unsigned char cmd;
  unsigned char panJogCmd;
  unsigned char tiltJogCmd;
  unsigned char zoom1JogCmd;
  unsigned char focus1JogCmd;
  unsigned char zoom2JogCmd;
  unsigned char focus2JogCmd;
};

struct SendQuicksetJogCmd
{
  unsigned char stx;
  unsigned char identity;
  JogCmd jogCmd;
  unsigned char lrc;
  unsigned char etx;
};

struct SendQuicksetHomeCmd
{
  unsigned char stx;
  unsigned char identity;
  unsigned char homeCmd;
  unsigned char lrc;
  unsigned char etx;
};

#pragma pack()

unsigned char GetQuicksetChecksum(unsigned char *data, size_t size)
{
  unsigned char checksum = 0;

  for (size_t i = 0; i < size; ++i)
  {
    checksum ^= data[i];
  }

  return checksum;
}

int GetCharIndexInCharArray(char querry, char *array, int length)
{
  for (int i = 0; i < length; ++i)
  {
    if (array[i] == querry)
    {
      return i;
    }
  }

  return CHAR_NOT_FOUND;
}

LandsharkMoog::LandsharkMoog()
  : m_IsCommunicationRunning(false)
  , m_DesiredProcessPeriod_us(1e5)
  , m_ControlMode(IDLE)
  , m_SerialPort( "/dev/ttyS0" )
{
  m_SerialPort = *RADL_THIS->device;
  std::cout << "[moog] device: " << m_SerialPort << std::endl;
  Initialize( m_SerialPort );
  SetRunning( true );
  m_ThreadGroup.create_thread(boost::bind(&LandsharkMoog::CommunicationProcess, this));
}

LandsharkMoog::~LandsharkMoog()
{
  SetRunning( false );
  m_ThreadGroup.join_all();
}

int LandsharkMoog::step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags )
{
  double p = std::min( VELOCITY_LIMIT, std::max( -VELOCITY_LIMIT, 100 * in->pan->data ) );
  double t = std::min( VELOCITY_LIMIT, std::max( -VELOCITY_LIMIT, 100 * in->tilt->data ) );
  double z = std::min( VELOCITY_LIMIT, std::max( -VELOCITY_LIMIT, 100 * in->zoom->data ) );

  if ( radl_is_failing( in_flags->pan ) ) {
    p = 0;
  }

  if ( radl_is_failing( in_flags->tilt ) ) {
    t = 0;
  }

  if ( radl_is_failing( in_flags->zoom ) ) {
    z = 0;
  }

  // set *velocity*
  this->SetPanVelocity( -p );
  this->SetTiltVelocity( t );
  this->SetZoomVelocity( z );

  // get *position*
  out->status_pan->data = -GetPan();
  out->status_tilt->data = GetTilt();
  out->status_zoom->data = GetZoom();
}

void LandsharkMoog::Initialize(const std::string& rPort)
{
  if ( this->GetRunning() )
  {
    std::cout << "Process is already initialized, will return." << std::endl;
    return;
  }

  char port_cstr[rPort.size() + 1];
  strcpy(port_cstr, rPort.c_str());

  m_PortFileDescriptor = open_port(port_cstr);
  if (m_PortFileDescriptor == BAD_FILE_DESCRIPTOR)
  {
    std::stringstream stringStream;
    stringStream << "Unable to open port: " << rPort;
    throw std::runtime_error(stringStream.str());
  }

  if (initport(m_PortFileDescriptor, SERIAL_SPEED) != SUCCESS_INIT_PORT)
  {
    std::stringstream stringStream;
    stringStream  << "Unable to initialize communication on port: " << rPort
      << ", error: " << strerror(errno);
    throw std::runtime_error(stringStream.str());
  }

  // have to disable flow control for quickset
  // get the current options for the port...
  struct termios options;

  if (tcgetattr(m_PortFileDescriptor, &options) == ERROR_GETTING_TERMINAL_FD_PARAMETERS)
  {
    std::stringstream stringStream;
    stringStream  << "Unable to get terminal file descriptor parameters of port: " << rPort
      << ", error: " << strerror(errno);
    throw std::runtime_error(stringStream.str());
  }

  // disable hardware flow control
  options.c_cflag &= ~CRTSCTS;
  // disable software flow control
  options.c_iflag &= ~(IXON | IXOFF | IXANY);

  // Set the new options for the port...
  // TCSAFLUSH -Flush input and output buffers and make the change
  // TCSANOW - Make changes now without waiting for data to complete
  // TCSADRAIN - Wait until everything has been transmitted
  if (tcsetattr(m_PortFileDescriptor, TCSANOW, &options) == ERROR_SETTING_TERMINAL_FD_PARAMETERS)
  {
    std::stringstream stringStream;
    stringStream  << "Unable to set the parameters associated with the terminal of port: " << rPort
      << ", error: " << strerror(errno);
    throw std::runtime_error(stringStream.str());
  }
}

void LandsharkMoog::SetPanVelocity(double panVelocity)
{
  if (panVelocity < -VELOCITY_LIMIT || panVelocity > VELOCITY_LIMIT) {
    std::cout << "Received inavalid velocity command: " << panVelocity
      << " should be between: " << -VELOCITY_LIMIT << " and " << VELOCITY_LIMIT << std::endl;

    return;
  }

  boost::mutex::scoped_lock lock(m_DataMutex);
  m_MoogPtzDesiredVelocity.pan = panVelocity;
  SetMode( VELOCITY );
}

void LandsharkMoog::SetTiltVelocity(double tiltVelocity)
{
  if (tiltVelocity < -VELOCITY_LIMIT || tiltVelocity > VELOCITY_LIMIT) {
    std::cout << "Received inavalid velocity command: " << tiltVelocity
      << " should be between: " << -VELOCITY_LIMIT << " and " << VELOCITY_LIMIT << std::endl;
    return;
  }

  boost::mutex::scoped_lock lock(m_DataMutex);
  m_MoogPtzDesiredVelocity.tilt = tiltVelocity;
  SetMode( VELOCITY );
}

void LandsharkMoog::SetZoomVelocity(double zoomVelocity)
{
  if (zoomVelocity < -VELOCITY_LIMIT || zoomVelocity > VELOCITY_LIMIT) {
    std::cout << "Received inavalid velocity command: " << zoomVelocity
      << " should be between: " << -VELOCITY_LIMIT << " and " << VELOCITY_LIMIT << std::endl;
    return;
  }

  boost::mutex::scoped_lock lock(m_DataMutex);
  m_MoogPtzDesiredVelocity.zoom = zoomVelocity;
  SetMode( VELOCITY );
}

double LandsharkMoog::GetPan()
{
  boost::mutex::scoped_lock lock(m_DataMutex);
  return m_MoogPtzMeasuredPosition.pan;
}

double LandsharkMoog::GetTilt()
{
  boost::mutex::scoped_lock lock(m_DataMutex);
  return m_MoogPtzMeasuredPosition.tilt;
}

double LandsharkMoog::GetZoom()
{
  boost::mutex::scoped_lock lock(m_DataMutex);
  return m_MoogPtzMeasuredPosition.zoom;
}

void LandsharkMoog::Home()
{
  SetMode( HOME );
}

void LandsharkMoog::CommunicationProcess()
{
  std::cout << "[moog] starting thread" << std::endl;
  boost::posix_time::time_duration processPeriod;
  boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();

  while ( this->GetRunning() )
  {
    processPeriod = boost::posix_time::microsec_clock::local_time() - startTime;
    unsigned int timeToSleep = m_DesiredProcessPeriod_us - processPeriod.total_microseconds();

    if (timeToSleep > 0) {
      boost::this_thread::sleep(boost::posix_time::microseconds(timeToSleep));
    }

    startTime = boost::posix_time::microsec_clock::local_time();

    switch ( GetMode() )
    {
      case IDLE:
        JogKeepAlive(0, 0, 0);
        {
          boost::mutex::scoped_lock lock(m_DataMutex);
          GetStatus(m_MoogPtzMeasuredPosition.pan, m_MoogPtzMeasuredPosition.tilt, m_MoogPtzMeasuredPosition.zoom);
        }
        //std::cout << "IDLE" << std::endl;
        break;
      case VELOCITY:
        {
          boost::mutex::scoped_lock lock(m_DataMutex);
          JogKeepAlive(m_MoogPtzDesiredVelocity.pan, m_MoogPtzDesiredVelocity.tilt, m_MoogPtzDesiredVelocity.zoom);
          GetStatus(m_MoogPtzMeasuredPosition.pan, m_MoogPtzMeasuredPosition.tilt, m_MoogPtzMeasuredPosition.zoom);
        }
        //std::cout << "VELOCITY" << std::endl;
        break;
      case HOME:
        SendHomeCommand();
        //std::cout << "HOME" << std::endl;
        break;
      default:
        //std::cout << "DEFAULT" << std::endl;
        break;
    }
    SetMode( IDLE );
  } // while

  // Send 0 velocity before shutting down.
  JogKeepAlive(0, 0, 0);
  boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  SetMode( IDLE );
  GetStatus(m_MoogPtzMeasuredPosition.pan, m_MoogPtzMeasuredPosition.tilt, m_MoogPtzMeasuredPosition.zoom);
  std::cout << "[moog] stopping thread" << std::endl;
}

void LandsharkMoog::JogKeepAlive(double panVelocity, double tiltVelocity, double zoomVelocity)
{
  assert(panVelocity >= -VELOCITY_LIMIT && panVelocity <= VELOCITY_LIMIT);
  assert(tiltVelocity >= -VELOCITY_LIMIT && tiltVelocity <= VELOCITY_LIMIT);
  assert(zoomVelocity >= -VELOCITY_LIMIT && zoomVelocity <= VELOCITY_LIMIT);

  int pan = static_cast<int>((panVelocity / VELOCITY_LIMIT) * 255.0);
  int tilt = static_cast<int>((tiltVelocity / VELOCITY_LIMIT) * 255.0);
  int zoom = static_cast<int>((zoomVelocity / VELOCITY_LIMIT) * 127.0);

  //std::cout << "[moog] PTZ = [" << pan << ", " << tilt << ", " << zoom << "]" << std::endl;

  SendQuicksetJogCmd command;

  command.stx = START_TEXT;
  command.identity = 0;

  command.jogCmd.cmdNum = GET_STATUS_JOG_CMD;
  command.jogCmd.cmd = 0;

  if (pan > 0)
  {
    command.jogCmd.cmd = command.jogCmd.cmd | PAN_CLOCKWISE;
  }

  if (tilt > 0)
  {
    command.jogCmd.cmd = command.jogCmd.cmd | TILT_UP;
  }

  command.jogCmd.panJogCmd = abs(pan);
  command.jogCmd.tiltJogCmd = abs(tilt);
  command.jogCmd.zoom1JogCmd = abs(zoom) << 1;

  if (zoom > 0)
  {
    command.jogCmd.zoom1JogCmd = command.jogCmd.zoom1JogCmd | ZOOM_OUT;
  }
  else
  {
    command.jogCmd.zoom1JogCmd = command.jogCmd.zoom1JogCmd & ZOOM_IN_MASK;
  }

  command.jogCmd.zoom2JogCmd = command.jogCmd.zoom1JogCmd;
  command.jogCmd.focus1JogCmd = 0;
  command.jogCmd.focus2JogCmd = 0;

  command.lrc = GetQuicksetChecksum(reinterpret_cast<unsigned char*>(&command.jogCmd), sizeof(JogCmd));
  command.lrc ^= command.identity;
  command.etx = END_TEXT;

  char *pCommand = reinterpret_cast<char*>(&command);

  std::vector<char> rawCommand;
  rawCommand.push_back(pCommand[0]);

  // replace control characters in command (ETX,STX,ACK,NAK,ESC)
  // with a ESC character and then be byte with bit 7 set
  for (size_t i = 1; i < sizeof(SendQuicksetJogCmd) - 1; ++i)
  {
    switch(pCommand[i])
    {
      case ACKNOWLEDGE: case NOT_ACKNOWLEDGE: case ESC_CHAR: case END_TEXT: case START_TEXT:
        rawCommand.push_back(ESC_CHAR);
        rawCommand.push_back(pCommand[i] | BIT_7);
        break;
      default:
        rawCommand.push_back(pCommand[i]);
        break;
    }
  }

  rawCommand.push_back(pCommand[sizeof(SendQuicksetJogCmd) - 1]);

  serialRawWrite(m_PortFileDescriptor, &rawCommand[0], rawCommand.size());
}

void LandsharkMoog::SendHomeCommand()
{
  SendQuicksetHomeCmd command;

  command.stx = START_TEXT;
  command.identity = 0;
  command.homeCmd = HOME_CMD;
  command.lrc = GetQuicksetChecksum(&command.homeCmd, sizeof(unsigned char));
  command.lrc ^= command.identity;
  command.etx = END_TEXT;

  char *pCommand = reinterpret_cast<char*>(&command);

  std::vector<char> rawCommand;
  rawCommand.push_back(pCommand[0]);

  // replace control characters in command (ETX,STX,ACK,NAK,ESC)
  // with a ESC character and then be byte with bit 7 set
  for (size_t i = 1; i < sizeof(SendQuicksetHomeCmd) - 1; ++i)
  {
    switch(pCommand[i])
    {
      case ACKNOWLEDGE: case NOT_ACKNOWLEDGE: case ESC_CHAR: case END_TEXT: case START_TEXT:
        rawCommand.push_back(ESC_CHAR);
        rawCommand.push_back(pCommand[i] | BIT_7);
        break;
      default:
        rawCommand.push_back(pCommand[i]);
        break;
    }
  }

  rawCommand.push_back(pCommand[sizeof(SendQuicksetHomeCmd) - 1]);

  serialRawWrite(m_PortFileDescriptor, &rawCommand[0], rawCommand.size());
}

void LandsharkMoog::GetStatus(double& rPanPosition, double& rTiltPosition, double& rZoom)
{
  boost::posix_time::ptime printTime = boost::posix_time::microsec_clock::local_time();
  char bytesRead[MAX_BYTES_READ];

  int bytesReadCount = readport_variable(m_PortFileDescriptor, bytesRead, MAX_BYTES_READ);

  if (bytesReadCount < MIN_STATUS_KEEP_ALIVE_BYTES) {
    boost::posix_time::time_duration dur = boost::posix_time::microsec_clock::local_time() - printTime;
    if ( dur.total_milliseconds() > 10e3 ) {
      std::cout << "Received a message of " << bytesReadCount << " bytes, expect at least: " << MIN_STATUS_KEEP_ALIVE_BYTES
        << " bytes from a keep alive status." << std::endl;
      printTime = boost::posix_time::microsec_clock::local_time();
    }

    boost::this_thread::sleep(boost::posix_time::microseconds(1e4));
    Reset();

    return;
  }

  int ackIndex = GetCharIndexInCharArray(static_cast<char>(ACKNOWLEDGE), bytesRead, bytesReadCount);
  if (ackIndex == CHAR_NOT_FOUND)
  {
    std::cout << "Could not find acknowledge byte in received message." << std::endl;

    return;
  }

  int etxIndex = GetCharIndexInCharArray(static_cast<char>(END_TEXT), &bytesRead[ackIndex], bytesReadCount - ackIndex);
  if (etxIndex == CHAR_NOT_FOUND)
  {
    std::cout << "Could not find end of text byte in receive message." << std::endl;

    return;
  }

  char status[bytesReadCount];
  size_t statusIndex = 0;

  for (int i = ackIndex; i < etxIndex; ++i)
  {
    switch (bytesRead[i])
    {
      case ESC_CHAR:
        status[statusIndex] = bytesRead[i + 1] & BIT_7_UNMASK;
        ++statusIndex;
        ++i;
        break;
      default:
        status[statusIndex] = bytesRead[i];
        ++statusIndex;
        break;
    }
  }

  int statusLength = statusIndex + 1;
  int dataIndex = 3;

  if (status[2] != static_cast<char>(GET_STATUS_JOG_CMD))
  {
    std::cout << "Expected command byte: " << std::hex << static_cast<int>(GET_STATUS_JOG_CMD) << " but received " << std::hex << static_cast<int>(status[2]) << std::endl;

    return;
  }

  if ((statusLength - dataIndex) < MIN_STATUS_KEEP_ALIVE_BYTES)
  {
    std::cout << "Status of " << statusLength - dataIndex << "bytes, expect at least: " << MIN_STATUS_KEEP_ALIVE_BYTES
      << " bytes from a keep alive status." << std::endl;

    return;
  }

  int panMsb = 0;
  int tiltMsb = 0;

  if (status[dataIndex + 2] & 0x80)
  {
    panMsb = 0xFF;
  }

  if (status[dataIndex + 5] & 0x80)
  {
    tiltMsb = 0xFF;
  }

  int pan = ((panMsb << 24) | static_cast<int>(static_cast<unsigned char>(status[dataIndex + 2])) << 16) | (static_cast<int>(static_cast<unsigned char>(status[dataIndex + 1])) << 8)  | (static_cast<int>(static_cast<unsigned char>(status[dataIndex])));
  int tilt = ((tiltMsb << 24) | static_cast<int>(static_cast<unsigned char>(status[dataIndex + 5])) << 16) | (static_cast<int>(static_cast<unsigned char>(status[dataIndex + 4])) << 8)  | (static_cast<int>(static_cast<unsigned char>(status[dataIndex + 3])));

  rPanPosition = M_PI * static_cast<double>(pan) / 18000.0;
  rTiltPosition = M_PI * static_cast<double>(tilt) / 18000.0;
  rZoom = static_cast<double>(status[dataIndex + 9]);
}

void LandsharkMoog::Reset()
{
  std::vector<char> rawCommand(6);
  rawCommand[0] = START_TEXT;
  rawCommand[1] = 0x0;
  rawCommand[2] = 0x0F;
  rawCommand[3] = 0x00;
  rawCommand[4] = 0 ^ rawCommand[1];
  rawCommand[5] = END_TEXT;

  serialRawWrite(m_PortFileDescriptor, &rawCommand[0], rawCommand.size());
}

