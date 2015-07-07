/*
 * Copyright (C) 2014, SRI International (R)
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

#include <stdlib.h>

#include <stdexcept>
#include <iostream>
#include <sstream>

#include <boost/bind.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "LandsharkGps.h"
#include RADL_HEADER

#define THROW_RUNTIME_ERROR(args) \
{ \
  std::stringstream ss; \
  ss << args; \
  throw std::runtime_error( ss.str() ); \
}

static const long READ_TIMEOUT = 2000; // [ms]

static const std::string GSA_HEADER("GSA");
static const std::string RMC_HEADER("RMC");
static const std::string GGA_HEADER("GGA");

static const size_t GSA_SENTENCE_SIZE = 18;
static const size_t RMC_SENTENCE_SIZE = 13;
static const size_t GGA_SENTENCE_SIZE = 13;

static const std::string GSA_MANUAL_MODE_STRING("M");
static const std::string GSA_AUTOMATIC_MODE_STRING("A");
static const std::string RMC_VALID_POSITION_STRING("A");
static const std::string RMC_NAV_RECEIVER_WARNING_STRING("V");
static const std::string RMC_AUTONOMOUS_MODE_STRING("A");
static const std::string RMC_DIFFERENTIAL_MODE_STRING("D");
static const std::string RMC_ESTIMATED_MODE_STRING("E");
static const std::string RMC_DATA_NOT_VALID_MODE_STRING("N");

static const uint8_t MIN_PRN_NUMBER = 1;
static const uint8_t MAX_PRN_NUMBER = 32;

static const double MIN_DILUTION = 0.5;
static const double MAX_DILUTION = 99.9;
static const double MIN_SPEED = 0.0;
static const double MAX_SPEED = 999.9;
static const double MIN_COURSE = 0.0;
static const double MAX_COURSE = 359.9;
static const double MIN_MAGNETIC_VARIATION = 0.0;
static const double MAX_MAGNETIC_VARIATION = 180.0;
static const double MIN_ANTENNA_ELEVATION = -9999.9;
static const double MAX_ANTENNA_ELEVATION = 99999.9;
static const double MIN_GEOIDAL_HEIGHT = -9999.9;
static const double MAX_GEOIDAL_HEIGHT = 9999.9;

static const int MIN_USED_SATELLITES = 0;
static const int MAX_USED_SATELLITES = 12;

static const char CARRIAGE_RETURN = 0x0D;

inline void CheckNmeaSentenceMinSize(const NmeaSentence& rNmeaSentence, size_t size)
{
  if (rNmeaSentence.size() < size)
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Invalid NMEA sentence. Contains: " << rNmeaSentence.size() << ", "  << size << " expected.");
  }
}

inline void CheckNmeaSentenceHeader(const NmeaSentence& rNmeaSentence, const std::string& rHeader)
{
  CheckNmeaSentenceMinSize(rNmeaSentence, 1);

  if (rNmeaSentence[0].find(rHeader) == std::string::npos)
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Invalid header in NMEA sentence: " << rNmeaSentence[0] << " expecting: " << rHeader);
  }
}

inline void GetUtcTripleFromString(const std::string& rString, uint32_t triple[3])
{
  if (rString.size() < (3 * 2))
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Invalid UTC string: " << rString << " should be at least 6 characters long");
  }

  for (size_t i = 0; i < 3; ++i)
  {
    triple[i] = static_cast<uint32_t>(atoi(rString.substr(i * 2, 2).c_str()));
  }
}

inline void GetLatitudeFromString(const std::string& rString, double& rLatitude)
{
  if (rString.size() < 9)
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Invalid latitude string: " << rString << " should be at least 9 characters long");
  }

  double degree = atof(rString.substr(0, 2).c_str());
  double minute = atof(rString.substr(2, 7).c_str());

  rLatitude = degree + minute / 60.0;
}

inline void UpdateLatitudeSignFromString(const std::string& rString, double rLatitude)
{
  if (rString == std::string("S"))
  {
    rLatitude = -rLatitude;
  }
  else if (rString != std::string("N"))
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Invalid latitude direction: " << rString << ", should be either N or S");
  }
}

inline void GetLongitudeFromString(const std::string& rString, double& rLongitude)
{
  if (rString.size() < 10)
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Invalid longitude string: " << rString << " should be at least 10 characters long");
  }

  double degree = atof(rString.substr(0, 3).c_str());
  double minute = atof(rString.substr(3, 7).c_str());

  rLongitude = degree + minute / 60.0;
}

inline void UpdateLongitudeSignFromString(const std::string& rString, double rLongitude)
{
  if (rString == std::string("W"))
  {
    rLongitude = -rLongitude;
  }
  else if (rString != std::string("E"))
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Invalid longitude direction: " << rString << ", should be either E or W");
  }
}

inline void GetUint8tFromString(const std::string& rString, uint8_t& rUint8t, const uint8_t& rMin, const uint8_t& rMax, const std::string& rVariableName)
{
  uint8_t result = static_cast<uint8_t>(atoi(rString.c_str()));
  if ( result > rMin && result < rMax )
  {
    rUint8t = result;
  }
  else
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Invalid " << rVariableName << ": " << static_cast<int>(result) << ", should be between " << static_cast<int>(rMin) << " and " << static_cast<int>(rMax));
  }
}

inline void GetDoubleFromString(const std::string& rString, double& rDouble, const double& rMin, const double& rMax, const std::string& rVariableName)
{
  double result = atof(rString.c_str());
  if ( result > rMin && result < rMax )
  {
    rDouble = result;
  }
  else
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Invalid " << rVariableName << ": " << result << ", should be between " << rMin << " and " << rMax);
  }
}

inline void GetDilutionFromString(const std::string& rString, double& rDilution)
{
  GetDoubleFromString(rString, rDilution, 0.0, MAX_DILUTION, std::string("dilution of precision"));
}

inline bool IsCheckSumValid(const std::string& rMessage)
{
  size_t checkSumSeparator = rMessage.find("*");
  size_t carriageReturnSeparator = rMessage.rfind(CARRIAGE_RETURN);

  if ((checkSumSeparator == std::string::npos) ||
      (carriageReturnSeparator == std::string::npos))
  {
    return false;
  }

  std::string data = rMessage.substr(0, checkSumSeparator);
  std::string checkSumString = rMessage.substr(checkSumSeparator + 1, carriageReturnSeparator - checkSumSeparator - 1);

  char checkSum = static_cast<char>(strtol(checkSumString.c_str(), NULL, 16));

  char sum = 0;
  for (size_t i = 1; i < data.size(); ++i)
  {
    sum ^= data[i];
  }

  return (sum == checkSum);
}

NmeaSentenceType GetNmeaSentenceType(const NmeaSentence& rNmeaSentence)
{
  CheckNmeaSentenceMinSize(rNmeaSentence, 1);

  NmeaSentenceType result = NMEA_NOT_IMPLEMENTED_YET;

  if (rNmeaSentence[0].find(GSA_HEADER) != std::string::npos)
  {
    result = NMEA_GSA;
  }
  else if (rNmeaSentence[0].find(RMC_HEADER) != std::string::npos)
  {
    result = NMEA_RMC;
  }
  else if (rNmeaSentence[0].find(GGA_HEADER) != std::string::npos)
  {
    result = NMEA_GGA;
  }

  return result;
}

void Convert(const NmeaSentence& rNmeaSentence, GsaData& rGsaData)
{
  rGsaData.m_Fresh = true;
  CheckNmeaSentenceMinSize(rNmeaSentence, GSA_SENTENCE_SIZE);
  CheckNmeaSentenceHeader(rNmeaSentence, GSA_HEADER);

  if (rNmeaSentence[1] == GSA_MANUAL_MODE_STRING)
  {
    rGsaData.m_Mode = GSA_MANUAL;
  }
  else if (rNmeaSentence[1] == GSA_AUTOMATIC_MODE_STRING)
  {
    rGsaData.m_Mode = GSA_AUTOMATIC;
  }
  else
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Unknown Gsa mode: " << rNmeaSentence[1]);
  }

  if (rNmeaSentence[2] == std::string("1"))
  {
    rGsaData.m_PositioningType = GSA_NOT_AVAILABLE;
  }
  else if (rNmeaSentence[2] == std::string("2"))
  {
    rGsaData.m_PositioningType = GSA_2D;
  }
  else if (rNmeaSentence[2] == std::string("3"))
  {
    rGsaData.m_PositioningType = GSA_3D;
  }
  else
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Unknown Gsa positioning type: " << rNmeaSentence[2]);
  }

  for (size_t i = 0; i < GSA_PRN_COUNT; ++i)
  {
    GetUint8tFromString(rNmeaSentence[3 + i], rGsaData.m_PseudoRandomNoiseNumber[i], 0, MAX_PRN_NUMBER, std::string("prn number"));
  }

  GetDilutionFromString(rNmeaSentence[15], rGsaData.m_PositionDilutionOfPrecision);
  GetDilutionFromString(rNmeaSentence[16], rGsaData.m_HorizontalDilutionOfPrecision);
  GetDilutionFromString(rNmeaSentence[17], rGsaData.m_VerticalDilutionOfPrecision);
}

void Convert(const NmeaSentence& rNmeaSentence, RmcData& rRmcData)
{
  rRmcData.m_Fresh = true;
  CheckNmeaSentenceMinSize(rNmeaSentence, RMC_SENTENCE_SIZE);
  CheckNmeaSentenceHeader(rNmeaSentence, RMC_HEADER);

  uint32_t utcTriple[3];
  GetUtcTripleFromString(rNmeaSentence[1], utcTriple);

  rRmcData.m_UtcTime.m_Hours = utcTriple[0];
  rRmcData.m_UtcTime.m_Minutes = utcTriple[1];
  rRmcData.m_UtcTime.m_Seconds = utcTriple[2];

  GetUtcTripleFromString(rNmeaSentence[9], utcTriple);

  rRmcData.m_UtcDate.m_Year = utcTriple[2];
  rRmcData.m_UtcDate.m_Month = utcTriple[1];
  rRmcData.m_UtcDate.m_Day = utcTriple[3];

  if (rNmeaSentence[2] == RMC_VALID_POSITION_STRING)
  {
    rRmcData.m_Status = RMC_VALID_POSITION;
  }
  else if (rNmeaSentence[2] == RMC_NAV_RECEIVER_WARNING_STRING)
  {
    rRmcData.m_Status = RMC_NAV_RECEIVER_WARNING;
  }
  else
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Unknown Rmc status: " << rNmeaSentence[2]);
  }

  double latitude;

  GetLatitudeFromString(rNmeaSentence[3], latitude);
  UpdateLatitudeSignFromString(rNmeaSentence[4], latitude);

  rRmcData.m_Latitude = latitude;

  double longitude;
  GetLongitudeFromString(rNmeaSentence[5], longitude);
  UpdateLongitudeSignFromString(rNmeaSentence[6], longitude);

  rRmcData.m_Longitude = longitude;

  GetDoubleFromString(rNmeaSentence[7], rRmcData.m_SpeedOverGround, MIN_SPEED, MAX_SPEED, std::string("ground speed"));
  GetDoubleFromString(rNmeaSentence[8], rRmcData.m_CourseOverGround, MIN_COURSE, MAX_COURSE, std::string("course over ground"));

  double magneticVariation;
  GetDoubleFromString(rNmeaSentence[10], magneticVariation, MIN_MAGNETIC_VARIATION, MAX_MAGNETIC_VARIATION, std::string("magnetic variation"));

  if (rNmeaSentence[11] == std::string("E"))
  {
    magneticVariation = -magneticVariation;
  }
  else if (rNmeaSentence[11] != std::string("W"))
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Invalid magnetic variation direction: " << rNmeaSentence[11] << ", should be either E or W");
  }

  rRmcData.m_MagneticVariation = magneticVariation;

  if (rNmeaSentence[12] == RMC_AUTONOMOUS_MODE_STRING)
  {
    rRmcData.m_ModeIndicator = RMC_AUTONOMOUS;
  }
  else if (rNmeaSentence[12] == RMC_DIFFERENTIAL_MODE_STRING)
  {
    rRmcData.m_ModeIndicator = RMC_DIFFERENTIAL;
  }
  else if (rNmeaSentence[12] == RMC_ESTIMATED_MODE_STRING)
  {
    rRmcData.m_ModeIndicator = RMC_ESTIMATED;
  }
  else if (rNmeaSentence[12] == RMC_DATA_NOT_VALID_MODE_STRING)
  {
    rRmcData.m_ModeIndicator = RMC_DATA_NOT_VALID;
  }
  else
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Unknown Rmc mode: " << rNmeaSentence[12]);
  }
}

void Convert(const NmeaSentence& rNmeaSentence, GgaData& rGgaData)
{
  rGgaData.m_Fresh = true;
  CheckNmeaSentenceMinSize(rNmeaSentence, GGA_SENTENCE_SIZE);
  CheckNmeaSentenceHeader(rNmeaSentence, GGA_HEADER);

  uint32_t utcTriple[3];
  GetUtcTripleFromString(rNmeaSentence[1], utcTriple);

  rGgaData.m_UtcTime.m_Hours = utcTriple[0];
  rGgaData.m_UtcTime.m_Minutes = utcTriple[1];
  rGgaData.m_UtcTime.m_Seconds = utcTriple[2];

  double latitude;

  GetLatitudeFromString(rNmeaSentence[2], latitude);
  UpdateLatitudeSignFromString(rNmeaSentence[3], latitude);

  rGgaData.m_Latitude = latitude;

  double longitude;
  GetLongitudeFromString(rNmeaSentence[4], longitude);
  UpdateLongitudeSignFromString(rNmeaSentence[5], longitude);

  rGgaData.m_Longitude = longitude;

  if (rNmeaSentence[6] == std::string("0"))
  {
    rGgaData.m_GpsQuality = GGA_NOT_AVAILABLE;
  }
  else if (rNmeaSentence[6] == std::string("1"))
  {
    rGgaData.m_GpsQuality = GGA_NON_DIFFERENTIAL_GPS;
  }
  else if (rNmeaSentence[6] == std::string("2"))
  {
    rGgaData.m_GpsQuality = GGA_DIFFERENTIAL_GPS;
  }
  else if (rNmeaSentence[6] == std::string("6"))
  {
    rGgaData.m_GpsQuality = GGA_ESTIMATED;
  }
  else
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Unknown Gga gps quality: " << rNmeaSentence[6]);
  }

  int usedSatellites = atoi(rNmeaSentence[7].c_str());
  if (usedSatellites > MIN_USED_SATELLITES && usedSatellites < MAX_USED_SATELLITES)
  {
    rGgaData.m_UsedSatellites = static_cast<uint8_t>(usedSatellites);
  }
  else
  {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Invalid number of use satellites: " << usedSatellites << " should be between " << MIN_USED_SATELLITES << " and " << MAX_USED_SATELLITES);
  }

  GetDilutionFromString(rNmeaSentence[8], rGgaData.m_HorizontalDilutionOfPrecision);

  double antennaElevation;
  GetDoubleFromString(rNmeaSentence[9], antennaElevation, MIN_ANTENNA_ELEVATION, MAX_ANTENNA_ELEVATION, std::string("antenna elevation"));
  rGgaData.m_AntennaElevation = antennaElevation;

  double geoidalHeight;
  GetDoubleFromString(rNmeaSentence[11], geoidalHeight, MIN_GEOIDAL_HEIGHT, MAX_GEOIDAL_HEIGHT, std::string("geoidal height"));
  rGgaData.m_GeoidalHeight = geoidalHeight;
}

LandsharkGps::LandsharkGps()
  : m_IoService()
  , m_SerialPort(m_IoService)
  , m_Timer(m_IoService)
  , m_DesiredProcessPeriod_us(1e5)
  , m_Device( "/dev/gps/0" )
  , m_BaudRate( 38400 )
  , m_IsCommunicationRunning( true )
{
}

void LandsharkGps::Start() {
  std::cout << "[gps] device=" << m_Device;
  std::cout << ", baudrate=" << m_BaudRate;
  std::cout << std::endl;
  m_Timer.expires_at(boost::posix_time::pos_infin);
  CheckDeadline();

  Initialize( m_Device, m_BaudRate );
  SetRunning( true );
  m_Thread = boost::thread( &LandsharkGps::CommunicationProcess, this );
}

LandsharkGps::~LandsharkGps()
{
  SetRunning( false );
  m_Thread.join();
  Close();
}


void LandsharkGps::Initialize( const std::string rPort, const uint32_t baudRate )
{
  boost::system::error_code errorCode;
  m_SerialPort.open(rPort, errorCode);

  if (errorCode != boost::system::errc::success) {
    std::stringstream ss;
    THROW_RUNTIME_ERROR("Unable to open serial port " << rPort << ": " << errorCode.message());
  }

  m_SerialPort.set_option(boost::asio::serial_port_base::baud_rate(baudRate));
  m_SerialPort.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  m_SerialPort.set_option(boost::asio::serial_port_base::character_size(8));
  m_SerialPort.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
  m_SerialPort.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
}

void LandsharkGps::Close()
{
  if (m_SerialPort.is_open()) {
    m_SerialPort.close();
  }
}

bool LandsharkGps::ReadNmeaSentence(NmeaSentence& rNmeaSentence)
{
  rNmeaSentence.clear();

  std::string line;
  ReadLine(line);

  if (!IsCheckSumValid(line))
  {
    std::cout << line << std::endl;
    return false;
  }

  size_t previousSeparatorPosition = -1;
  size_t separatorPosition = line.find(",");
  size_t checkSumSeparator = line.find("*");

  line = line.substr(0, checkSumSeparator);

  while (separatorPosition != std::string::npos)
  {
    rNmeaSentence.push_back(line.substr(previousSeparatorPosition + 1, separatorPosition - previousSeparatorPosition - 1));

    previousSeparatorPosition = separatorPosition;

    separatorPosition = line.find(",", separatorPosition + 1);
  }

  rNmeaSentence.push_back(line.substr(previousSeparatorPosition + 1));

  return true;
}

void LandsharkGps::ReadLine(std::string& rLine)
{
  rLine.clear();

  m_Timer.expires_from_now(boost::posix_time::milliseconds(READ_TIMEOUT));

  boost::system::error_code errorCode = boost::asio::error::would_block;
  size_t size = 0;

  boost::asio::streambuf streamBuffer;
  boost::asio::async_read_until(m_SerialPort, streamBuffer, '\n', boost::bind(&LandsharkGps::Handle, _1, _2, &errorCode, &size));

  do
  {
    m_IoService.run_one();
  }
  while (errorCode == boost::asio::error::would_block);

  if (errorCode == boost::system::errc::success)
  {
    boost::asio::streambuf::const_buffers_type buffer = streamBuffer.data();
    rLine = std::string(boost::asio::buffers_begin(buffer), boost::asio::buffers_begin(buffer) + size);
  }
}

void LandsharkGps::CheckDeadline()
{
  if (m_Timer.expires_at() <= boost::asio::deadline_timer::traits_type::now())
  {
    m_SerialPort.cancel();
    m_Timer.expires_at(boost::posix_time::pos_infin);
  }

  m_Timer.async_wait(boost::bind(&LandsharkGps::CheckDeadline, this));
}

void LandsharkGps::Handle(const boost::system::error_code& rErrorCodeIn, size_t sizeIn, boost::system::error_code* pErrorCodeOut, std::size_t* pSizeOut)
{
  *pErrorCodeOut = rErrorCodeIn;
  *pSizeOut = sizeIn;
}

bool LandsharkGps::CommunicationProcess( ) 
{
  std::cout << "[gps] starting thread" << std::endl;
  //boost::posix_time::time_duration processPeriod;
  //boost::posix_time::ptime startTime = boost::posix_time::microsec_clock::local_time();
  struct radl_timer_state timer;
  radl_duration_t loop_period( m_DesiredProcessPeriod_us * 1000 );
  radl_timer_init( &timer, loop_period );

  while ( GetRunning() ) {
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

    try {
      NmeaSentence nmeaSentence;
      bool result = ReadNmeaSentence(nmeaSentence);

      if (!result) {
        std::cerr << "Received data with an invalid checksum." << std::endl;
        continue;
      }
      
      NmeaSentenceType sentenceType = GetNmeaSentenceType(nmeaSentence);

      boost::mutex::scoped_lock lock( m_DataMutex );
      switch (sentenceType)
      {
        case NMEA_GSA:
          std::cout << "<< GGA" << std::endl;
          Convert(nmeaSentence, m_GsaData);
          break;

        case NMEA_RMC:
          std::cout << "<< RMC" << std::endl;
          Convert(nmeaSentence, m_RmcData);
          break;
        
        case NMEA_GGA:
          std::cout << "<< RMC" << std::endl;
          Convert(nmeaSentence, m_GgaData);
          break;

        default:
          break;
      }
    }
    catch (std::exception& rException) {
      std::cerr << "LandsharkGps exception: " << rException.what() << std::endl;
    }

    boost::posix_time::seconds sleep_time( 0.01 );
    boost::this_thread::sleep( sleep_time );
  }
  std::cout << "[gps] stopping thread" << std::endl;
}


