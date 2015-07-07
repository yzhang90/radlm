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

#ifndef LANDSHARKGPS_H
#define LANDSHARKGPS_H

#include <string>
#include <vector>
#include <iostream>

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

static const double KNOTS_TO_METER_PER_S = 0.5144444444;
static const double DEGREE_TO_RADIAN = 0.0174532925;


typedef std::vector<std::string> NmeaSentence;

/**
 * Define NMEA sentence type
 */
enum NmeaSentenceType
{
  NMEA_NOT_IMPLEMENTED_YET,
  NMEA_GSA,
  NMEA_RMC,
  NMEA_GGA
};

/**
 * NMEA sentence type
 * @param rNmeaSentence NMEA sentence
 * @result NMEA sentence type
 */
NmeaSentenceType GetNmeaSentenceType(const NmeaSentence& rNmeaSentence);

/**
 * Define Gsa mode
 */
enum GsaMode
{
  GSA_MANUAL,
  GSA_AUTOMATIC
};

/**
 * Define Gsa positioning type
 */
enum GsaPositioningType
{
  GSA_NOT_AVAILABLE,
  GSA_2D,
  GSA_3D
};

static const size_t GSA_PRN_COUNT = 12; 

/**
 * This structure represents GPS DOP and Active Satellites data
 */
struct GsaData
{
  bool m_Fresh;
  GsaMode m_Mode;
  GsaPositioningType m_PositioningType;
  uint8_t m_PseudoRandomNoiseNumber[GSA_PRN_COUNT];
  double m_PositionDilutionOfPrecision;
  double m_HorizontalDilutionOfPrecision;
  double m_VerticalDilutionOfPrecision;

  GsaData() 
    : m_Fresh( false )
  {
  }
};

/**
 * Convert a NMEA sentence to GSA data
 * @param rNmeaSentence NMEA sentence
 * @param rGsaData GSA data
 */
void Convert(const NmeaSentence& rNmeaSentence, GsaData& rGsaData);

/**
 * This structure represents the UTC time of the GPS
 */
struct GpsUtcTime
{
  uint32_t m_Hours;
  uint32_t m_Minutes;
  uint32_t m_Seconds;
};

/**
 * This structure represents the UTC date of the GPS
 */
struct GpsUtcDate
{
  uint32_t m_Year;
  uint32_t m_Month;
  uint32_t m_Day;
};

/**
 * Define Rmc status
 */
enum RmcStatus
{
  RMC_VALID_POSITION,
  RMC_NAV_RECEIVER_WARNING
};

/**
 * Define Rmc mode indicator
 */
enum RmcModeIndicator
{
  RMC_AUTONOMOUS,
  RMC_DIFFERENTIAL,
  RMC_ESTIMATED,
  RMC_DATA_NOT_VALID
};

/**
 * This structure represents Recommended Minimum Specific GPS/TRANSIT data
 */
struct RmcData
{
  bool m_Fresh;
  GpsUtcTime m_UtcTime;
  GpsUtcDate m_UtcDate;
  RmcStatus m_Status;
  double m_Latitude; // [deg]
  double m_Longitude; // [deg]
  double m_SpeedOverGround; // [knots]
  double m_CourseOverGround; // [deg]
  double m_MagneticVariation; // [deg]
  RmcModeIndicator m_ModeIndicator;

  RmcData() 
    : m_Fresh( false )
  {
  }
};

/**
 * Convert a NMEA sentence to RMC data
 * @param rNmeaSentence NMEA sentence
 * @param rRmcData RMC data
 */
void Convert(const NmeaSentence& rNmeaSentence, RmcData& rRmcData);

/**
 * Define GGA Gps quality
 */
enum GgaGpsQuality
{
  GGA_NOT_AVAILABLE,
  GGA_NON_DIFFERENTIAL_GPS,
  GGA_DIFFERENTIAL_GPS,
  GGA_ESTIMATED
};

/**
 * This structure represents Global Positioning System Fix Data
 */

struct GgaData
{
  bool m_Fresh;
  GpsUtcTime m_UtcTime;
  double m_Latitude; // [deg]
  double m_Longitude; // [deg]
  GgaGpsQuality m_GpsQuality;
  uint8_t m_UsedSatellites;
  double m_HorizontalDilutionOfPrecision;
  double m_AntennaElevation; // [m]
  double m_GeoidalHeight; // [m]

  GgaData() 
    : m_Fresh( false )
  {
  }
};

/**
 * Convert a NMEA sentence to GGA data
 * @param rNmeaSentence NMEA sentence
 * @param rGgaData GGA data
 */
void Convert(const NmeaSentence& rNmeaSentence, GgaData& rGgaData);

/**
 * This class represents an interface to Gps that uses NMEA sentences.
 */
class LandsharkGps
{
  public:
    /**
     * Constructor
     */
    LandsharkGps();

    /**
     * Destructor
     */
    ~LandsharkGps();

    /**
     * Initialize the communication
     * @param rPort gps serial port
     * @param baudRate baud rate
     */
    void Initialize( const std::string rPort, const uint32_t baudRate );

    /**
     * Close the device
     */
    void Close();

    /**
     * Read NMEA sentence
     * @param rNmeaSentence nmea sentence
     * @result wheter the sentence was properly read
     */
    bool ReadNmeaSentence(NmeaSentence& rNmeaSentence);

  protected:
    void Start();

    inline void SetRunning( const bool value ) 
    {
      boost::mutex::scoped_lock lock( m_RunningMutex );
      m_IsCommunicationRunning = value;
    }

    inline bool GetRunning( ) 
    {
      boost::mutex::scoped_lock lock( m_RunningMutex );
      return m_IsCommunicationRunning;
    }

    bool CommunicationProcess();
    void ReadLine(std::string& rLine);
    void CheckDeadline();
    static void Handle(const boost::system::error_code& rErrorCodeIn, size_t sizeIn, boost::system::error_code* pErrorCodeOut, std::size_t* pSizeOut);

    boost::asio::io_service m_IoService;
    boost::asio::serial_port m_SerialPort;
    boost::asio::deadline_timer m_Timer;
    double m_DesiredProcessPeriod_us;
    std::string m_Device;
    uint32_t m_BaudRate;
    boost::mutex m_RunningMutex;
    bool m_IsCommunicationRunning;

    boost::thread m_Thread;
    boost::mutex m_DataMutex;
    GsaData m_GsaData;
    RmcData m_RmcData;
    GgaData m_GgaData;
};


#endif // LANDSHARKGPS_H
