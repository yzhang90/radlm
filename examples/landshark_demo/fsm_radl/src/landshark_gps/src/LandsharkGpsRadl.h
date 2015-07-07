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
 * Author(s): Aravind Sundaresan (aravind@ai.sri.com)
 */

#ifndef LANDSHARKGPSRADL_H
#define LANDSHARKGPSRADL_H

#include "LandsharkGps.h" 

#ifdef PUBLISH_SENSOR_MSGS
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/TwistStamped.h"
#endif // PUBLISH_SENSOR_MSGS

#include RADL_HEADER

class LandsharkGpsRadl : public LandsharkGps 
{
  public:
    inline LandsharkGpsRadl() 
    {
      m_Device = *RADL_THIS->device;
      m_BaudRate = *RADL_THIS->baudrate;

      Start();
    }

    inline ~LandsharkGpsRadl()
    {
    }

    inline int step( const radl_in_t *in, const radl_in_flags_t *in_flags, radl_out_t *out, radl_out_flags_t *out_flags )
    {
      bool hasEnoughSatellites = false;

      boost::mutex::scoped_lock lock( m_DataMutex );
      if ( m_GsaData.m_Fresh ) {
        if (m_GsaData.m_PositioningType == GSA_3D) {
          hasEnoughSatellites = true;
        }
        else {
          hasEnoughSatellites = false;
        }
      }

      if ( m_GsaData.m_Fresh && hasEnoughSatellites) {
        out->twist->stamp = radl_gettime();

        out->twist->linear.x = m_RmcData.m_SpeedOverGround * KNOTS_TO_METER_PER_S * sin(DEGREE_TO_RADIAN * m_RmcData.m_CourseOverGround);
        out->twist->linear.y = m_RmcData.m_SpeedOverGround * KNOTS_TO_METER_PER_S * cos(DEGREE_TO_RADIAN * m_RmcData.m_CourseOverGround);
        out->twist->linear.z = 0;
        out->twist->angular.x = 0;
        out->twist->angular.y = 0;
        out->twist->angular.z = 0;
        m_GsaData.m_Fresh = false;
      }
      else {
        radl_turn_on( radl_STALE, &out_flags->twist );
      }

      if ( m_GgaData.m_Fresh ) {
        m_GgaData.m_Fresh = false;
        switch (m_GgaData.m_GpsQuality) {
          case GGA_NOT_AVAILABLE:
            out->navsatfix->status.status = radlast_constants()->NavSatStatus->STATUS_NO_FIX;
            break;
          case GGA_NON_DIFFERENTIAL_GPS:
            out->navsatfix->status.status = radlast_constants()->NavSatStatus->STATUS_FIX;
            break;
          case GGA_DIFFERENTIAL_GPS:
            out->navsatfix->status.status = radlast_constants()->NavSatStatus->STATUS_SBAS_FIX;
            break;
          case GGA_ESTIMATED:
            out->navsatfix->status.status = radlast_constants()->NavSatStatus->STATUS_NO_FIX;
            break;
          default:
            // Not sure if this should be evaluated.
            out->navsatfix->status.status = radlast_constants()->NavSatStatus->STATUS_NO_FIX;
            std::cout << "Unknown value: m_GpsQuality= " << m_GgaData.m_GpsQuality << std::endl;
            break;
        }

        for ( size_t i = 0; i < 9; i++ ) {
          out->navsatfix->position_covariance[0] = 0;
        }

        out->navsatfix->status.service = radlast_constants()->NavSatStatus->SERVICE_GPS;
        out->navsatfix->stamp  = radl_gettime();

        out->navsatfix->latitude = m_GgaData.m_Latitude;
        out->navsatfix->longitude = m_GgaData.m_Longitude;

        out->navsatfix->position_covariance[0] = m_GgaData.m_HorizontalDilutionOfPrecision * m_GgaData.m_HorizontalDilutionOfPrecision;
        out->navsatfix->position_covariance[4] = out->navsatfix->position_covariance[0];
        out->navsatfix->position_covariance[8] = 4 * out->navsatfix->position_covariance[0];
        out->navsatfix->position_covariance_type = radlast_constants()->NavSatFix->COVARIANCE_TYPE_APPROXIMATED;

        out->navsatfix->altitude = m_GgaData.m_AntennaElevation + m_GgaData.m_GeoidalHeight;

        // XXX This is inaccurate (Thomas De Candia)
        boost::posix_time::ptime ctime(boost::posix_time::second_clock::universal_time());
        boost::posix_time::ptime gtime(ctime.date(), boost::posix_time::time_duration(m_GgaData.m_UtcTime.m_Hours, m_GgaData.m_UtcTime.m_Minutes, m_GgaData.m_UtcTime.m_Seconds));
        out->timeref->stamp = out->navsatfix->stamp;
        out->timeref->time_ref = out->navsatfix->stamp;
      }
      else {
        radl_turn_on( radl_STALE, &out_flags->navsatfix );
      }

#ifdef PUBLISH_SENSOR_MSGS
      {
        static ros::NodeHandle nh( "~" );
        static ros::Publisher p1 = nh.advertise<sensor_msgs::NavSatFix>("navsatfix", 100);
        static ros::Publisher p2 = nh.advertise<geometry_msgs::TwistStamped>("velocity", 100);
        static sensor_msgs::NavSatFix msg1;
        static geometry_msgs::TwistStamped msg2;
        msg1.latitude = out->navsatfix->latitude;
        msg1.longitude = out->navsatfix->longitude;
        msg1.altitude = out->navsatfix->altitude;
        msg1.header.stamp = ros::Time::now();
        msg1.header.frame_id = "gps";
        msg1.status.status = out->navsatfix->status.status;
        msg1.status.service = out->navsatfix->status.service;
        msg1.position_covariance_type = out->navsatfix->position_covariance_type;
        p1.publish( msg1 );
        msg2.twist.linear.x = out->twist->linear.x;
        msg2.twist.linear.y = out->twist->linear.z;
        msg2.twist.linear.z = out->twist->linear.z;
        msg2.twist.angular.x = out->twist->angular.x;
        msg2.twist.angular.y = out->twist->angular.z;
        msg2.twist.angular.z = out->twist->angular.z;
        msg2.header = msg1.header;
        p2.publish( msg2 );
      }
#endif // PUBLISH_SENSOR_MSGS

    }
};

#endif // LANDSHARKGPSRADL_H


