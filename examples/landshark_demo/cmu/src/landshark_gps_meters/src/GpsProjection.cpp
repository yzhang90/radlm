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

#include "GpsProjection.h"
#include "LandsharkCommonFunction.h"

#include <string>
#include <iostream>
#include <sstream>
#include <exception>
#include <stdexcept>

extern "C"
{
#include <proj_api.h>
}

namespace landshark
{
  static const std::string PROJECTION_TYPE = "+proj=tmerc";
  static const std::string ELLIPSOID_TYPE = "+ellps=WGS84";
  static const std::string LONGITUDE_ZERO_ARGUMENT = "+lon_0=";
  static const std::string LATITUDE_ZERO_ARGUMENT = "+lat_0=";
  static const double LONGITUDE_MAX = 180.0;
  static const double LATITUDE_MAX = 90.0;
  static const int PROJ_OK = 0;

  struct GpsProjection::Impl
  {
    bool m_IsLocalCoordinateSystemSet;
    projPJ m_Projection;
    double m_LongitudeOffset;
    double m_LatitudeOffset;
  };

  GpsProjection::GpsProjection():
    m_pImpl(new GpsProjection::Impl())
  {}

  GpsProjection::~GpsProjection()
  {
    if (m_pImpl)
    {
      if (m_pImpl->m_Projection)
      {
        pj_free(m_pImpl->m_Projection);
      }

      delete m_pImpl;
    }
  }

  void GpsProjection::SetLocalCoordinateSystemOrigin(double longitude, double latitude)
  {
    if (!IsWithinRange(longitude, -LONGITUDE_MAX, LONGITUDE_MAX))
    {
      std::stringstream stream;
      stream << "Invalid longitude: " << longitude << " should be with +- " << LONGITUDE_MAX;

      throw std::invalid_argument(stream.str());
    }

    if (!IsWithinRange(latitude, -LATITUDE_MAX, LATITUDE_MAX))
    {
      std::stringstream stream;
      stream << "Invalid latitude: " << latitude << " should be with +- " << LATITUDE_MAX;

      throw std::invalid_argument(stream.str());
    }

    std::stringstream argumentStream;
    argumentStream  << PROJECTION_TYPE << " " << ELLIPSOID_TYPE << " "
                    << LONGITUDE_ZERO_ARGUMENT << longitude << " "
                    << LATITUDE_ZERO_ARGUMENT << latitude;

    if (!(m_pImpl->m_Projection = pj_init_plus(argumentStream.str().c_str())))
    {
      std::stringstream stream;
      stream << "Failed to initialize projection";

      int *pErrorNumber = pj_get_errno_ref();
      if (pErrorNumber)
      {
        stream << ", error: " << pj_strerrno(*pErrorNumber);
      }

      throw std::runtime_error(stream.str());
    }

    // std::cout << "pj_get_def: " << pj_get_def(m_pImpl->m_Projection, 0) << std::endl;

    // set the offset, !caution! proj4 rounds long, lat arguments during initialization
    projUV localPoint;
    localPoint.u = 0.0;
    localPoint.v = 0.0;

    projUV gpsPoint = pj_inv(localPoint, m_pImpl->m_Projection);

    int *pErrorNumber = pj_get_errno_ref();
    if (pErrorNumber && (*pErrorNumber != PROJ_OK))
    {
      std::stringstream stream;
      stream << "Failed to apply projection, error: " << pj_strerrno(*pErrorNumber);

      throw std::runtime_error(stream.str());
    }

    m_pImpl->m_LongitudeOffset = gpsPoint.u * RAD_TO_DEG - longitude;
    m_pImpl->m_LatitudeOffset = gpsPoint.v * RAD_TO_DEG - latitude;
  }

  void GpsProjection::GetLocalCoordinatesFromGps(double longitude, double latitude, double& rX, double & rY)
  {
    if (!m_pImpl->m_Projection)
    {
      throw std::logic_error("No local coordinate system has been defined.");
    }

    projUV gpsPoint;
    gpsPoint.u = (longitude + m_pImpl->m_LongitudeOffset) * DEG_TO_RAD;
    gpsPoint.v = (latitude + m_pImpl->m_LatitudeOffset) * DEG_TO_RAD;

    projUV localPoint = pj_fwd(gpsPoint, m_pImpl->m_Projection);

    int *pErrorNumber = pj_get_errno_ref();
    if (pErrorNumber && (*pErrorNumber != PROJ_OK))
    {
      std::stringstream stream;
      stream << "Failed to apply projection, error: " << pj_strerrno(*pErrorNumber);

      throw std::runtime_error(stream.str());
    }

    rX = localPoint.u;
    rY = localPoint.v;
  }

  void GpsProjection::GetGpsFromLocalCoordinates(double x, double y, double& rLongitude, double& rLatitude)
  {
    if (!m_pImpl->m_Projection)
    {
      throw std::logic_error("No local coordinate system has been defined.");
    }

    projUV localPoint;
    localPoint.u = x;
    localPoint.v = y;

    projUV gpsPoint = pj_inv(localPoint, m_pImpl->m_Projection);

    int *pErrorNumber = pj_get_errno_ref();
    if (pErrorNumber && (*pErrorNumber != PROJ_OK))
    {
      std::stringstream stream;
      stream << "Failed to apply projection, error: " << pj_strerrno(*pErrorNumber);

      throw std::runtime_error(stream.str());
    }

    rLongitude = gpsPoint.u * RAD_TO_DEG - m_pImpl->m_LongitudeOffset;
    rLatitude = gpsPoint.v * RAD_TO_DEG - m_pImpl->m_LatitudeOffset;
  }
} // namespace
