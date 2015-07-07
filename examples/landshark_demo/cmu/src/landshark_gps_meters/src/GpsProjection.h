/*
 * Copyright (C) 2012, SRI International
 *
 * The material contained in this release is copyrighted. It may not be copied,
 * reproduced, translated, reverse engineered, modified or reduced to any
 * electronic medium or machine-readable form without the prior written consent
 * of REARM team .
 *
 * Portions of files in this release may be unpublished work
 * containing REARM team CONFIDENTIAL AND PROPRIETARY INFORMATION.
 * Disclosure, use, reverse engineering, modification, or reproduction without
 * written authorization of REARM team is likewise prohibited.
 *
 * Author(s): Thomas de Candia (thomasd@ai.sri.com)
 *
 * $Id$
 *
 */

#ifndef GPSPROJECTION_H
#define GPSPROJECTION_H

namespace landshark
{
  /**
   * This class provides an interface to convert GPS coordinates from/to local metric coordinates.
   */
  class GpsProjection
  {
  public:
    /**
     * Constructor
     */
    GpsProjection();

    /**
     * Destructor
     */
    ~GpsProjection();

    /**
     * Set the origin of the local coordinate sytem.
     * The x axe of the local coordinate system points toward the east.
     * The y axe of the local coordinate system points toward the north.
     * @param longitude longitude of the local coordinate system [deg]
     * @param latitude latitude of the local coordinate system [deg]
     * @throw std::invalid_argument if the longitude/latitude are not within +- 180/90 deg.
     * @throw std::runtime_error thrown when unable to initialize projection
     */
    void SetLocalCoordinateSystemOrigin(double longitude, double latitude);

    /**
     * Retrieve the local coordinates corresponding to GPS coordinates.
     * @param longitude longitude [deg]
     * @param latitude latitude [deg]
     * @param rX x component of the local coordinates [m]
     * @param rY y component of the local coordinates [m]
     * @throw std::logic_error thrown if the local coordinate system origin has not been set
     * @throw std::runtime_error if the projection fails
     */
    void GetLocalCoordinatesFromGps(double longitude, double latitude, double& rX, double & rY);

    /**
     * Retrieve the GPS coordinates corresponding to local coordinates.
     * @param x x component of the local coordinates [m]
     * @param y y component of the local coordinates [m]
     * @param rLongitude corresponding longitude [deg]
     * @param rLatitude corresponding latitude [deg]
     * @throw std::logic_error thrown if the local coordinate system origin has not been set
     * @throw std::runtime_error if the projection fails
     */
    void GetGpsFromLocalCoordinates(double x, double y, double& rLongitude, double& rLatitude);

  private:
    struct Impl;
    Impl* m_pImpl;
  };

} // namespace

#endif
