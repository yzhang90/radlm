//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
 * \file    gps_drift_server.h
 * \brief   server for the set gps drift service takes an offset in x and y 
 *\	    and takes time t to offset to this time
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================
#ifndef gpsdriftserverh
#define gpsdriftserverh

#include "ros/ros.h"
#include "cmu_gps_drift/gpsDrift.h" 
#include <sensor_msgs/NavSatFix.h>
#include <landshark_gps/GpsProjection.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <deque>
#include <std_msgs/Header.h>

const int driftExecuted = 0;
const int driftQueued = 1;
const int driftRejected = 2;
const int driftExecuting = 3;
const int noDrift = 4;
const int maxDriftQueueSize = 1000;

class gpsDriftInfoMaster
{
public:
  struct gpsDriftInfo
  {
    float driftx;
    float drifty;
    float driftz;
    float driftt;
    float dx;
    float dy;
    float dz;
    float dt;
    ros::Time startDrift;
    ros::Time lastDriftTime;
    int isDrifting;
    float driftxOffset;
    float driftyOffset;
    float driftzOffset;
    float gps_originLat;
    float gps_originLon;

    gpsDriftInfo()
    {
      driftx = 0;
      drifty = 0;
      driftz = 0;
      driftt = 0;
      dx = 0;
      dy = 0;
      dz = 0;
      dt = 0;
      isDrifting = noDrift;
      driftxOffset = 0;
      driftyOffset = 0;
      driftzOffset = 0;
      startDrift = ros::Time(0.0);
      gps_originLat = 0;
      gps_originLon = 0;

    }
    gpsDriftInfo(float glat, float glon)
    {
      driftx = 0;
      drifty = 0;
      driftz = 0;
      driftt = 0;
      dx = 0;
      dy = 0;
      dz = 0;
      dt = 0;
      isDrifting = noDrift;
      driftxOffset = 0;
      driftyOffset = 0;
      driftzOffset = 0;
      startDrift = ros::Time(0.0);
      gps_originLat = glat;
      gps_originLon = glon;
    }
    void resetDriftInfo()
    {
      driftx = 0;
      drifty = 0;
      driftz = 0;
      driftt = 0;
      dx = 0;
      dy = 0;
      dz = 0;
      dt = 0;
      isDrifting = noDrift;
      startDrift = ros::Time(0.0);
    }
    void initDriftInfo(float glat, float glon)
    {
      driftx = 0;
      drifty = 0;
      driftz = 0;
      driftt = 0;
      dx = 0;
      dy = 0;
      dz = 0;
      dt = 0;
      isDrifting = noDrift;
      driftxOffset = 0;
      driftyOffset = 0;
      startDrift = ros::Time(0.0);
      gps_originLat = glat;
      gps_originLon = glon;
    }
  };
  std::deque<gpsDriftInfo> driftInfoVector;
  float gpsDriftOffsetx;
  float gpsDriftOffsety;
  float gpsDriftOffsetz;
  bool drifting;
  ros::Time timeNow;
  geometry_msgs::PointStamped gpsSpoofed;
  geometry_msgs::PointStamped biasInfo;
  gpsDriftInfoMaster();
  int addDrift(cmu_gps_drift::gpsDrift::Request &req);
  void do_passive_gps_drift(float gps_x, float gps_y, float gps_alt, std_msgs::Header hdr);
  void do_active_gps_drift(float gps_x, float gps_y, float gps_alt, std_msgs::Header hdr);
};

class gpsDriftServer
{
public:

  ros::NodeHandle nh;
  geometry_msgs::PointStamped gpsMeters;
  std::vector<float> gpsLatLonOrigin;
  landshark::GpsProjection gpsProjection;
  ros::Subscriber gpsSub;

  gpsDriftInfoMaster gpsDriftInfoM;
//  ros::Time timeBefore;
//  ros::Time startDrift;
//  bool drifting;
  ros::Publisher gpsMetersPub;
  ros::Publisher biasPub;
  ros::Publisher spoofedMetersPub;

  bool set_drift_parameters(cmu_gps_drift::gpsDrift::Request &req,
                            cmu_gps_drift::gpsDrift::Response &res);
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &pMsg);
  void initGPS(float gps_lon_origin, float gps_lat_origin);
  gpsDriftServer(ros::NodeHandle n);
};

#endif
