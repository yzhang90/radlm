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
 * \author  Fatma Faruq, Jason Larkin
 * \other   HACMS CMU Advisor Manuela Veloso, SpiralGen, Inc.
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

//--------------------------------------------------------------------------------
class gpsDriftServer
{
public:
  ros::NodeHandle nh;
  gpsDriftServer(ros::NodeHandle n);
  landshark::GpsProjection gpsProjection;
  
  ros::Subscriber subGps;
  ros::Publisher pubGpsMeters;
  geometry_msgs::PointStamped gpsMeters;
  ros::Publisher pubGpsMetersBias;
  geometry_msgs::PointStamped gpsMetersBias;
  ros::Publisher pubGpsMetersClean;
  geometry_msgs::PointStamped gpsMetersClean;
  
  struct gpsDriftInfo
  {
    double drift_x;double drift_y;double drift_z;double drift_t;
    double dx;double dy;double dz;double dt;double drift_time;
    ros::Time startDrift;
    ros::Time lastDriftTime;
    int isDrifting;
    double driftxOffset;double driftyOffset;double driftzOffset;
    
    double gps_x_origin;
    double gps_y_origin;
    double gps_lat_origin;
    double gps_lon_origin;
    
    std::deque<gpsDriftInfo> driftInfoVector;
    
    gpsDriftInfo()
    {
      drift_x = 0.0;drift_y = 0.0;drift_z = 0.0;drift_t = 0.0;
      dx = 0.0;dy = 0.0;dz = 0.0;dt = 0.0;drift_time = 0.0;
      isDrifting = noDrift;
      driftxOffset = 0.0;
      driftyOffset = 0.0;
      driftzOffset = 0.0;
      
      gps_x_origin = 0.0;
      gps_y_origin = 0.0;
      gps_lat_origin = 0.0;
      gps_lon_origin = 0.0;
      
      startDrift = ros::Time(0.0);
    }
  };
  
  gpsDriftInfo driftInfo;

  void initialize(double gps_lat_origin, double gps_lon_origin);
  bool set_drift_parameters(\
  cmu_gps_drift::gpsDrift::Request &req,\
  cmu_gps_drift::gpsDrift::Response &res);
  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &pMsg);
  
  int addDrift(cmu_gps_drift::gpsDrift::Request &req);
  int addDrift_single(cmu_gps_drift::gpsDrift::Request &req);
  
  void do_passive_gps_drift(\
  double gps_x, double gps_y, double gps_alt, std_msgs::Header hdr);
  void do_active_gps_drift(\
  double gps_x, double gps_y, double gps_alt, std_msgs::Header hdr);
  void initGPSMetersOrigin(\
  double gps_x, double gps_y, double gps_alt, std_msgs::Header hdr);
  
};
//--------------------------------------------------------------------------------
#endif
