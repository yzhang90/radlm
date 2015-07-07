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
 * \file    gps_drift_server.cpp
 * \brief   server for the set gps drift service
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================
#include "ros/ros.h"
#include "cmu_gps_drift/gpsDrift.h" 
#include <sensor_msgs/NavSatFix.h>
#include <landshark_gps/GpsProjection.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include "gps_drift_server.h"
gpsDriftInfoMaster::gpsDriftInfoMaster()
{
  gpsDriftOffsetx = 0;
  gpsDriftOffsety = 0;
  gpsDriftOffsetz = 0;
  drifting = false;
  ros::Time timeNow = ros::Time(0.0);

}
int gpsDriftInfoMaster::addDrift(cmu_gps_drift::gpsDrift::Request &req)
{
  bool adddrift = true;
  gpsDriftInfo driftInfo;
  int err_code = 0;
//	debugging
//  printf("\nDriftVector Size %d", (int)driftInfoVector.size());
  if (driftInfoVector.size() >= maxDriftQueueSize)
  {
    err_code = driftRejected;
    ROS_INFO(
        "request rejected: driftx: %f drifty: %f driftz: %f driftt: %f", driftInfo.driftx, driftInfo.drifty, driftInfo.driftz, driftInfo.driftt);
    adddrift = false;
  }

  else if (drifting)
  {
    err_code = driftQueued;
    ROS_INFO(
        "request queued: driftx: %f drifty: %f driftz: %f driftt: %f", driftInfo.driftx, driftInfo.drifty, driftInfo.driftz, driftInfo.driftt);
  }
  else
  {
    ROS_INFO(
        "request: driftx: %f drifty: %f driftz: %f driftt: %f", driftInfo.driftx, driftInfo.drifty, driftInfo.driftz, driftInfo.driftt);
    err_code = driftExecuted;
  }
  if (adddrift)
  {
    driftInfo.driftx = req.drift_x - gpsDriftOffsetx;
    driftInfo.drifty = req.drift_y - gpsDriftOffsety;
    driftInfo.driftz = req.drift_z - gpsDriftOffsetz;
    driftInfo.driftt = req.drift_time;
    float driftTime = driftInfo.driftt;
    if (driftInfo.driftt == 0)
      driftTime = 1;
    driftInfo.dx = driftInfo.driftx / driftTime;
    driftInfo.dy = driftInfo.drifty / driftTime;
    driftInfo.dz = driftInfo.driftz / driftTime;
    driftInfo.dt = driftTime / driftTime;

    if (!drifting)
    {
      driftInfo.startDrift = ros::Time::now();
      driftInfo.isDrifting = driftExecuting;
      drifting = true;
    }
    else
    {
      driftInfo.isDrifting = driftQueued;
      driftInfo.startDrift = ros::Time(0.0);
    }
    if (driftInfoVector.size() < maxDriftQueueSize)
      driftInfoVector.push_back(driftInfo);
  }
//	debugging
//  printf("\nDriftVector Size %d", (int)driftInfoVector.size());
  return err_code;
}
void gpsDriftInfoMaster::do_passive_gps_drift(float gps_x, float gps_y, float gps_alt, std_msgs::Header hdr)
{
  biasInfo.header = hdr;
  gpsSpoofed.header = hdr;
  timeNow = hdr.stamp;

  biasInfo.point.x = gpsDriftOffsetx;
  biasInfo.point.y = gpsDriftOffsety;
  biasInfo.point.z = gpsDriftOffsetz;
  gpsSpoofed.point.x = gps_x + gpsDriftOffsetx;
  gpsSpoofed.point.y = gps_y + gpsDriftOffsety;
  gpsSpoofed.point.z = gps_alt + gpsDriftOffsetz;

}
void gpsDriftInfoMaster::do_active_gps_drift(float gps_x, float gps_y, float gps_alt, std_msgs::Header hdr)
{
  float x;
  float y;
  float z;
  if (driftInfoVector[0].isDrifting == driftExecuting)
  {
    ros::Duration dt = timeNow - driftInfoVector[0].startDrift;
    if (dt.toSec() <= driftInfoVector[0].driftt && dt.toSec() > 0)
    {
      x = gps_x + driftInfoVector[0].dx * dt.toSec() + gpsDriftOffsetx;
      y = gps_y + driftInfoVector[0].dy * dt.toSec() + gpsDriftOffsety;
      z = gps_alt + driftInfoVector[0].dz * dt.toSec() + gpsDriftOffsetz;

      biasInfo.header = hdr;
      biasInfo.point.x = driftInfoVector[0].dx * dt.toSec() + gpsDriftOffsetx;
      biasInfo.point.y = driftInfoVector[0].dy * dt.toSec() + gpsDriftOffsety;
      biasInfo.point.z = driftInfoVector[0].dz * dt.toSec() + gpsDriftOffsetz;

      gpsSpoofed.header = hdr;
      gpsSpoofed.point.x = x;
      gpsSpoofed.point.y = y;
      gpsSpoofed.point.z = z;

      printf("\nt:%f x:%f y:%f z:%f\n", dt.toSec(), x, y, z);
    }
    else
    {
      driftInfoVector[0].isDrifting = driftExecuted;
      driftInfoVector[0].driftxOffset += driftInfoVector[0].driftx;
      driftInfoVector[0].driftyOffset += driftInfoVector[0].drifty;
      driftInfoVector[0].driftzOffset += driftInfoVector[0].driftz;
      gpsDriftOffsetx += driftInfoVector[0].driftx;
      gpsDriftOffsety += driftInfoVector[0].drifty;
      gpsDriftOffsetz += driftInfoVector[0].driftz;
      gpsSpoofed.point.x = gps_x + gpsDriftOffsetx;
      gpsSpoofed.point.y = gps_y + gpsDriftOffsety;
      gpsSpoofed.point.z = gps_alt + gpsDriftOffsetz;
      biasInfo.header = hdr;
      biasInfo.point.x = gpsDriftOffsetx;
      biasInfo.point.y = gpsDriftOffsety;
      biasInfo.point.z = gpsDriftOffsetz;
      printf("\nt:%f x:%f y:%f z:%f\n", dt.toSec(), gpsSpoofed.point.x, gpsSpoofed.point.y, gpsSpoofed.point.z);
      driftInfoVector.pop_front();
      drifting = false;
      //debugging
//	if(driftInfoVector.size() == 0)
//		printf("\nDrift Vector empty\n");
//
//      printf("\nDriftVector Size %d\n", (int)driftInfoVector.size());
      if (driftInfoVector.size() > 0)
      {
        if (driftInfoVector[0].isDrifting == driftQueued)
        {
          driftInfoVector[0].startDrift = ros::Time::now();
          driftInfoVector[0].isDrifting = driftExecuting;
          drifting = true;

        }
      }

    }
  }

}
bool gpsDriftServer::set_drift_parameters(cmu_gps_drift::gpsDrift::Request &req,
                                          cmu_gps_drift::gpsDrift::Response &res)
{
  ROS_INFO("request: x drift= %f ydrift=%f zdrift=%f time=%f", req.drift_x, req.drift_y, req.drift_z, req.drift_time);

  res.err_code = gpsDriftInfoM.addDrift(req);

  return true;
}

void gpsDriftServer::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &pMsg)
{
  float gps_lat = pMsg->latitude;
  float gps_lon = pMsg->longitude;
  float gps_alt = pMsg->altitude;
  gpsMeters.header = pMsg->header;
  double gps_x;
  double gps_y;

  try
  {
    gpsProjection.GetLocalCoordinatesFromGps(gps_lon, gps_lat, gps_x, gps_y);
//// 	for debugging
//    gps_x = 0;
//    gps_y = 0;
//    gps_alt = 0;
    gpsMeters.point.x = gps_x;
    gpsMeters.point.y = gps_y;
    gpsMeters.point.z = gps_alt;
    gpsDriftInfoM.do_passive_gps_drift(gps_x, gps_y, gps_alt, pMsg->header);

  }
  catch (std::exception& exception)
  {
    printf("\nPoint Too Far");
  }
  gpsDriftInfoM.do_active_gps_drift(gps_x, gps_y, gps_alt, pMsg->header);
  gpsMetersPub.publish(gpsMeters);
  biasPub.publish(gpsDriftInfoM.biasInfo);
  spoofedMetersPub.publish(gpsDriftInfoM.gpsSpoofed);

}
void gpsDriftServer::initGPS(float gps_lon_origin, float gps_lat_origin)
{
  gpsProjection.SetLocalCoordinateSystemOrigin(gps_lon_origin, gps_lat_origin);

}
gpsDriftServer::gpsDriftServer(ros::NodeHandle n)
{
  nh = n;
  static const double SF_long = -122.429988645;
  static const double SF_lat = 37.78298205;
  static const double CMU_long = -79.945;
  static const double CMU_lat = 40.443;

  initGPS(CMU_long,CMU_lat);
  gpsSub = nh.subscribe<sensor_msgs::NavSatFix>("/landshark/gps", 1, &gpsDriftServer::gpsCallback, this);
  spoofedMetersPub = nh.advertise<geometry_msgs::PointStamped>("/landshark/gps_meters", 1);
  biasPub = nh.advertise<geometry_msgs::PointStamped>("/landshark/gps_meters_bias", 1);
  gpsMetersPub = nh.advertise<geometry_msgs::PointStamped>("/landshark/gps_meters_clean", 1);

  ROS_INFO("Ready to drift data");

}

