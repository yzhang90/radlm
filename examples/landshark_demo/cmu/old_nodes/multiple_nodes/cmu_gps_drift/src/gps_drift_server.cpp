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
 * \author  Fatma Faruq, Jason Larkin
 * \other   HACMS CMU Advisor Manuela Veloso, SpiralGen, Inc.
 */
//========================================================================
#include "ros/ros.h"
#include "cmu_gps_drift/gpsDrift.h" 
#include <sensor_msgs/NavSatFix.h>
#include <landshark_gps/GpsProjection.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include "gps_drift_server.h"

//--------------------------------------------------------------------------------
gpsDriftServer::gpsDriftServer(ros::NodeHandle n)
{
  nh = n; 
  subGps = nh.subscribe<sensor_msgs::NavSatFix>(\
  "/landshark/gps", 1, &gpsDriftServer::gpsCallback, this);
  pubGpsMetersBias = nh.advertise<geometry_msgs::PointStamped>(\
  "/landshark/gps_meters_bias", 1);
  pubGpsMeters = nh.advertise<geometry_msgs::PointStamped>(\
  "/landshark/gps_meters", 1);
  pubGpsMetersClean = nh.advertise<geometry_msgs::PointStamped>(\
  "/landshark/gps_meters_clean", 1);
//  spoofedMetersPub_norm = nh.advertise<geometry_msgs::PointStamped>("/landshark/gps_meters_norm", 1);
//  gpsMetersPub_norm = nh.advertise<geometry_msgs::PointStamped>("/landshark/gps_meters_clean_norm", 1);
  ROS_INFO("Ready to drift data...\n");
}
//--------------------------------------------------------------------------------
void gpsDriftServer::initialize(double gps_lat_origin, double gps_lon_origin)
{
//  static const double SF_lat = 37.78298205;
//  static const double SF_long = -122.429988645;
//  static const double SRI_lat = 37.4571017302;
//  static const double SRI_long = -122.173497004;
//  static const double CMU_lat = 40.443;
//  static const double CMU_long = -79.945;

//  gpsDriftInfo gpsDriftInfo;

  ROS_INFO("gpsDriftServer::Initialize \n");
  driftInfo.gps_lat_origin = gps_lat_origin;
  driftInfo.gps_lon_origin = gps_lon_origin;
  gpsProjection.SetLocalCoordinateSystemOrigin(gps_lon_origin, gps_lat_origin); 
  
  ROS_INFO("initial: drift_x= %f drift_y=%f drift_z=%f drift_time=%f",\
  driftInfo.drift_x, driftInfo.drift_y, driftInfo.drift_z, driftInfo.drift_time); 
  
//  gpsDriftServer::initGPSCoordSysOrigin(gps_lat_origin,gps_lon_origin);
//  gpsDriftServer::initGPSOrigin(gps_lat_origin,gps_lon_origin);
}
//--------------------------------------------------------------------------------
bool gpsDriftServer::set_drift_parameters(\
cmu_gps_drift::gpsDrift::Request &req,\
cmu_gps_drift::gpsDrift::Response &res)
{
  ROS_INFO("request: drift_x= %f drift_y=%f drift_z=%f drift_time=%f",\
  req.drift_x, req.drift_y, req.drift_z, req.drift_time);
  
  driftInfo.drift_x = req.drift_x; 
  driftInfo.drift_y = req.drift_y; 
  driftInfo.drift_z = req.drift_z; 
  driftInfo.drift_time = req.drift_time;
  
//JASON
//  res.err_code = gpsDriftInfoM.addDrift(req);
    res.err_code = addDrift_single(req);
  return true;
}
//--------------------------------------------------------------------------------
//void gpsDriftServer::initGPS(double gps_lat_origin, double gps_lon_origin)
//{
//  gpsProjection.SetLocalCoordinateSystemOrigin(gps_lon_origin, gps_lat_origin);
//}
//--------------------------------------------------------------------------------
//void gpsDriftServer::initGPSOrigin(double gps_lat_origin, double gps_lon_origin)
//{
//  gpsProjection.GetLocalCoordinatesFromGps(\
//  gps_lon_origin, gps_lat_origin, driftInfo.gps_x_origin,\
//  driftInfo.gps_y_origin);
//  printf("\n gps_x_origin= %f gps_y_origin= %f\n", driftInfo.gps_x_origin,\
//  driftInfo.gps_y_origin );
//}
//--------------------------------------------------------------------------------
//JASON
int gpsDriftServer::addDrift_single(\
cmu_gps_drift::gpsDrift::Request &req)
{
  int err_code = 0; 
//  driftInfo.driftx = req.drift_x - gpsDriftOffsetx;
//  driftInfo.drifty = req.drift_y - gpsDriftOffsety;
//  driftInfo.driftz = req.drift_z - gpsDriftOffsetz;
  driftInfo.drift_x = req.drift_x;
  driftInfo.drift_y = req.drift_y;
  driftInfo.drift_z = req.drift_z;
  
  driftInfo.drift_t = req.drift_time;
  double driftTime = driftInfo.drift_t;
  if (driftInfo.drift_t == 0)
    driftTime = 1;
  driftInfo.dx = driftInfo.drift_x / driftTime;
  driftInfo.dy = driftInfo.drift_y / driftTime;
  driftInfo.dz = driftInfo.drift_z / driftTime;
  driftInfo.dt = driftTime / driftTime;
  driftInfo.startDrift = ros::Time::now();
  driftInfo.isDrifting = driftExecuting;
//  drifting = true;
//  driftInfoVector[0] = driftInfo;
return err_code;
}
//--------------------------------------------------------------------------------
void gpsDriftServer::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &pMsg)
{
  double gps_lat = pMsg->latitude; double gps_lon = pMsg->longitude;
  double gps_alt = pMsg->altitude; gpsMeters.header = pMsg->header;
  double gps_x; double gps_y;
  try
  {
//    gpsProjection.SetLocalCoordinateSystemOrigin(\
//    driftInfo.gps_lon_origin, driftInfo.gps_lat_origin);
    gpsProjection.SetLocalCoordinateSystemOrigin(\
    driftInfo.gps_lon_origin, driftInfo.gps_lat_origin);
//  printf("\ngpsCallback\n");
//  printf("\n gps_lat = %f gps_lon = %f \n", gps_lat,gps_lon);
//  printf("\n driftInfo.gps_lat_origin = %f driftInfo.gps_lon_origin = %f \n",\
//  driftInfo.gps_lat_origin,driftInfo.gps_lon_origin);

    gpsProjection.GetLocalCoordinatesFromGps(\
    gps_lon, gps_lat, gps_x, gps_y);
    gpsMetersClean.point.x = gps_x; //- gpsDriftInfoM.gps_x_origin;
    gpsMetersClean.point.y = gps_y; //- gpsDriftInfoM.gps_y_origin;
    gpsMetersClean.point.z = gps_alt;
    do_passive_gps_drift(gps_x, gps_y, gps_alt, pMsg->header);
  }
  catch (std::exception& exception)
  {
    ROS_INFO("\nPoint Too Far\n");
  }
//JASON
//  gpsDriftInfoM.do_active_gps_drift(gps_x, gps_y, gps_alt, pMsg->header);
  pubGpsMeters.publish(gpsMeters);
  pubGpsMetersBias.publish(gpsMetersBias);
  pubGpsMetersClean.publish(gpsMetersClean);
}
//--------------------------------------------------------------------------------
void gpsDriftServer::do_passive_gps_drift(\
double gps_x, double gps_y, double gps_alt, std_msgs::Header hdr)
{
  gpsMetersBias.header = hdr;
  gpsMeters.header = hdr;
//  timeNow = hdr.stamp;

  gpsMetersBias.point.x = driftInfo.drift_x;
  gpsMetersBias.point.y = driftInfo.drift_y;
  gpsMetersBias.point.z = driftInfo.drift_z;
  gpsMeters.point.x = gps_x + driftInfo.drift_x;// - gps_x_origin;
  gpsMeters.point.y = gps_y + driftInfo.drift_y;// - gps_y_origin;
  gpsMeters.point.z = gps_alt + driftInfo.drift_z;   
}
//--------------------------------------------------------------------------------























//int gpsDriftInfoMaster::addDrift(landshark_cmu_gps_drift::gpsDrift::Request &req)
//{
//  bool adddrift = true;
//  gpsDriftInfo driftInfo;
//  int err_code = 0;
////	debugging
////  printf("\nDriftVector Size %d", (int)driftInfoVector.size());
//  if (driftInfoVector.size() >= maxDriftQueueSize)
//  {
////    printf("\ndriftInfoVector.size():%lu maxDriftQueueSize:%d \n", driftInfoVector.size(), maxDriftQueueSize);
//    err_code = driftRejected;
//    ROS_INFO(
//        "request rejected: driftx: %f drifty: %f driftz: %f driftt: %f", driftInfo.driftx, driftInfo.drifty, driftInfo.driftz, driftInfo.driftt);
//    adddrift = false;
//  }

//  else if (drifting)
//  {
//    err_code = driftQueued;
//    ROS_INFO(
//        "request queued: driftx: %f drifty: %f driftz: %f driftt: %f", driftInfo.driftx, driftInfo.drifty, driftInfo.driftz, driftInfo.driftt);
//  }
//  else
//  {
//    ROS_INFO(
//        "request: driftx: %f drifty: %f driftz: %f driftt: %f", driftInfo.driftx, driftInfo.drifty, driftInfo.driftz, driftInfo.driftt);
//    err_code = driftExecuted;
//  }
//  if (adddrift)
//  {
//    driftInfo.driftx = req.drift_x - gpsDriftOffsetx;
//    driftInfo.drifty = req.drift_y - gpsDriftOffsety;
//    driftInfo.driftz = req.drift_z - gpsDriftOffsetz;
//    driftInfo.driftt = req.drift_time;
//    double driftTime = driftInfo.driftt;
//    if (driftInfo.driftt == 0)
//      driftTime = 1;
//    driftInfo.dx = driftInfo.driftx / driftTime;
//    driftInfo.dy = driftInfo.drifty / driftTime;
//    driftInfo.dz = driftInfo.driftz / driftTime;
//    driftInfo.dt = driftTime / driftTime;

//    if (!drifting)
//    {
//      driftInfo.startDrift = ros::Time::now();
//      driftInfo.isDrifting = driftExecuting;
//      drifting = true;
//    }
//    else
//    {
//      driftInfo.isDrifting = driftQueued;
//      driftInfo.startDrift = ros::Time(0.0);
//    }
//    if (driftInfoVector.size() < maxDriftQueueSize)
//      driftInfoVector.push_back(driftInfo);
//  }
////	debugging
////  printf("\nDriftVector Size %d", (int)driftInfoVector.size());
//  return err_code;
//}




//void gpsDriftInfoMaster::do_active_gps_drift(double gps_x, double gps_y, double gps_alt, std_msgs::Header hdr)
//{
//  double x;
//  double y;
//  double z;
//  if (driftInfoVector[0].isDrifting == driftExecuting)
//  {
//    ros::Duration dt = timeNow - driftInfoVector[0].startDrift;
//    if (dt.toSec() <= driftInfoVector[0].driftt && dt.toSec() > 0)
//    {
//      x = gps_x + driftInfoVector[0].dx * dt.toSec() + gpsDriftOffsetx;
//      y = gps_y + driftInfoVector[0].dy * dt.toSec() + gpsDriftOffsety;
//      z = gps_alt + driftInfoVector[0].dz * dt.toSec() + gpsDriftOffsetz;

//      biasInfo.header = hdr;
//      biasInfo.point.x = driftInfoVector[0].dx * dt.toSec() + gpsDriftOffsetx;
//      biasInfo.point.y = driftInfoVector[0].dy * dt.toSec() + gpsDriftOffsety;
//      biasInfo.point.z = driftInfoVector[0].dz * dt.toSec() + gpsDriftOffsetz;

//      gpsSpoofed.header = hdr;
//      gpsSpoofed.point.x = x;
//      gpsSpoofed.point.y = y;
//      gpsSpoofed.point.z = z;

//      printf("\nt:%f x:%f y:%f z:%f\n", dt.toSec(), x, y, z);
//    }
//    else
//    {
//      driftInfoVector[0].isDrifting = driftExecuted;
//      driftInfoVector[0].driftxOffset += driftInfoVector[0].driftx;
//      driftInfoVector[0].driftyOffset += driftInfoVector[0].drifty;
//      driftInfoVector[0].driftzOffset += driftInfoVector[0].driftz;
//      gpsDriftOffsetx += driftInfoVector[0].driftx;
//      gpsDriftOffsety += driftInfoVector[0].drifty;
//      gpsDriftOffsetz += driftInfoVector[0].driftz;
//      gpsSpoofed.point.x = gps_x + gpsDriftOffsetx;
//      gpsSpoofed.point.y = gps_y + gpsDriftOffsety;
//      gpsSpoofed.point.z = gps_alt + gpsDriftOffsetz;
//      biasInfo.header = hdr;
//      biasInfo.point.x = gpsDriftOffsetx;
//      biasInfo.point.y = gpsDriftOffsety;
//      biasInfo.point.z = gpsDriftOffsetz;
//      printf("\nt:%f x:%f y:%f z:%f\n", dt.toSec(), gpsSpoofed.point.x, gpsSpoofed.point.y, gpsSpoofed.point.z);
//      driftInfoVector.pop_front();
//      drifting = false;
//      //debugging
////	if(driftInfoVector.size() == 0)
////		printf("\nDrift Vector empty\n");
////
////      printf("\nDriftVector Size %d\n", (int)driftInfoVector.size());
//      if (driftInfoVector.size() > 0)
//      {
//        if (driftInfoVector[0].isDrifting == driftQueued)
//        {
//          driftInfoVector[0].startDrift = ros::Time::now();
//          driftInfoVector[0].isDrifting = driftExecuting;
//          drifting = true;

//        }
//      }

//    }
//  }

//}


