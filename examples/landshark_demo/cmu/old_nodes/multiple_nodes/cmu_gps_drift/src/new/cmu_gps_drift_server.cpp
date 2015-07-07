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
 * \file    gps_drift_server_start.cpp
 * \brief   starts the service
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
#include <iostream> 
#include <cstdlib> 
using namespace std;
//--------------------------------------------------------------------------------
sensor_msgs::NavSatFix msgGps;
cmu_gps_drift::gpsDrift::Request reqGpsDrift;

static const double SRI_lon = -122.177062167;
static const double SRI_lat = 37.4559631667;
//--------------------------------------------------------------------------------
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &pMsg)
{msgGps = *pMsg;}

bool gpsDriftCallback(cmu_gps_drift::gpsDrift::Request &request,\
cmu_gps_drift::gpsDrift::Response &response)
{reqGpsDrift = request;}
//--------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cmu_gps_drift_server");
  ros::NodeHandle nh;
//  gpsDriftServer drifter(n);
//  drifter.initialize(atof(argv[1]),atof(argv[2]));
  
  double origin_lat = SRI_lat; double origin_lon = SRI_lon;
  double gps_x; double gps_y; double gps_altitude = 0.0;
  
  string paramName;

  geometry_msgs::PointStamped gpsMeters; 
  geometry_msgs::PointStamped gpsMetersBias;
  geometry_msgs::PointStamped gpsMetersClean;
  landshark::GpsProjection gpsProjection;
  
  ros::Subscriber subGps;
  subGps = nh.subscribe<sensor_msgs::NavSatFix>(\
  "/landshark/gps", 1, gpsCallback);
  ros::Publisher pubGpsMetersBias;
  pubGpsMetersBias = nh.advertise<geometry_msgs::PointStamped>(\
  "/landshark/gps_meters_bias", 1);
  ros::Publisher pubGpsMeters;
  pubGpsMeters = nh.advertise<geometry_msgs::PointStamped>(\
  "/landshark/gps_meters", 1);
  ros::Publisher pubGpsMetersClean;
  pubGpsMetersClean = nh.advertise<geometry_msgs::PointStamped>(\
  "/landshark/gps_meters_clean", 1);
  
  gpsMeters.point.x = 0.0;
  gpsMeters.point.y = 0.0;
  gpsMeters.point.z = 0.0; 
  
  gpsMetersClean.point.x = 0.0;
  gpsMetersClean.point.y = 0.0;
  gpsMetersClean.point.z = 0.0;
  
  gpsMetersBias.point.x = 0.0;
  gpsMetersBias.point.y = 0.0;
  gpsMetersBias.point.z = 0.0;
  
//--------------------------------------------------------------------------------
  paramName = "origin_lat";
  if( !nh.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",
		 paramName.c_str(),origin_lat);}
	 else nh.getParam(paramName,origin_lat);
	 
	 paramName = "origin_lon";
  if( !nh.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",
		 paramName.c_str(),origin_lon);}
	 else nh.getParam(paramName,origin_lon);
	 
	 paramName = "gps_altitude";
  if( !nh.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",
		 paramName.c_str(),gps_altitude);}
	 else nh.getParam(paramName,gps_altitude);
  
  ROS_INFO("\ngps_drift_server::Initialize \n");
  gpsProjection.SetLocalCoordinateSystemOrigin(origin_lon, origin_lat);  
//--------------------------------------------------------------------------------

//sleep so gps topic gets published to
ros::Duration(1.0).sleep();
  
ROS_INFO("\nready to drift data: \n");
    
  ros::Rate rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    ros::ServiceServer service =\
    nh.advertiseService("/cmu_gps_drift/set_gps_drift",gpsDriftCallback);
    
    try
    {
//      ROS_INFO("initial: drift_x= %f drift_y=%f drift_z=%f drift_time=%f",\
//      driftInfo.drift_x, driftInfo.drift_y, driftInfo.drift_z, driftInfo.drift_time);
//      printf("\nmsgGps.longitude = %f \n", msgGps.longitude);
//      printf("\nmsgGps.latitude = %f \n", msgGps.latitude);
      
      gpsProjection.GetLocalCoordinatesFromGps(\
      msgGps.longitude, msgGps.latitude, gps_x, gps_y);
      
      gpsMeters.point.x = gps_x + reqGpsDrift.drift_x;// - gps_x_origin;
      gpsMeters.point.y = gps_y + reqGpsDrift.drift_y;// - gps_y_origin;
      gpsMeters.point.z = gps_altitude + reqGpsDrift.drift_z; 
      
      gpsMetersClean.point.x = gps_x;
      gpsMetersClean.point.y = gps_y;
      gpsMetersClean.point.z = gps_altitude;
      
      gpsMetersBias.point.x = reqGpsDrift.drift_x;
      gpsMetersBias.point.y = reqGpsDrift.drift_y;
      gpsMetersBias.point.z = reqGpsDrift.drift_z;
      
      pubGpsMeters.publish(gpsMeters);
      pubGpsMetersBias.publish(gpsMetersBias);
      pubGpsMetersClean.publish(gpsMetersClean);      
    }
    catch (std::exception& exception)
    {
      ROS_INFO("\nlat/lon point too far\n");
    }  
    rate.sleep();
  }
//  ros::spin();
  return 0;
}

