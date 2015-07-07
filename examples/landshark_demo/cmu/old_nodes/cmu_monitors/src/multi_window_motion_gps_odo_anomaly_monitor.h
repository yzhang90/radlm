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
 * \file    motion_gps_odo_anomaly_monitor.cpp
 * \brief   Detects significant differences between gps and odom displacements
 * \author  Juan Pablo Mendoza
 */
//========================================================================

#include "multi_window_anomaly_monitor.h"
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <landshark_gps/GpsProjection.h>

#ifndef MOTION_GPS_ODO_ANOMALY_MONITOR
#define MOTION_GPS_ODO_ANOMALY_MONITOR


/* Finds abnormal behavior in the motion of the robot. In particular,
 * it looks at the difference between commanded and measured velocities
 * of the robot, and decides, based on aggregation of data over multiple
 * scales of time, whether motion execution is normal or not.
 */
class MotionGpsOdoAnomalyMonitor: public AnomalyMonitor{
private:
  //subscribers for necessary topics for motion execution
  ros::Subscriber odometrySubscriber;
  ros::Subscriber gpsSubscriber;
  //avoid conflicts in data access
  static pthread_mutex_t msgMutex;
  //last received odometry message
  nav_msgs::OdometryConstPtr odoMsg;
  //old odometry message
  nav_msgs::OdometryConstPtr oldOdoMsg;
  //synthetic odo message synced up to oldGpsMsg
  nav_msgs::Odometry oldGpsTOdoMessage;
  //synthetic odo message synced up to gpsMsg
  nav_msgs::Odometry gpsTOdoMessage;
  
//ME  
//  std::vector<double> residual;
//ME

  //last received location message
  landshark::GpsProjection gpsProjection;
  sensor_msgs::NavSatFixConstPtr gpsMsgLatLon;
  
//JP  geometry_msgs::PointStampedConstPtr gpsMsg;
  sensor_msgs::NavSatFixConstPtr gpsMsg;
  //old location message
  //sensor_msgs::NavSatFix oldGpsMsg;
//JP  geometry_msgs::PointStampedConstPtr oldGpsMsg;
  sensor_msgs::NavSatFixConstPtr oldGpsMsg;
  //number of received odometry messages
  unsigned int nOdo;
  //number of received gps messages
  unsigned int nGps;
  //number of synthetic odo messages
  unsigned int nOdoSynth;
  //indicates a gps msg was received but the model is not yet updated
  bool hasNewGps;
  
  double robot_meters_x;
  double robot_meters_y;
  double robot_meters_x_old;
  double robot_meters_y_old;  

  //callback functions for different messages.
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void gpsCallback(const sensor_msgs::NavSatFixConstPtr& msg);
  //void gpsCallback(const geometry_msgs::PointStampedConstPtr& msg);
  //adds latest data point to the anomaly monitor data structure
  void updateModel();
  
  double origin_lon;
  double origin_lat;

  double sq(double x){return x*x;};
  
public:
  void init(ros::NodeHandle *n, double origin_lon, double origin_lat);
private:
  float getGpsDisplacement(float lat0,float lon0, float z0, float lat1,
                           float lon1, float z1);
};

#endif
