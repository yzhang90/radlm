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
 * \brief   Detects significant differences between gps and odometry.
 * \author  Juan Pablo Mendoza
 */
//========================================================================
#include "multi_window_motion_gps_odo_anomaly_monitor.h"
#include <stdio.h>
#include <math.h>

pthread_mutex_t MotionGpsOdoAnomalyMonitor::msgMutex = 
  PTHREAD_MUTEX_INITIALIZER;


void MotionGpsOdoAnomalyMonitor::init(ros::NodeHandle* n) {
  bool debug = false;
  if(debug) printf("Initializing cmu_z_test_monitor\n");
  nOdo = 0;
  nGps = 0;
  nOdoSynth = 0;
  odometrySubscriber = n->subscribe("landshark/odom",1,
      &MotionGpsOdoAnomalyMonitor::odometryCallback,this);
  gpsSubscriber = n->subscribe("landshark/gps_meters",1,
      &MotionGpsOdoAnomalyMonitor::gpsCallback,this);

  AnomalyMonitor::init(n);
  if(debug) printf("cmu_z_test_monitor initialized successfully\n");
  
}

void MotionGpsOdoAnomalyMonitor::gpsCallback(const
//    sensor_msgs::NavSatFixConstPtr& msg) {
    geometry_msgs::PointStampedConstPtr& msg) {
  bool debug = false;
  if(debug) printf("GPS msg timestamp= %4.3f\n",msg->header.stamp.toSec());
  //if(isnan(msg->altitude)) return;
  pthread_mutex_lock(&msgMutex);
  oldGpsMsg = gpsMsg;
  gpsMsg = msg;
  nGps++;
  hasNewGps = true;
  pthread_mutex_unlock(&msgMutex);
}

void MotionGpsOdoAnomalyMonitor::odometryCallback(const
    nav_msgs::OdometryConstPtr& msg) {
  bool debug = false;
  if(debug) printf("Odo Msg timestamp= %4.3f\n",msg->header.stamp.toSec());
  pthread_mutex_lock(&msgMutex);
  oldOdoMsg = odoMsg;
  odoMsg = msg;
  nOdo++;
  //if there is a new gps, create a synthetic message
  if(hasNewGps){
    hasNewGps = false;
    if(nOdo>1){//only possible to interpolate with at least two odometry messages
      oldGpsTOdoMessage = gpsTOdoMessage;
      nOdoSynth ++;
      
      double t0,t1,tGps,interp0,interp1;
      tGps = gpsMsg->header.stamp.toSec();
      t0 = oldOdoMsg->header.stamp.toSec();
      t1 = odoMsg->header.stamp.toSec();
      interp1 = (tGps-t0)/(t1-t0);
      interp0 = (t1-tGps)/(t1-t0);

      gpsTOdoMessage.header.stamp = gpsMsg->header.stamp;
      gpsTOdoMessage.pose.pose.position.x =
          oldOdoMsg->pose.pose.position.x*interp0 +
          odoMsg->pose.pose.position.x*interp1;
      gpsTOdoMessage.pose.pose.position.y =
          oldOdoMsg->pose.pose.position.y*interp0 +
          odoMsg->pose.pose.position.y*interp1;
      gpsTOdoMessage.pose.pose.position.z =
          oldOdoMsg->pose.pose.position.z*interp0 +
          odoMsg->pose.pose.position.z*interp1;
    }
    //update the model if there is enough info
    if(nOdoSynth>=2) updateModel();
  }
  pthread_mutex_unlock(&msgMutex);
}

void MotionGpsOdoAnomalyMonitor::updateModel() {
  bool debug = false;
  if(debug) printf("update called\n");

  //displacements
  geometry_msgs::Point odoP0 = oldGpsTOdoMessage.pose.pose.position;
  geometry_msgs::Point odoP1 = gpsTOdoMessage.pose.pose.position;
  double odoD = sqrt(sq(odoP0.x-odoP1.x)+sq(odoP0.y-odoP1.y));
      //+sq(odoP0.z-odoP1.z));

  geometry_msgs::Point gpsP0 = oldGpsMsg->point;
  geometry_msgs::Point gpsP1 = gpsMsg->point;
   double gpsD = sqrt(sq(gpsP0.x-gpsP1.x)+sq(gpsP0.y-gpsP1.y));
       //+sq(gpsP0.z-gpsP1.z));
  //double gpsD = getGpsDisplacement(oldGpsMsg.latitude,oldGpsMsg.longitude,
  //    oldGpsMsg.altitude,gpsMsg.latitude,gpsMsg.longitude,gpsMsg.altitude);
  
  state s;
  s.s.push_back(gpsMsg->header.stamp.toSec());
  
  //s.s.push_back(getGpsDisplacement(0.0,0.0,0.0,0.0,gpsMsg.longitude,0.0));
  //s.s.push_back(getGpsDisplacement(0.0,0.0,0.0,gpsMsg.latitude,0.0,0.0));
  //s.s.push_back(gpsMsg.altitude);

  s.s.push_back(gpsMsg->point.x);
  s.s.push_back(gpsMsg->point.y);
  s.s.push_back(gpsMsg->point.z);
  std::vector<double> residual;
  residual.push_back(odoD-gpsD);
  //printf("odometry:%f, gps:%f, residual:%f\n",odoD,gpsD,odoD-gpsD);
  stateStat r;
  r.n=1;
  r.mean=residual;
  for (unsigned int i=0;i<residual.size();i++){
    r.ex2.push_back(residual[i]*residual[i]);
    r.variance.push_back(variance); //not using sample variance
  }
  addDataPoint(s,r);
}

float MotionGpsOdoAnomalyMonitor::getGpsDisplacement(float lat0, float lon0,
    float z0,float lat1, float lon1, float z1)
{
  if(lat0==lat1 && lon0==lon1 && z0==z1) return 0.0;
  const float EarthR = 6371000.0; //m
  float dLat = (lat1-lat0)*M_PI/180.0;
  float dLon = (lon1-lon0)*M_PI/180.0;
  float lat0R = lat0*M_PI/180.0;
  float lat1R = lat1*M_PI/180.0;
  float a = sq(sin(dLat/2.0)) + sq(sin(dLon/2.0))*cos(lat0R)*cos(lat1R);
  float dxy = 2.0*EarthR*atan2(sqrt(a),sqrt(1-a));
//   printf("lat0:%f,lat1:%f,dLat:%f,dLon:%f,a:%f,dxy:%f,z0:%f,z1:%f\n",
//       lat0R,lat1R,dLat,dLon,a,dxy,z0,z1);
  return sqrt(sq(dxy)+sq(z1-z0));
}
