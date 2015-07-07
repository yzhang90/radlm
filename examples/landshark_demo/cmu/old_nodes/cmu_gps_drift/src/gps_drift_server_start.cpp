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

int main(int argc, char **argv)
{

  ros::init(argc, argv, "cmu_gps_drift_server");
  ros::NodeHandle n;
  gpsDriftServer drifter(n);
  ros::ServiceServer service = n.advertiseService("/landshark/cmu_gps_drift/set_gps_drift", &gpsDriftServer::set_drift_parameters,
                                                  &drifter);
  ros::spin();
  return 0;
}

