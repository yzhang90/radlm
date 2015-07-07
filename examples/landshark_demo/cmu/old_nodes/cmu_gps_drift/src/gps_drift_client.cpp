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
 * \file    gps_drift_client.h
 * \brief   client for the set gps drift service
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================
#include "ros/ros.h"
#include "cmu_gps_drift/gpsDrift.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_drift_client");
  if (argc != 5 && argc != 4)
  {
    ROS_INFO("usage(1): gps_drift_client DriftX DriftY DriftZ DriftT\nusage(2): gps_drift_client DriftX DriftY DriftT");
    return 1;
  }
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<cmu_gps_drift::gpsDrift>("/landshark/cmu_gps_drift/set_gps_drift");
  cmu_gps_drift::gpsDrift srv;
  if (argc == 4)
  {
    srv.request.drift_x = atoll(argv[1]);
    srv.request.drift_y = atoll(argv[2]);
    srv.request.drift_z = 0;
    srv.request.drift_time = atoll(argv[3]);
  }
  else
  {
    if (argc == 5)
    {
      srv.request.drift_x = atoll(argv[1]);
      srv.request.drift_y = atoll(argv[2]);
      srv.request.drift_z = atoll(argv[3]);
      srv.request.drift_time = atoll(argv[4]);
    }
  }
  if (client.call(srv))
  {
    ROS_INFO("Status : %d", srv.response.err_code);
  }
  else
  {
    ROS_ERROR("Failed to call service gps drift");
    return 1;
  }
  return 0;
}

