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
 * \file    runLocalPlanner.cpp
 * \brief   calls local planner 
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================


#include "obstacleAvoidanceEstimates.h"
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv)
{
  /// Initialize the ros node
  ros::init(argc, argv, "dynamicWindowApproachTemp");
  ros::NodeHandle nodeHandle;
  ros::Rate rate(1000);
  obstacleAvoidanceEst dwa(nodeHandle);
  bool start = false;
  string filePath = "/home/fatma/ros_groovy_base/CMU/src/landshark_cmu_dwmonitorIntegration/src/gpsFile.txt"; 
  if(nodeHandle.hasParam("gps_file_pathname"))
   nodeHandle.getParam("gps_file_pathname",filePath);
  while (ros::ok())
  {
    ros::spinOnce();
    if (!start) //&& dwa.robotFunctions.gpsPosition.point.x != 0)
    {
      ros::spinOnce();
      dwa.eV.eS.robotFunctions.convertGPS=true; 
      dwa.eV.eS.robotFunctions.gpsMode = true;
      string fileName = filePath;
      dwa.eV.readFromGPSFile(fileName);
      dwa.goToTargetAvoidObstacles();
      start = true;
    }

    rate.sleep();

  }

  return 0;

}

