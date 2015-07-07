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
 * \file    obstacleAvoidanceEstimates.h
 * \brief   performs the obstacle avoiding task and planning
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================

#include "EnvironmentVariables.h"

//using namespace std;

class obstacleAvoidanceEst
{
public:
  environmentVariables eV;
  ros::NodeHandle nodeHandle;

  obstacleAvoidanceEst();
  obstacleAvoidanceEst(ros::NodeHandle nh);
  void initClass(ros::NodeHandle nh);
  void predictLocation(float currx, float curry, float currAngle, float v, float w, float &newx, float &newy,
                       float& error);
  void dwaToTargetObstacles();
  void goToTargetAvoidObstacles();

};
