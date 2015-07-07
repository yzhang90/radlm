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
 * \file    environmentVariables.h
 * \brief   set environment variables
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================

#include "environmentSpecifics.h"

extern "C"
{
#include "dwmonitor.h"
}

using namespace std;

class environmentVariables
{
public:
  environmentSpecifics eS;
  ros::NodeHandle nodeHandle;

  environmentVariables();
  environmentVariables(ros::NodeHandle nh);
  void initClass(ros::NodeHandle nh);
  void initConstants();
  void initWall(float x1, float y1, bool xorientation = true);
  void initWall(float x1, float y1, float wallDist, bool xorientation = true);
  void initCorridor(float x1, float y1, float x2, float y2, float offsetMult);
  void initCorridor(float x1, float y1, float x2, float y2, float offsetMult, float wallDist);
  void readFromGPSFile(string fileName);

};
