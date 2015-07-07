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
 * \file    EnvironmentVariables.cpp
 * \brief   sets environment (robot and surrounding) variables
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================

#include "EnvironmentVariables.h"
#include "readGPSCoordinateFile.h"

environmentVariables::environmentVariables()
{

}
environmentVariables::environmentVariables(ros::NodeHandle nh)
{
  nodeHandle = nh;
  eS.initClass(nodeHandle);

}

void environmentVariables::initClass(ros::NodeHandle nh)
{
  nodeHandle = nh;
  eS.initClass(nodeHandle);

}
void environmentVariables::initConstants()
{
  eS.setVmax(1);
  eS.setWmax(1);
  eS.setDt(0.25);
  eS.setAccs(4, 4);
  eS.setDmax(10);
}
void environmentVariables::initWall(float x1, float y1, bool xorientation)
{
  float wallDist = 5;
  initWall(x1, y1, wallDist, xorientation);
}
void environmentVariables::initWall(float x1, float y1, float wallDist, bool xorientation)
{
  float wallInc = 0.5;

  if (xorientation)
  {
//    printf("\nWall Params %f %f, wallDist %f", x1, y1, wallDist);
//    printf("\nRange %f %f with %f constant", x1 - wallDist, x1 + wallDist, y1);
//    cin.get();
    for (float wall = x1 - wallDist; wall <= x1 + wallDist; wall += wallInc)
    {
      eS.initializeObstacle(wall, y1, 0);
    }
  }
  else
  {
    for (float wall = y1 - wallDist; wall <= y1 + wallDist; wall += wallInc)
    {
      eS.initializeObstacle(x1, wall, 0);
    }

  }
}
void environmentVariables::initCorridor(float x1, float y1, float x2, float y2, float offsetMult)
{
  float wallDist = 5;
  initCorridor(x1, y1, x2, y2, offsetMult, wallDist);
}
void environmentVariables::initCorridor(float x1, float y1, float x2, float y2, float offsetMult, float wallDist)
{

  float wallInc = 0.5;
  // float wallDist = 5;
  float wallOffset = -10 * offsetMult;
  float walls = 0;
  float walle = 0;
  float wallsConstant;
  bool isX;

  if (fabs(x1 - x2) < 2)
  {
    isX = true;
    wallsConstant = x1;
    if (y1 < y2)
    {
      walls = y1;
      walle = y2;
    }
    else
    {
      walls = y2;
      walle = y1;
    }

//    printf("\nCorridor Params %f %f, wallDist %f", walls, walle, wallDist);
//    printf("\nRange %f %f with %f constant", walls - wallDist, walle + wallDist, wallsConstant);
//    printf("\nRange %f %f with %f constant", walls - wallDist, walle + wallDist, wallsConstant + wallOffset);
//    cin.get();

    for (float wall = walls - wallDist; wall <= walle + wallDist; wall += wallInc)
    {
      eS.initializeObstacle(wallsConstant, wall, 0);
      printf("-");
      eS.initializeObstacle(wallsConstant + wallOffset, wall, 0);
    }

  }
  else
  {
    isX = false;
    wallsConstant = y1;
    if (x1 < x2)
    {
      walls = x1;
      walle = x2;
    }
    else
    {
      walls = x2;
      walle = x1;
    }

//    printf("\nCorridor Params %f %f, wallDist %f", walls, walle, wallDist);
//    printf("\nRange %f %f with %f constant", walls - wallDist, walle + wallDist, wallsConstant);
//    printf("\nRange %f %f with %f constant", walls - wallDist, walle + wallDist, wallsConstant + wallOffset);
//    cin.get();

    for (float wall = walls - wallDist; wall <= walle + wallDist; wall += wallInc)
    {
      eS.initializeObstacle(wall, wallsConstant, 0);
      printf("*");
      eS.initializeObstacle(wall, wallsConstant + wallOffset, 0);
    }

  }

}


void environmentVariables::readFromGPSFile(string fileName)
{
  initConstants();
  readGPSCoordinateFile gpsReader;
  cout << "\nTrying to read a file with gps coordinates";
  gpsReader.readFile(fileName);
  cout << "\nRead the file";
  for (int i = 0; i < gpsReader.coordinates.size(); i += 4)
  {
    if (gpsReader.coordinates[i + 3] == 1)
      eS.initializeTarget(gpsReader.coordinates[i], gpsReader.coordinates[i + 1], gpsReader.coordinates[i + 2]);
    else if (gpsReader.coordinates[i + 3] == 0)
      eS.initializeObstacle(gpsReader.coordinates[i], gpsReader.coordinates[i + 1], gpsReader.coordinates[i + 2]);
    printf("\n i: %d  tag %f - %f %f %f", i, gpsReader.coordinates[i + 3], gpsReader.coordinates[i],
           gpsReader.coordinates[i + 1], gpsReader.coordinates[i + 2]);
  }
}
