/*
 * environmentSpecifics.cpp
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */
#include "environmentSpecifics.h"
#include <sstream>

environmentSpecifics::environmentSpecifics(ros::NodeHandle nh)
{
    nodeHandle = nh;
    robotFunctions.initClass(nodeHandle);
    obstacles.initClass();
}
environmentSpecifics::environmentSpecifics()
{

}
void environmentSpecifics::initClass(ros::NodeHandle nh)
{
    nodeHandle = nh;
    robotFunctions.initClass(nodeHandle);
    obstacles.initClass();
}
void environmentSpecifics::setDmax(float d)
{
    dmax = d;
}
void environmentSpecifics::setVmax(float v)
{
    vmax = v;
}
void environmentSpecifics::setWmax(float w)
{
    wmax = w;
}
void environmentSpecifics::setDt(float dt)
{
    timePeriod = dt;
}
void environmentSpecifics::setAccs(float v, float w)
{
    accv = v;
    accw = w;
}
void environmentSpecifics::initializeObstacle(float x, float y, float z)
{
    obstacles.initializeObstacle(x, y, z);
}
float environmentSpecifics::findClosestObstacleLine(float robot_x, float robot_y, float& obs_x, float& obs_y,
                                                    string& obsNo)
{
  float minObsDist = obstacles.findClosestObstacleLine(robot_x, robot_y, obs_x, obs_y, obsNo);
  return min(dmax, minObsDist);
}

