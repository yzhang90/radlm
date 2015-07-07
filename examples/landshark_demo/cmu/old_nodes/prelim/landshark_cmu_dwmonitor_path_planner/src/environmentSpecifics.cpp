
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
 * \file    environmentSpecifics.cpp
 * \brief   
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================

#include "environmentSpecifics.h"
#include <ros/ros.h>

environmentSpecifics::environmentSpecifics(ros::NodeHandle nh)
  {
    nodeHandle = nh;
    robotFunctions.initClass(nodeHandle);
  }
environmentSpecifics::environmentSpecifics()
{

}
  void environmentSpecifics::initClass(ros::NodeHandle nh){
    nodeHandle = nh;
    robotFunctions.initClass(nodeHandle);
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
  //initialize target function
  void environmentSpecifics::initializeTarget(float x, float y, float z)
  {
    target.point.x = x;
    target.point.y = y;
    target.point.z = z;
  }
  //initialize obstacles function
  void environmentSpecifics::initializeObstacle(float x, float y, float z)
  {
    geometry_msgs::PointStamped obs;
    obs.point.x = x;
    obs.point.y = y;
    obs.point.z = z;
    obstacles.push_back(obs);
  }
  //initialize multiple obstacles
  void environmentSpecifics::initializeMultipleObstacles(vector<float> obstaclesPoints)
  {
    for (int i = 0; i < obstaclesPoints.size() - 2; i += 3)
    {
      initializeObstacle(obstaclesPoints[i], obstaclesPoints[i + 1], obstaclesPoints[i + 2]);
    }
  }
  // find the distance between two point stamped points disregarding the z values
  float environmentSpecifics::distanceBetweenPointStampedNoZ(geometry_msgs::PointStamped p1, geometry_msgs::PointStamped p2)
  {
    return distanceBetweenXYPoints(p1.point.x, p1.point.y, p2.point.x, p2.point.y);
  }
  // find distance given x y only
  float environmentSpecifics::distanceBetweenXYPoints(float x1, float y1, float x2, float y2)
  {
    return robotFunctions.getDistanceBetweenPoints3(x1, y1, 0, x2, y2, 0);
  }
  // find the closest obstacle straight line distance
  float environmentSpecifics::findClosestObstacleLine(float robot_x, float robot_y, float& obs_x, float& obs_y, int& obsNo)
  {
    float maxDist = 900000;
    float minObsDist = maxDist;
    float obsDist = 0;
    float tempObsx = 0;
    float tempObsy = 0;

    for (int i = 0; i < obstacles.size(); i++)
    {
      tempObsx = obstacles[i].point.x;
      tempObsy = obstacles[i].point.y;
     // cout<<"obs"<<" "<<tempObsx<<" "<<tempObsy<<endl;
      obsDist = distanceBetweenXYPoints(robot_x, robot_y, tempObsx, tempObsy);
      if (obsDist < minObsDist)
      {
        minObsDist = obsDist;
        obs_x = tempObsx;
        obs_y = tempObsy;
        obsNo = i;
      }
    }
    minObsDist = min(dmax, minObsDist);
   // cout<<"minDist "<<obsDist<<" obsno "<<obsNo<<endl;
    return minObsDist;
  }
  // generate velocities within a given range given that we can accelerate at x m/s2
  void environmentSpecifics::generateVelocityRange(float dt, float currv, float currw, float accv, float accw, float vmax, float wmax,
                             float& maxv, float& minv, float& maxw, float& minw)
  {
    maxv = currv + accv * dt;
    minv = currv - accv * dt;
    maxv = fmin(maxv, vmax);
    minv = fmax(minv, -vmax);

    maxw = currw + accw * dt;
    minw = currw - accw * dt;
    maxw = fmin(maxw, wmax);
    minw = fmax(minw, -wmax);
  }
  void environmentSpecifics::generateVelocityRange(float currv, float currw, float& maxv, float& minv, float& maxw, float& minw)
  {
    generateVelocityRange(timePeriod, currv, currw, accv, accw, vmax, wmax, maxv, minv, maxw, minw);
    minv = fmax(0, minv);
    maxv = fmax(0, maxv);
  }

  float environmentSpecifics::wrapAngle(float angle)
  {
    float pi = 3.142;
    float pi2 = 2 * pi;
    float angToRet;
    while (angle >= pi)
    {
      angle -= pi2;
    }
    while (angle < -pi)
    {
      angle += pi2;
    }
    angToRet = angle;
    return angToRet;

  }




  void environmentSpecifics::predictLocation(float currx, float curry, float currAngle, float v, float w, float &newx, float &newy,
                         float& error)
    {
      float dx = 0;
      float dy = 0;
      float dt = timePeriod;
      if (w == 0)
      {
        dx = currx + v * cos(currAngle) * dt;
        dy = curry + v * sin(currAngle) * dt;
      }
      else
      {
        float r = (v / w);
        float robotR = 0.35;
        if (r > 0)
        {
          r = r + robotR;
        }
        else
        {
          r = r - robotR;
        }
        float xc = currx - r * sin(currAngle);
        float yc = curry + r * cos(currAngle);
        dx = xc + r * sin(currAngle + w * dt);
        dy = yc - r * cos(currAngle + w * dt);
      }
      newx = dx;
      newy = dy;
      error = 0.3;

    }
