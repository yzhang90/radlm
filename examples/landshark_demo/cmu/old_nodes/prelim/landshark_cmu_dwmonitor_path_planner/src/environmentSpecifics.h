
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
 * \file    environmentSpecifics.h
 * \brief   
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================


#ifndef environmentSpecifics_H_
#define environmentSpecifics_H_
#include <cmu_ros_pub_sub/getRosParameters.h>
#include <vector>
#include <ros/ros.h>

class environmentSpecifics
{
public:
  geometry_msgs::PointStamped target;
  vector<geometry_msgs::PointStamped> obstacles;
  getROSParameters robotFunctions;
  float vmax;
  float wmax;
  float timePeriod;
  float accv;
  float accw;
  float dmax;
  ros::NodeHandle nodeHandle;

  environmentSpecifics(ros::NodeHandle nh);
  environmentSpecifics();
  void initClass(ros::NodeHandle nh);
  void setDmax(float d);
  void setVmax(float v);
  void setWmax(float w);
  void setDt(float dt);
  void setAccs(float v, float w);
  //initialize target function
  void initializeTarget(float x, float y, float z);
  //initialize obstacles function
  void initializeObstacle(float x, float y, float z);
  //initialize multiple obstacles
  void initializeMultipleObstacles(vector<float> obstaclesPoints);
  // find the distance between two point stamped points disregarding the z values
  float distanceBetweenPointStampedNoZ(geometry_msgs::PointStamped p1, geometry_msgs::PointStamped p2);
  // find distance given x y only
  float distanceBetweenXYPoints(float x1, float y1, float x2, float y2);
  // find the closest obstacle straight line distance
  float findClosestObstacleLine(float robot_x, float robot_y, float& obs_x, float& obs_y, int& obsNo);
  // generate velocities within a given range given that we can accelerate at x m/s2
  void generateVelocityRange(float dt, float currv, float currw, float accv, float accw, float vmax, float wmax,
                             float& maxv, float& minv, float& maxw, float& minw);
  void generateVelocityRange(float currv, float currw, float& maxv, float& minv, float& maxw, float& minw);

  float wrapAngle(float angle);
  void predictLocation(float currx, float curry, float currAngle, float v, float w, float &newx, float &newy,
                       float& error);
};

#endif /* environmentSpecifics_H_ */
