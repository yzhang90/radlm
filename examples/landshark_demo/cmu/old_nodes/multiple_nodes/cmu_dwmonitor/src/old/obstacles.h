/*
 *      obstacles.h
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */

#ifndef OBSTACLES_H_
#define OBSTACLES_H_
#include <map>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
using namespace std;

class environmentObstacles
{
public:
    map<string, geometry_msgs::Point> obstacles;
    int obsCount;

public:
    environmentObstacles();
    void initClass();
    //initialize obstacles function
    void initializeObstacleTag(float x, float y, float z, string tag);
    void initializeObstacle(float x, float y, float z);
    //initialize multiple obstacles
    void initializeMultipleObstacles(vector<float> obstaclesPoints, vector<string> tags);
    //remove obstacles
    void removeObstacle(string tag);
    // find the distance between two point stamped points disregarding the z values
    float distanceBetweenPointStampedNoZ(geometry_msgs::PointStamped p1, geometry_msgs::PointStamped p2);
    // find the distance between two point stamped points disregarding the z values
    float distanceBetweenPointStampedNoZ(geometry_msgs::Point p1, geometry_msgs::Point p2);
    // find the distance between two point stamped points disregarding the z values
    float distanceBetweenPointStampedNoZ(geometry_msgs::PointStamped p1, geometry_msgs::Point p2);
    float distanceBetweenXYPoints(float x1, float y1, float z1, float x2, float y2, float z2);
    // find distance given x y only
    float distanceBetweenXYPoints(float x1, float y1, float x2, float y2);
    // find the closest obstacle straight line distance
    float findClosestObstacleLine(float robot_x, float robot_y, float& obs_x, float& obs_y, string& obsNo);
    //remap obstacles
    void updateObstacleLocations(float robot_x_base, float robot_y_base, float robot_x_new, float robot_y_new);

};

#endif /* OBSTACLES_H_ */
