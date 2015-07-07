/*
 *      obstacles.cpp
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */

#include "obstacles.h"
#include <sstream>

environmentObstacles::environmentObstacles()
{
    obsCount = 0;
}
void environmentObstacles::initClass()
{
    obsCount = 0;
}
//initialize obstacles function
void environmentObstacles::initializeObstacleTag(float x, float y, float z, string tag)
{
    geometry_msgs::Point obs;
    obs.x = x;
    obs.y = y;
    obs.z = z;
    obstacles[tag] = obs;
    //obstacles.push_back(obs);
}
void environmentObstacles::initializeObstacle(float x, float y, float z)
{
    stringstream ss;
    ss << obsCount;
    string obsTag(ss.str());
    initializeObstacleTag(x, y, z, obsTag);
    obsCount++;
}
//remove obstacles
void environmentObstacles::removeObstacle(string tag)
{
    obstacles.erase(tag);
}
//initialize multiple obstacles
void environmentObstacles::initializeMultipleObstacles(vector<float> obstaclesPoints, vector<string> tags)
{
    int obsCount = 0;
    for (int i = 0; i < obstaclesPoints.size() - 2; i += 3)
    {
        initializeObstacleTag(obstaclesPoints[i], obstaclesPoints[i + 1], obstaclesPoints[i + 2], tags[obsCount]);
        obsCount++;
    }
}
// find the distance between two point stamped points disregarding the z values
float environmentObstacles::distanceBetweenPointStampedNoZ(geometry_msgs::PointStamped p1,
        geometry_msgs::PointStamped p2)
{
    return distanceBetweenXYPoints(p1.point.x, p1.point.y, p2.point.x, p2.point.y);
}
// find the distance between two point stamped points disregarding the z values
float environmentObstacles::distanceBetweenPointStampedNoZ(geometry_msgs::PointStamped p1, geometry_msgs::Point p2)
{
    return distanceBetweenXYPoints(p1.point.x, p1.point.y, p2.x, p2.y);
}
// find the distance between two point stamped points disregarding the z values
float environmentObstacles::distanceBetweenPointStampedNoZ(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return distanceBetweenXYPoints(p1.x, p1.y, p2.x, p2.y);
}
// find distance given x y z
float environmentObstacles::distanceBetweenXYPoints(float x1, float y1, float z1, float x2, float y2, float z2)
{

    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    float dx2 = dx * dx;
    float dy2 = dy * dy;
    float dz2 = dz * dz;
    float distance = sqrt(dx2 + dy2 + dz2);
    return distance;
}
// find distance given x y only
float environmentObstacles::distanceBetweenXYPoints(float x1, float y1, float x2, float y2)
{
    return distanceBetweenXYPoints(x1, y1, 0, x2, y2, 0);
}
// find the closest obstacle straight line distance

float environmentObstacles::findClosestObstacleLine(float robot_x, float robot_y, float& obs_x, float& obs_y,
        string& obsNo)
{
    float maxDist = 10000;
    float minObsDist = maxDist;
    float obsDist = 0;
    float tempObsx = 0;
    float tempObsy = 0;

    for (map<string, geometry_msgs::Point>::iterator it = obstacles.begin(); it != obstacles.end(); it++)
    {
        tempObsx = it->second.x;
        tempObsy = it->second.y;
        obsDist = distanceBetweenXYPoints(robot_x, robot_y, tempObsx, tempObsy);
        if (obsDist < minObsDist)
        {
            minObsDist = obsDist;
            obs_x = tempObsx;
            obs_y = tempObsy;
            obsNo = it->first;
        }
    }
    return minObsDist;
}

void environmentObstacles::updateObstacleLocations(float robot_x_base, float robot_y_base, float robot_x_new,
        float robot_y_new)
{
    float tempObsx = 0;
    float tempObsy = 0;
    float xdiff = 0;
    float ydiff = 0;

    for (map<string, geometry_msgs::Point>::iterator it = obstacles.begin(); it != obstacles.end(); it++)
    {
        tempObsx = it->second.x;
        tempObsy = it->second.y;
        xdiff = robot_x_base - tempObsx;
        ydiff = robot_y_base - tempObsy;
        it->second.x = robot_x_new + xdiff;
        it->second.y = robot_y_new + ydiff;
    }
}
