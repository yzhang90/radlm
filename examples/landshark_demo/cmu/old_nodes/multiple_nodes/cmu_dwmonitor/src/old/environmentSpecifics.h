/*
 * environmentSpecifics.h
 *
 *      Author: Fatma Faruq
 *	HACMS CMU Team, Advisor Manuela Veloso
 */

#ifndef environmentSpecifics_H_
#define environmentSpecifics_H_
#include <landshark_cmu_ros_pub_sub/getRosParameters.h>
#include "obstacles.h"
#include <vector>
#include <map>
#include <string>
using namespace std;

/*Setting Environment Specific Data
 *The maximum speed of the robot
 *
 */
class environmentSpecifics
{
public:
    geometry_msgs::PointStamped target;
    getROSParameters robotFunctions;
    environmentObstacles obstacles;
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
    void initializeObstacle(float x, float y, float z);
    float findClosestObstacleLine(float robot_x, float robot_y, float& obs_x, float& obs_y, string& obsNo);
};

#endif /* environmentSpecifics_H_ */
