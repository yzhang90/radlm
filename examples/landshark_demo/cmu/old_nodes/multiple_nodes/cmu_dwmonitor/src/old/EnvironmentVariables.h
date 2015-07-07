/*
 *      EnvironmentVariables.h
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */

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


};
