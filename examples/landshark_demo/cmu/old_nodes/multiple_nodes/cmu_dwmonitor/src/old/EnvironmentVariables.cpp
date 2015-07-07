/*
 *      EnvironmentVariables.cpp
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */
#include "EnvironmentVariables.h"

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

