/*
 *      teleopFilterCheck.h
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */

#ifndef teleopfiltercheck_H_
#define teleopfiltercheck_H_
#include "dwmonitor.h"
#include "environmentSpecifics.h"
#include "landshark_cmu_dwmonitor/obstacleMsg.h"
#include "convertToGPSClass.h"

using namespace std;

class teleopFilterCheck
{
private:
   ros::NodeHandle nodeHandle;
    bool hasObstacle;
    ros::Subscriber obsSub;
    std_msgs::Header lastObsHeader;
    convertToGPSClass gpsConverter;
    bool obstaclesConverted;
    bool readObsFromTop;
    float clearance1; 
    float clearance2;
    float maxObsVel;

    void readObstacleFromTopic();
    void obstacleCallback(const landshark_cmu_dwmonitor::obstacleMsg::ConstPtr& pMsg);

public:
    environmentSpecifics es;
    bool gpsMode;
    teleopFilterCheck(bool gps,ros::NodeHandle nh, bool obstop = true,float c1=1.5, float c2 = 2.5);
    teleopFilterCheck();
    void initClass(bool gps, ros::NodeHandle nh,bool obstop = true, float c1 = 1.5, float c2 = 2.5);
    void initConstants(float vmax_robot=1, float wmax_robot=1, float eps=0.25,float maxAccLin=4, float maxAccAng=4, float maxDist=10,float maxOVel = 1);
    void initGPSConverter(float lat,float lon);
    bool okayToGo(float v, float w);
};
#endif
