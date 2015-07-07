/*
 *      teleopFilterCheck.cpp
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */

#include "teleopFilterCheck.h"
#include "dwmonitor.h"

static const double CMU_lon = -79.945;
static const double CMU_lat = 40.443;
teleopFilterCheck::teleopFilterCheck(bool gps, ros::NodeHandle nh, bool readObsFromTopic,float c1, float c2)
{
    nodeHandle = nh;
    es.initClass(nodeHandle);
    gpsMode = gps;
    obstaclesConverted = false;
    hasObstacle = false;
    if (gpsMode)
    {
        es.robotFunctions.convertGPSToggle();
        es.robotFunctions.gpsMode = true;
    }
    readObsFromTop = readObsFromTopic;
    if(readObsFromTop)
    {
        obsSub = nodeHandle.subscribe<landshark_cmu_dwmonitor::obstacleMsg>("/landshark/dwmonitor/obstacle",1,&teleopFilterCheck::obstacleCallback,this);
    }
    lastObsHeader.stamp = ros::Time::now();
    initConstants();
    gpsConverter.initClass(gps,nh);
    gpsConverter.initGPS(CMU_lat,CMU_lon);
    clearance1 = c1;
    clearance2 = c2;
}

teleopFilterCheck::teleopFilterCheck()
{
    obstaclesConverted = false;
    readObsFromTop = false;
    hasObstacle = false;
    if(readObsFromTop)
    {
        obsSub = nodeHandle.subscribe<landshark_cmu_dwmonitor::obstacleMsg>("/landshark/dwmonitor/obstacle",1,&teleopFilterCheck::obstacleCallback,this);
    }
    lastObsHeader.stamp = ros::Time::now();
    initConstants();
    clearance1 = 1.5;
    clearance2 = 2.5;
}
void teleopFilterCheck::initClass(bool gps,  ros::NodeHandle nh, bool readObsFromTopic,float c1,float c2)
{
    nodeHandle = nh;
    es.initClass(nodeHandle);
    obstaclesConverted = false;
    hasObstacle = false;
    gpsMode = gps;
    if (gpsMode)
    {
        es.robotFunctions.convertGPSToggle();
        es.robotFunctions.gpsMode = true;
    }
    readObsFromTop = readObsFromTopic;
    if(readObsFromTop)
    {
        obsSub = nodeHandle.subscribe<landshark_cmu_dwmonitor::obstacleMsg>("/landshark/dwmonitor/obstacle",1,&teleopFilterCheck::obstacleCallback,this);
    }
    lastObsHeader.stamp = ros::Time::now();
    initConstants();
    gpsConverter.initClass(gps,nh);
    gpsConverter.initGPS(CMU_lat,CMU_lon);
    clearance1 = c1;
    clearance2 = c2;
}


void teleopFilterCheck::initConstants(float vmax_robot, float wmax_robot, float eps,float maxAccLin, float maxAccAng, float maxDist,float maxOVel)
{
    es.setVmax(vmax_robot);
    es.setWmax(wmax_robot);
    es.setDt(eps);
    es.setAccs(maxAccLin, maxAccAng);
    es.setDmax(maxDist);
    maxObsVel = maxOVel;
}

void teleopFilterCheck::obstacleCallback(const landshark_cmu_dwmonitor::obstacleMsg::ConstPtr& pMsg)
{
    bool debug = false;
    if(debug)
    {
        printf("\nIn the obstacle function");
    }
    if(lastObsHeader.stamp != pMsg->coordinates.header.stamp)
    {
        float x = pMsg->coordinates.point.x;
        float y = pMsg->coordinates.point.y;
        float z = pMsg->coordinates.point.z;
        

        if(debug)
        {
            printf("\nIn the obstacle function: passed time check");
        }

        if(pMsg->type == 0)
        {
            if(debug)
            {
                printf("\nIn the obstacle function: init obs");
            }


            es.initializeObstacle(x,y,z);
hasObstacle = true;
            obstaclesConverted= true;

        }
        else if(pMsg->type == 1)
        {
            if(debug)
            {
                printf("\nIn the obstacle function: gps stuff");
                printf("\nRobot in lat lon = %f %f %f",es.robotFunctions.gpsData.latitude, es.robotFunctions.gpsData.longitude, es.robotFunctions.gpsData.altitude);
                printf("\nRobot position in meters %f %f %f ",es.robotFunctions.gpsPosition.point.x,es.robotFunctions.gpsPosition.point.y,es.robotFunctions.gpsPosition.point.z);
            
//cin.get();
}
            float xx=0, yy=0, zz=0;
	    if(debug)
            printf("\nConvert point %f %f %f to %f %f",x,y,z,xx,yy);
            gpsConverter.converPointsToGPS((double)x, (double)y, (double)z);
	    xx = (float)gpsConverter.gpsX; 
	    yy = (float)gpsConverter.gpsY;
if(debug)
 printf("\nConverted point %f %f %f to %f %f",x,y,z,xx,yy);
//	if(xx != 0.0 && yy !=0.0)
//{
            es.initializeObstacle(xx,yy,zz);
            if(debug)
            {
                printf("\nSaving Converted point %f %f %f to %f %f Done \n",x,y,z,xx,yy);
//cin.get();
            }
            obstaclesConverted = true;
hasObstacle = true;
//}

        }
        lastObsHeader.stamp = pMsg->coordinates.header.stamp;
    }
}
void teleopFilterCheck::initGPSConverter(float lat,float lon)
{
	gpsConverter.initGPS(lat,lon);
}
bool teleopFilterCheck::okayToGo(float v, float w)
{
    bool debug = false;
    ros::spinOnce();
    bool okayToGob = false;
    float robot_x = es.robotFunctions.gpsPosition.point.x;
    float robot_y = es.robotFunctions.gpsPosition.point.y;
    float obs_x = 0;
    float obs_y = 0;
    string obsNo = "";
    float distToObs = es.findClosestObstacleLine(robot_x, robot_y, obs_x, obs_y, obsNo);
    if(!hasObstacle)
    {
        //Create a dummy obstacle
        obs_x = robot_x + 20000000;
        obs_y = robot_y + 20000000;
        distToObs = 20000000;
    }
    if(debug)
    {
    printf("\nObs no = %s", obsNo.c_str());
    printf("\n Closest Obstacle \n x %f y %f dist %f\n", obs_x, obs_y, distToObs);
   
    printf("\n Robot Position \n x %f y %f \n", robot_x, robot_y);
    }
    //distToObs = es.findClosestObstacleLineFrontBack(robot_x,robot_y, obs_x, obs_y, obsNo, (v > 0));
    //printf("\n Commanded velocity is positive ? %f then \n", v);
    //printf("\n Closest Obstacle \n x %f y %f dist %f\n", obs_x, obs_y, distToObs);
    //float angleToObs = es.robotFunctions.getHeading2(es.robotFunctions.gpsPosition, es.obstacles.obstacles[obsNo],
    //                  es.robotFunctions.odomAngle, 0, 0);
    float obsWeight = 0;
    double f_V = fabs(v);//f_V = maxObsVel;
    float accv = es.accv;
    float timePeriod = es.timePeriod;
    float X[5];

    X[0] = fabs((float)es.robotFunctions.currentLinearSpeed);
    X[1] = (float)robot_x;
    X[2] = (float)robot_y;
    X[3] = (float)obs_x;
    X[4] = (float)obs_y;
    double extraClearance = clearance1;//1.5;
    double conservativeClearance = clearance2;//2.5;
    double d0 = (double)((accv / accv + 1) * (accv / 2 * timePeriod * timePeriod + f_V * timePeriod));
    double D[3];
    D[0] = d0 + conservativeClearance;
    D[1] = (double)((f_V / accv) + timePeriod * (accv / accv + 1));
    D[2] = (double)(1 / (2 * accv));
//    printf("\n The equation stuff accv %f f_V %f timePerido %f",accv,f_V,timePeriod);
//    printf("\n Equation D: %f %f %f",D[0],D[1],D[2]);
//    printf("\nSpeed %f\n asked for v %f w %f", es.robotFunctions.currentLinearSpeed, v, w);
    int result = dwmonitor(X, D);
//    printf("\n Conservative Result: %d", result);
    std::exit;
    if (result != 1)
    {  
//      printf("\n if (result != 1): %d", result);
//      std::exit;
        //you are going to be unsafe soon
        D[0] = d0 + extraClearance;
        int result2 = dwmonitor(X, D);
        if(debug)
	printf("\n Conservative Not Passed");
        if (result2 == 1)
        {
           if(debug)
           printf("\n-------- Please back up you are too close to the obstacle -----");
            okayToGob = true;
        }

    }
    else
    {
//	printf("\n okay well it doesnt even matter cuz conservative passed");
        okayToGob = true;
    }
    return okayToGob;

}

