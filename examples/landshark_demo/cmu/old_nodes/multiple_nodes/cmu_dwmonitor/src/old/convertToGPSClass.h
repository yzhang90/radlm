/*
 *      convertToGPSClass.h
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */


#include "environmentSpecifics.h"
#include <iostream>
using namespace std;

class convertToGPSClass
{
public:
    double gpsX;
    double gpsY;
    environmentSpecifics es;
    ros::NodeHandle nodeHandle;
    //double wallDist;
    bool gpsMode;
    //bool obstaclesConverted;
    convertToGPSClass(bool gps, ros::NodeHandle nh);
    convertToGPSClass();
    void initClass(bool gps,ros::NodeHandle nh);
    void converPointsToGPS(double lat, double lon, double alt);
    void initGPS(double lat, double lon);

};
