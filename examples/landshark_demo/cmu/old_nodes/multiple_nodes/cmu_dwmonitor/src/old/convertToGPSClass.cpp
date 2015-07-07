/*
 *      convertToGPSClass.cpp
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */


#include "convertToGPSClass.h"

convertToGPSClass::convertToGPSClass(bool gps, ros::NodeHandle nh)
{
    nodeHandle = nh;
    es.initClass(nodeHandle);
    gpsMode = gps;
    if (gpsMode)
    {
        es.robotFunctions.convertGPSToggle();
        es.robotFunctions.gpsMode = true;
    }
//  es.setVmax(1);
//  es.setWmax(1);
//  es.setDt(0.25);
//  es.setAccs(4, 4);
//  es.setDmax(10);
}

convertToGPSClass::convertToGPSClass()
{
    //es.setVmax(1);
    //es.setWmax(1);
    //es.setDt(0.25);
    //es.setAccs(4, 4);
    //es.setDmax(10);
}

void convertToGPSClass::initClass(bool gps, ros::NodeHandle nh)
{
    nodeHandle = nh;
    es.initClass(nodeHandle);
    gpsMode = gps;
    if (gpsMode)
    {
        es.robotFunctions.convertGPSToggle();
        es.robotFunctions.gpsMode = true;
    }
    //es.setVmax(1);
    //es.setWmax(1);
    //es.setDt(0.25);
    //es.setAccs(4, 4);
    //es.setDmax(10);
}

void convertToGPSClass::converPointsToGPS(double lat, double lon, double alt)
{
    float robot_x = es.robotFunctions.gpsPosition.point.x;
    float robot_y = es.robotFunctions.gpsPosition.point.y;
    // get robot orignial position
    bool canConvert = true;
    if (!(robot_x == 0 && robot_y == 0))
        canConvert = true;
    if (canConvert)
    {

        float rx = es.robotFunctions.gpsOriginalPosition.point.x;
        float ry = es.robotFunctions.gpsOriginalPosition.point.y;
        es.robotFunctions.convertAnyGPSToXY(lon,lat,alt, rx,ry);//, float sf_long, float sf_lat);
        printf("\n Actual Position \n lat %f lon %f \n", lat, lon);
        printf("\n Converted gps \n x %f y %f\n", rx, ry);
	gpsX = (double)rx; 
	gpsY = (double)ry;
        printf("\n Converted gps \n x %f y %f\n", gpsX, gpsY);
        //printf("\n0 %f %f", rx, ry);
        //  ofstream myfile;
        //  myfile.open("gpsLocations.txt");
        //  myfile << "0 " << robot_x << " " << robot_y<<"\n2";
        //  myfile.close();
    }

}
void convertToGPSClass::initGPS(double lat, double lon)
{
    es.robotFunctions.initGPSProjection(lon,lat);
}
/*
int main(int argc, char **argv)
{
  //initialize ros
  ros::init(argc, argv, "convertToGPSClass");
  ros::NodeHandle n;
  ros::Rate rate(1000);
  convertToGPSClass slg(true, n);
    static const double CMU_long = -79.945;
  static const double CMU_lat = 40.443;
  slg.es.robotFunctions.initGPSProjection(CMU_long, CMU_lat);
  while (ros::ok())
  {
    ros::spinOnce();
 //Actual Position
 //lat 37.457102 lon -122.173497
    slg.converPointsToGPS(37.457102,-122.173497 ,0);
  }
  return 0;
}*/
