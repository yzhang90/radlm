/*
 *      testRosParameters.cpp
 *
 * 	Test the getROSParameters class
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */

#include "cmu_ros_pub_sub/getRosParameters.h"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc,argv,"testGetRosParameters"); 
	ros::NodeHandle n; 
	ros::Rate rate(1000);
	getROSParameters rP(n); 
        rP.convertGPSToggle();
	rP.gpsMode = true;
	while(ros::ok())
	{
		ros::spinOnce();
		rP.displayGPS(); 
		rP.displayOdomXYZ(); 
		rP.displayAngle();
		rP.displayIMU(); 
		rP.displayImuAngle();
		rP.displayConvertedGPS();
		ros::Time start = ros::Time::now(); 
		ros::Time end = ros::Time::now(); 
		while((end-start).toSec() < 3) 
		{
			rP.moveRobot(1,1); 
			end = ros::Time::now();
		}
		cout<<"Done\n";
		cin.get();
		rate.sleep();
	}

	return 0;	
}
