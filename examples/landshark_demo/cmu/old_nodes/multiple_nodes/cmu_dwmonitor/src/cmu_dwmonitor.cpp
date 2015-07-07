/*
 *      landshark_dwmonitor.cpp
 *
 *
 *      Author: Fatma Faruq, Jason Larkin
 *      HACMS CMU Team, Advisor Manuela Veloso, SpiralGen, Inc.
 */
//--------------------------------------------------------------------------------
#include "ros/ros.h"
#include "dwmonitor.h"

#include <map>
#include <string>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include "nav_msgs/Odometry.h"

#include "landshark_msgs/BoolStamped.h"
#include <landshark_gps/GpsProjection.h>

//#include "cmu_dwmonitor/dwswitch.h"
#include "cmu_dwmonitor/obstacle.h"

//--------------------------------------------------------------------------------
using namespace std;

sensor_msgs::NavSatFix msgGps;
geometry_msgs::TwistStamped msgBaseVelocity;
nav_msgs::Odometry msgOdom;

cmu_dwmonitor::obstacle msgObstacle;
geometry_msgs::TwistStamped twistSafeSpeed;
geometry_msgs::TwistStamped twistMinSpeed;
landshark::GpsProjection gpsProjection;

static const double CMU_lon = -79.945;
static const double CMU_lat = 40.443;

static const double SRI_lon = -122.177062167;
static const double SRI_lat = 37.4559631667;

landshark_msgs::BoolStamped msgDWMonitorSwitch;
//--------------------------------------------------------------------------------
void getBaseVelocity(const geometry_msgs::TwistStamped::ConstPtr& pMsg)
{msgBaseVelocity = *pMsg;}

void getOdom(const nav_msgs::Odometry::ConstPtr& pMsg)
{msgOdom = *pMsg;}

void getGps(const  sensor_msgs::NavSatFix::ConstPtr& pMsg)
{msgGps = *pMsg;}

void dwMonitorSwitchCallback(const landshark_msgs::BoolStamped::ConstPtr& pMsg)
{if(pMsg != NULL) msgDWMonitorSwitch = *pMsg;}

void getObstacle(const cmu_dwmonitor::obstacle::ConstPtr& pMsg)
{msgObstacle = *pMsg;}

bool okayToGo(\
float robot_x, float robot_y, float obs_x, float obs_y,\
float v, float w, float dt, float max_accv, float clearance1, float clearance2)
{
  bool okayToGob = false; float X[5]; double D[3];
  double f_v = fabs(v); float accv = max_accv; float timePeriod = dt;
  X[0] = fabs((float)v); 
  X[1] = (float)robot_x; X[2] = (float)robot_y; 
  X[3] = (float)obs_x; X[4] = (float)obs_y;
  double extraClearance = clearance1; double conservativeClearance = clearance2;
  double d0 = (double)((accv / accv + 1) *\
  (accv / 2 * timePeriod * timePeriod + f_v * timePeriod));
  D[0] = d0 + conservativeClearance;
  D[1] = (double)((f_v / accv) + timePeriod * (accv / accv + 1));
  D[2] = (double)(1 / (2 * accv));
  int result = dwmonitor(X, D);
  if (result == 1)
  {okayToGob = true;}
return okayToGob;
}
  
//  if (result != 1)
//  {  
////you are going to be unsafe soon
//    D[0] = d0 + extraClearance;
//    int result2 = dwmonitor(X, D);
////    printf("\n Conservative Not Passed");
//    if (result2 == 1)
//    {
////      printf("\n-------- back up you are too close to the obstacle -----");
//      okayToGob = true;
//    }
//  }
//  else{okayToGob = true;}
  


//--------------------------------------------------------------------------------
int main(int argc, char **argv)
{
//initialize ros
  ros::init(argc, argv, "cmu_dwmonitor"); ros::NodeHandle node;
  ros::Rate rate(30);
//get parameters
  string topicBaseVelocity; string topicGps; string topicOdom;
  string topicDwmonitorStatus; string topicDwmonitorSwitch; 
  string topicDwmonitorObstacle;
  string topicSafeSpeed; string topicMinSpeed;
  string position_mode = "gps";
  bool useDWmonitor; bool okayBool = false;
  double origin_lat = SRI_lat; double origin_lon = SRI_lon;  
  double clearanceDist1 = 0; double clearanceDist2 = 0;
  double accel_lin_max = 4.0; double accel_ang_max = 4.0; 
  double max_obs_vel = 1.0; double max_obs_dist = 10.0; 
  double vel_lin_max = 1.0; double vel_ang_max = 1.0; 
  double epsilon = 0.25; double dt = 0.1;
  
  double temp_x,temp_y;
  double obstacle_meters_x,obstacle_meters_y,robot_meters_x,robot_meters_y;
  
//--------------------------------------------------------------------------------
//input params
//--------------------------------------------------------------------------------	 
  string paramName = "/cmu_dwmonitor/dt";
  if( !node.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",paramName.c_str(),dt);}
	 else {node.getParam(paramName,dt);}
	 
	 paramName = "origin_lat";
  if( !node.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",
		 paramName.c_str(),origin_lat);
	 }
	 else
	 node.getParam(paramName,origin_lat);
	 
	 paramName = "origin_lon";
  if( !node.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",
		 paramName.c_str(),origin_lon);
	 }
	 else
	 node.getParam(paramName,origin_lon);	 
	 
	 paramName = "/cmu_dwmonitor/safetyClearance2";
  if( !node.hasParam(paramName))
	 {
		 printf("\nCan't get %s from parameter server. Setting to default value %f",
		  "safetyClearance2",2.5);
		  clearanceDist1 = 2.5;
	 }
	 else
	 node.getParam("/cmu_dwmonitor/safetyClearance2",clearanceDist2);

  paramName = "/cmu_dwmonitor/safetyClearance1";
  if( !node.hasParam(paramName))
	   {printf("\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),1.5);
		  clearanceDist1 = 1.5;
 	  }
	 else
	 node.getParam(paramName,clearanceDist1);
	 
  paramName = "/cmu_dwmonitor/accel_lin_max";
  if( !node.hasParam(paramName))
	   {printf("\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),accel_lin_max);
	   }
	 else
	 node.getParam(paramName,accel_lin_max);

  paramName = "/cmu_dwmonitor/accel_ang_max";
  if( !node.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),accel_ang_max);
	 }
	 else
	 node.getParam(paramName,accel_ang_max);

  paramName = "/cmu_dwmonitor/vel_ang_max";
  if( !node.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),vel_ang_max);
	 }
	 else
	 node.getParam(paramName,vel_ang_max);

  paramName = "/cmu_dwmonitor/vel_lin_max";
  if( !node.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),vel_lin_max);
	 }
	 else
	 node.getParam(paramName,vel_lin_max);

  paramName = "/cmu_dwmonitor/max_obs_vel";
  if( !node.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),max_obs_vel);
	 }
	 else
	 node.getParam(paramName,max_obs_vel);

  paramName = "/cmu_dwmonitor/max_obs_dist";
  if( !node.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),max_obs_dist);
	 }
	 else
	 node.getParam(paramName,max_obs_dist);

  paramName = "/cmu_dwmonitor/epsilon";
  if( !node.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",
		 paramName.c_str(),epsilon);
	 }
	 else
	 node.getParam(paramName,epsilon);
	 
	 paramName = "/cmu_dwmonitor/position_mode";
  if( !node.hasParam(paramName))
	 {printf("\nCan't get cmu_dwmonitor/position_mode from parameter server. Setting to default value gps");
	 }
	 else{node.getParam("cmu_dwmonitor/position_mode",position_mode);}

//--------------------------------------------------------------------------------
//topics dwmonitor
//--------------------------------------------------------------------------------	
  if(!(node.hasParam("/cmu_dwmonitor/status")))
  {cout<<"\nCan't find /cmu_dwmonitor/status. Setting to /cmu_dwmonitor/status";
    topicDwmonitorStatus = "/cmu_dwmonitor/status";}
  else{node.getParam("/cmu_dwmonitor/status", topicDwmonitorStatus);}
  ros::Publisher pubStatus =\
  node.advertise < landshark_msgs::BoolStamped > (topicDwmonitorStatus, 1);
  
  if(!(node.hasParam("/cmu_dwmonitor/dwswitch")))
  {cout<<"\nCan't find /cmu_dwmonitor/dwswitch. Setting to /cmu_dwmonitor/dwswitch";
    topicDwmonitorSwitch = "/cmu_dwmonitor/dwswitch";}
  else{node.getParam("/cmu_dwmonitor/dwswitch", topicDwmonitorSwitch);}  
  ros::Subscriber subSwitch =\
  node.subscribe <landshark_msgs::BoolStamped> (topicDwmonitorSwitch,1,\
  dwMonitorSwitchCallback);
  ros::Publisher pubSwitch =\
  node.advertise < landshark_msgs::BoolStamped > (topicDwmonitorSwitch, 1);
  
  msgDWMonitorSwitch.data = false;
  
  if(!(node.hasParam("/cmu_dwmonitor/obstacle")))
  {cout<<"\nCan't find /cmu_dwmonitor/obstacle. Setting to /cmu_dwmonitor/obstacle";
    topicDwmonitorObstacle = "/cmu_dwmonitor/obstacle";}
  else{node.getParam("/cmu_dwmonitor/obstacle", topicDwmonitorObstacle);}  
  ros::Subscriber subObstacle =\
  node.subscribe <cmu_dwmonitor::obstacle> (topicDwmonitorObstacle,1,&getObstacle);
  
  if (!node.hasParam("/cmu_dwmonitor/safeSpeed"))
  {cout << "\nCan't find /cmu_dwmonitor/safeSpeed. Setting to /cmu_dwmonitor/safeSpeed";
    topicSafeSpeed = "/cmu_dwmonitor/safeSpeed";
    cout << "\nPublishing to " << topicSafeSpeed << endl;}
  else{node.getParam("/cmu_dwmonitor/safeSpeed", topicSafeSpeed);
    cout << "\nSubscribing to " << topicSafeSpeed << endl;}
  ros::Publisher pubSafeSpeed =\
  node.advertise < geometry_msgs::TwistStamped > (topicSafeSpeed, 1);   
  twistSafeSpeed.twist.linear.x = 0.0; 
  twistSafeSpeed.twist.angular.z = 0.0;
  
  if (!node.hasParam("/cmu_dwmonitor/minSpeed"))
  {cout << "\nCan't find /cmu_dwmonitor/minSpeed. Setting to /cmu_dwmonitor/minSpeed";
    topicMinSpeed = "/cmu_dwmonitor/minSpeed";
    cout << "\nPublishing to " << topicMinSpeed << endl;}
  else{node.getParam("/cmu_dwmonitor/minSpeed", topicMinSpeed);
    cout << "\nSubscribing to " << topicMinSpeed << endl;}
  ros::Publisher pubMinSpeed =\
  node.advertise < geometry_msgs::TwistStamped > (topicMinSpeed, 1);   
  twistMinSpeed.twist.linear.x = 0.0; 
  twistMinSpeed.twist.angular.z = 0.0;
  
  if (!node.hasParam("/landshark_control/base_velocity"))
  {cout << "\nCan't find /landshark_control/base_velocity. Setting to /landshark_control/base_velocity";
    topicBaseVelocity = "/landshark_control/base_velocity";
    cout << "\nPublishing to " << topicBaseVelocity << endl;}
  else{node.getParam("/landshark_control/base_velocity", topicBaseVelocity);
    cout << "\nSubscribing to " << topicBaseVelocity << endl;}
  ros::Subscriber subBaseVelocity =\
  node.subscribe < geometry_msgs::TwistStamped > (topicBaseVelocity, 1, getBaseVelocity); 
  
  if (!node.hasParam("/landshark/odom"))
  {cout << "\nCan't find /landshark/odom. Setting to /landshark/odom";
    topicOdom = "/landshark/odom";}
  else{node.getParam("/landshark/odom", topicOdom);
    cout << "\nSubscribing to " << topicOdom << endl;}
  ros::Subscriber subOdom =\
  node.subscribe < nav_msgs::Odometry > (topicOdom, 1, getOdom);
  
  if (!node.hasParam("/landshark/gps"))
  {cout << "\nCan't find /landshark/gps. Setting to /landshark/gps";
    topicGps = "/landshark/gps";}
  else{node.getParam("/landshark/gps", topicGps);
    cout << "\nSubscribing to " << topicGps << endl;}
  ros::Subscriber subGps =\
  node.subscribe < sensor_msgs::NavSatFix > (topicGps, 1, getGps);

//--------------------------------------------------------------------------------
msgObstacle.type = 1;
landshark_msgs::BoolStamped statusMessage;

//sleep so gps topic gets published to
ros::Duration(1.0).sleep();

  while (ros::ok())
  {
    ros::spinOnce();
    statusMessage.header.stamp = ros::Time::now();
    statusMessage.data = false;   
//    printf("\nmsgDWMonitorSwitch.dwswitch = %d\n", msgDWMonitorSwitch.dwswitch);
    if(msgDWMonitorSwitch.data)
    {
//      twistSafeSpeed.header.stamp.secs = ros::Time::now();
      if (position_mode == "odom")
      {
//        printf("\ninside odom\n");
        robot_meters_x = msgOdom.pose.pose.position.x;
        robot_meters_y = msgOdom.pose.pose.position.y;
        obstacle_meters_x = msgObstacle.x;
        obstacle_meters_y = msgObstacle.y;
      }
      else if (position_mode == "gps")
      {
//        printf("\ninside gps\n");
//        if (msgObstacle.type==0){
//          printf("\norigin_lon = %f\n",origin_lon);
//          printf("\norigin_lat = %f\n",origin_lat);
//          printf("\nmsgGps.longitude = %f\n",msgGps.longitude);
//          printf("\nmsgGps.latitude = %f\n",msgGps.latitude);

          gpsProjection.SetLocalCoordinateSystemOrigin(origin_lon,origin_lat); 
          gpsProjection.GetLocalCoordinatesFromGps(\
          (double)(msgGps.longitude),(double)(msgGps.latitude),temp_x,temp_y);
          robot_meters_x = (double)temp_x;
          robot_meters_y = (double)temp_y;
//          gpsProjection.GetLocalCoordinatesFromGps(\
//          (double)(msgObstacle.x),\
//          (double)(msgObstacle.y),temp_x,temp_y);
//          obstacle_meters_x = (double)temp_x;
//          obstacle_meters_y = (double)temp_y;}
//        else if (msgObstacle.type==1){
          obstacle_meters_x = (double)msgObstacle.x;
          obstacle_meters_y = (double)msgObstacle.y;
//          }
      }      
//      printf("\n msgSvSpeed.speed = %f", msgSvSpeed.speed);
      okayBool = okayToGo(\
      robot_meters_x, robot_meters_y, obstacle_meters_x, obstacle_meters_y,\
      msgBaseVelocity.twist.linear.x, msgBaseVelocity.twist.angular.z, dt,\
      accel_lin_max, clearanceDist1, clearanceDist2);
      
      statusMessage.data = okayBool;
   
      if (!okayBool)
      {
        twistSafeSpeed.twist.linear.x =  0.0;
        twistSafeSpeed.twist.angular.z = 0.0;
      }
      else
      {
        twistSafeSpeed.twist.linear.x = msgBaseVelocity.twist.linear.x;
        twistSafeSpeed.twist.angular.z = msgBaseVelocity.twist.angular.z;
      }
      
      pubSafeSpeed.publish(twistSafeSpeed);
      
//      printf("\nmsgBaseVelocity.twist.linear.x = %f \n", msgBaseVelocity.twist.linear.x);
           
      twistMinSpeed.twist.linear.x = std::min(\
      (double)twistSafeSpeed.twist.linear.x,(double)msgBaseVelocity.twist.linear.x); 
      twistMinSpeed.twist.angular.z = std::min(\
      (double)twistSafeSpeed.twist.angular.z,(double)msgBaseVelocity.twist.angular.z);

      pubMinSpeed.publish(twistMinSpeed);
      
      pubStatus.publish(statusMessage);      
    }
    rate.sleep();
  }
  return 0;
}
