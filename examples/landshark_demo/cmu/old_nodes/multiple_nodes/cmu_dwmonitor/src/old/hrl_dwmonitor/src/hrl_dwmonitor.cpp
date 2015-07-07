/*
 *      dwmonitor.cpp
 *
 *
 *      Author: Fatma Faruq, Jason Larkin
 *      HACMS CMU Team, Advisor Manuela Veloso, SpiralGen, Inc.
 */
//--------------------------------------------------------------------------------
#include <ros/ros.h>
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

#include "landshark_msgs/BoolStamped.h"
#include <landshark_gps/GpsProjection.h>

#include "hrl_dwmonitor/obstacleMsg.h"
#include "hrl_dwmonitor/dwMonitorSwitch.h"
#include "hrl_dwmonitor/obstacleMsg.h"
#include "hrl_dwmonitor/Speed.h"
#include "hrl_dwmonitor/WheelEncoderFront.h"
#include "hrl_dwmonitor/WheelEncoderRear.h"

//--------------------------------------------------------------------------------
using namespace std;

hrl_dwmonitor::Speed msgSvSpeed;
sensor_msgs::NavSatFix msgAbvGps;
geometry_msgs::TwistStamped atempTwist;
geometry_msgs::TwistStamped twistSafeSpeed;
geometry_msgs::TwistStamped twistMinSpeed;
landshark::GpsProjection gpsProjection;
hrl_dwmonitor::obstacleMsg msgObstacle;

static const double CMU_lon = -79.945;
static const double CMU_lat = 40.443;

static const double HRL_lon = 0.0;
static const double HRL_lat = 0.0;

int dwMonitorSwitch = 0;
//--------------------------------------------------------------------------------
void getSvSpeed(const hrl_dwmonitor::Speed::ConstPtr& pMsg)
{
    msgSvSpeed = *pMsg;
}

void printSvSpeed()
{
    printf("printSvSpeed: msgSvSpeed.speed = %.2f\n", msgSvSpeed.speed);
}

void getAbvGps(const sensor_msgs::NavSatFix::ConstPtr& pMsg)
{
    msgAbvGps = *pMsg;
}

void dwMonitorSwitchCallback(\
  const hrl_dwmonitor::dwMonitorSwitch::ConstPtr& pMsg)
{
    if(pMsg != NULL)
        dwMonitorSwitch = pMsg->dwMonitorSwitch;
}

void getObstacle(\
  const hrl_dwmonitor::obstacleMsg::ConstPtr& pMsg)
{ 
  msgObstacle = *pMsg; 
}

bool okayToGo(\
float car_x, float car_y, float obs_x, float obs_y,\
float v, float w, float dt, float max_accv, float clearance1, float clearance2)
{
//  ros::spinOnce();
  bool okayToGob = false;
//  int result; int result2; 
  
  double f_v = fabs(v);//f_V = maxObsVel;
  float accv = max_accv;
  float timePeriod = dt;
  float X[5];
  
  X[0] = fabs((float)v);
  X[1] = (float)car_x;
  X[2] = (float)car_y;
  X[3] = (float)obs_x;
  X[4] = (float)obs_y;
  double extraClearance = clearance1;//1.5;
  double conservativeClearance = clearance2;//2.5;
  double d0 = (double)((accv / accv + 1) * (accv / 2 * timePeriod * timePeriod + f_v * timePeriod));
  double D[3];
  D[0] = d0 + conservativeClearance;
  D[1] = (double)((f_v / accv) + timePeriod * (accv / accv + 1));
  D[2] = (double)(1 / (2 * accv));

  int result = dwmonitor(X, D);
  
  if (result == 1)
  {
    okayToGob = true;
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
  
return okayToGob;
}

//--------------------------------------------------------------------------------
int main(int argc, char **argv)
{
//initialize ros
  ros::init(argc, argv, "hrl_dwmonitor");
  ros::NodeHandle node;
  ros::Rate rate(1000);
  
//get parameters
  string topicSvSpeed;
  string topicAbvGps;
  string topicDwmonitorStatus;
  string topicDwmonitorSwitch;
  string topicDwmonitorObstacle;
  string topicSafeSpeed;
  string topicMinSpeed;
  bool useDWmonitor;
  bool okayBool = false;
  double dt = 0.1; 
  double clearanceDist1 = 0; 
  double clearanceDist2 = 0;
  double accel_lin_max = 7.24; 
  double accel_ang_max = 9.8; 
  double max_obs_vel = 1; 
  double max_obs_dist = 10; 
  double vel_lin_max = 1; 
  double vel_ang_max = 1; 
  double epsilon = 0.25;
  
  double temp_x,temp_y;
  float gps_meters_x,gps_meters_y,obstacle_gps_meters_x,obstacle_gps_meters_y;
  
//--------------------------------------------------------------------------------
//input params
//--------------------------------------------------------------------------------	 
  string paramName = "dt";
  if( !node.hasParam(paramName))
	   {
		  printf(
		  "\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),dt);
	   }
	 else
	 node.getParam(paramName,dt);
	 
  if( !node.hasParam("safetyClearance2"))
	 {
		  printf(
		  "\nCan't get %s from parameter server. Setting to default value %f",
		  "safetyClearance2",2.5);
		  clearanceDist1 = 2.5;
	 }
	 else
	 node.getParam("safetyClearance2",clearanceDist2);

  paramName = "safetyClearance1";
  if( !node.hasParam(paramName))
	   {
		  printf(
		  "\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),1.5);
		  clearanceDist1 = 1.5;
 	  }
	 else
	 node.getParam(paramName,clearanceDist1);
	 
  paramName = "accel_lin_max";
  if( !node.hasParam(paramName))
	   {
		  printf(
		  "\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),accel_lin_max);
	   }
	 else
	 node.getParam(paramName,accel_lin_max);

  paramName = "accel_ang_max";
  if( !node.hasParam(paramName))
	 {
		  printf(
		  "\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),accel_ang_max);
	 }
	 else
	 node.getParam(paramName,accel_ang_max);

  paramName = "vel_ang_max";
  if( !node.hasParam(paramName))
	 {
		  printf(
		  "\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),vel_ang_max);
	 }
	 else
	 node.getParam(paramName,vel_ang_max);

  paramName = "vel_lin_max";
  if( !node.hasParam(paramName))
	 {
		  printf(
		  "\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),vel_lin_max);
	 }
	 else
	 node.getParam(paramName,vel_lin_max);

  paramName = "max_obs_vel";
  if( !node.hasParam(paramName))
	 {
		  printf(
		  "\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),max_obs_vel);
	 }
	 else
	 node.getParam(paramName,max_obs_vel);

  paramName = "max_obs_dist";
  if( !node.hasParam(paramName))
	 {
		  printf(
		  "\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),max_obs_dist);
	 }
	 else
	 node.getParam(paramName,max_obs_dist);

  paramName = "epsilon";
  if( !node.hasParam(paramName))
	 {
		  printf(
		  "\nCan't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),epsilon);
	 }
	 else
	 node.getParam(paramName,epsilon);

//--------------------------------------------------------------------------------
//topics dwmonitor
//--------------------------------------------------------------------------------	
  if(!(node.hasParam("dwmonitor_status_topic")))
  {cout<<"\nCan't find dwmonitor_status_topic. Setting to /hrl/dwmonitor/status";
    topicDwmonitorStatus = "/hrl/dwmonitor/status";}
  else{node.getParam("dwmonitor_status_topic", topicDwmonitorStatus);}
  ros::Publisher pubStatus =\
  node.advertise < landshark_msgs::BoolStamped > (topicDwmonitorStatus, 1);
  
  if(!(node.hasParam("dwmonitor_switch_topic")))
  {cout<<"\nCan't find dwmonitor_switch_topic. Setting to /hrl/dwmonitor/switch";
    topicDwmonitorSwitch = "/hrl/dwmonitor/switch";}
  else{node.getParam("dwmonitor_switch_topic", topicDwmonitorSwitch);}  
  ros::Subscriber subSwitch =\
  node.subscribe <hrl_dwmonitor::dwMonitorSwitch> (topicDwmonitorSwitch,1,&dwMonitorSwitchCallback);
  ros::Publisher pubSwitch =\
  node.advertise < landshark_msgs::BoolStamped > (topicDwmonitorSwitch, 1);
  
  if(!(node.hasParam("dwmonitor_obstacle_topic")))
  {cout<<"\nCan't find dwmonitor_obstacle_topic. Setting to /hrl/dwmonitor/obstacle";
    topicDwmonitorObstacle = "/hrl/dwmonitor/obstacle";}
  else{node.getParam("dwmonitor_obstacle_topic", topicDwmonitorObstacle);}  
  ros::Subscriber subObstacle =\
  node.subscribe <hrl_dwmonitor::obstacleMsg> (topicDwmonitorObstacle,1,&getObstacle);
    
  if (!node.hasParam("dwmonitor_safeSpeed_topic"))
  {cout << "\nCan't find safeSpeed_topic. Setting to /hrl/dwmonitor/safeSpeed";
    topicSafeSpeed = "/hrl/dwmonitor/safeSpeed";
    cout << "\nPublishing to " << topicSafeSpeed << endl;}
  else{node.getParam("dwmonitor_safeSpeed_topic", topicSafeSpeed);
    cout << "\nSubscribing to " << topicSafeSpeed << endl;}
  ros::Publisher pubSafeSpeed =\
  node.advertise < geometry_msgs::TwistStamped > (topicSafeSpeed, 1);  
  
  if (!node.hasParam("dwmonitor_minSpeed_topic"))
  {cout << "\nCan't find minSpeed_topic. Setting to /hrl/dwmonitor/minSpeed";
    topicMinSpeed = "/hrl/dwmonitor/minSpeed";
    cout << "\nPublishing to " << topicMinSpeed << endl;}
  else{node.getParam("dwmonitor_minSpeed_topic", topicMinSpeed);
    cout << "\nSubscribing to " << topicMinSpeed << endl;}
  ros::Publisher pubMinSpeed =\
  node.advertise < geometry_msgs::TwistStamped > (topicMinSpeed, 1);   
  twistMinSpeed.twist.linear.x = 0.0; 
  twistMinSpeed.twist.angular.z = 0.0;
  pubMinSpeed.publish(twistMinSpeed);
//--------------------------------------------------------------------------------
//topics hrl
//--------------------------------------------------------------------------------	  
  if (!node.hasParam("sv_speed_topic"))
  {cout << "\nCan't find sv_speed_topic. Setting to /sv/speed";
    topicSvSpeed = "/sv/speed";}
  else{node.getParam("sv_speed_topic", topicSvSpeed);
    cout << "\nSubscribing to " << topicSvSpeed << endl;}
  ros::Subscriber subSvSpeed =\
  node.subscribe < hrl_dwmonitor::Speed > (topicSvSpeed, 1, getSvSpeed);
  
  if (!node.hasParam("abv_gps_topic"))
  {cout << "\nCan't find velocity_sub_topic. Setting to /abv/gps";
    topicAbvGps = "/abv/gps";}
  else{node.getParam("abv_gps_topic", topicAbvGps);
    cout << "\nSubscribing to " << topicAbvGps << endl;}
  ros::Subscriber subAbvGps = node.subscribe < sensor_msgs::NavSatFix > (topicAbvGps, 1, getAbvGps);
  gpsProjection.SetLocalCoordinateSystemOrigin(HRL_lat,HRL_lon);

landshark_msgs::BoolStamped statusMessage;
//--------------------------------------------------------------------------------
  while (ros::ok())
  {
    ros::spinOnce();
    statusMessage.header.stamp = ros::Time::now();
    statusMessage.data = false;
    if(dwMonitorSwitch == hrl_dwmonitor::dwMonitorSwitch::ON)
    {
//      twistSafeSpeed.header.stamp.secs = ros::Time::now();
      gpsProjection.GetLocalCoordinatesFromGps(\
      (double)(msgAbvGps.latitude),(double)(msgAbvGps.longitude),temp_x,temp_y);
      gps_meters_x = (float)temp_x;
      gps_meters_y = (float)temp_y;
      
      gpsProjection.GetLocalCoordinatesFromGps(\
      (double)(msgObstacle.coordinates.point.x),(double)(msgObstacle.coordinates.point.y),temp_x,temp_y);
      obstacle_gps_meters_x = (float)temp_x;
      obstacle_gps_meters_y = (float)temp_y;
      
//      printf("\n msgSvSpeed.speed = %f", msgSvSpeed.speed);
//      printf("\n msgAbvGps.latitude = %f", (double)(msgAbvGps.latitude));
//      printf("\n msgAbvGps.longitude = %f", (double)(msgAbvGps.longitude));
//      printf("\n gps_meters_x = %f", gps_meters_x);
//      printf("\n gps_meters_y = %f", gps_meters_y);
//      printf("\n msgObstacle.coordinates.point.x = %f", msgObstacle.coordinates.point.x);
//      printf("\n msgObstacle.coordinates.point.y = %f", msgObstacle.coordinates.point.y);
//      printf("\n obstacle_gps_meters_x = %f", obstacle_gps_meters_x);
//      printf("\n obstacle_gps_meters_y = %f", obstacle_gps_meters_y); 

      okayBool = okayToGo(\
      gps_meters_x, gps_meters_y, obstacle_gps_meters_x, obstacle_gps_meters_y,\
      msgSvSpeed.speed*(1000.0/3600.0), 0.0, dt, accel_lin_max, clearanceDist1, clearanceDist2);
      
      statusMessage.data = okayBool;

// convert to m/s   
      if (!okayBool)
      {
        twistSafeSpeed.twist.linear.x =  0.0;
        twistSafeSpeed.twist.angular.z = 0.0;
      }
      else
      {
// convert to m/s 
        twistSafeSpeed.twist.linear.x = msgSvSpeed.speed*(1000.0/3600.0);
        twistSafeSpeed.twist.angular.z = 0.0;
      }
      pubSafeSpeed.publish(twistSafeSpeed);
// convert to m/s     
      twistMinSpeed.twist.linear.x = std::min(\
      (double)twistSafeSpeed.twist.linear.x,(double)msgSvSpeed.speed*(10000.0/3600.0)); 
      twistMinSpeed.twist.angular.z = 0.0;

      pubMinSpeed.publish(twistMinSpeed);
      
      pubStatus.publish(statusMessage);
      
      rate.sleep();
      }
  }
  return 0;
}
