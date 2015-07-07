/*
 *      landshark_dwmonitor.cpp
 *
 *
 *      Author: Fatma Faruq, Jason Larkin
 *      HACMS CMU Team, Advisor Manuela Veloso, SpiralGen, Inc.
 */
//--------------------------------------------------------------------------------
#include "ros/ros.h"

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

//#include "cmu_monitor_monitor/cmu_velocity.h"
#include "cmu_sensor_monitor/AnomalyMsg.h"
//#include "cmu_dwmonitor/dwswitch.h"

//--------------------------------------------------------------------------------
using namespace std;

sensor_msgs::NavSatFix msgGps;
geometry_msgs::TwistStamped msgBaseVelocity;
nav_msgs::Odometry msgOdom;

landshark_msgs::BoolStamped msgDeadman;
landshark_msgs::BoolStamped msgDWmonitorStatus;
cmu_sensor_monitor::AnomalyMsg msgMonitorStatus;
geometry_msgs::TwistStamped msgCmuVelocity;
//cmu_velocity::cmu_deadman msgCmuDeadman;
landshark::GpsProjection gpsProjection;
landshark_msgs::BoolStamped msgDWMonitorSwitch;

static const double CMU_lon = -79.945;
static const double CMU_lat = 40.443;

static const double SRI_lon = -122.177062167;
static const double SRI_lat = 37.4559631667;

string dwMonitorSwitch = "off";
bool falseBool = false;
//--------------------------------------------------------------------------------
//void getBaseVelocity(const geometry_msgs::TwistStamped::ConstPtr& pMsg)
//{msgBaseVelocity = *pMsg;}

void getOdom(const nav_msgs::Odometry::ConstPtr& pMsg)
{msgOdom = *pMsg;}

void getDWmonitorStatus(const landshark_msgs::BoolStamped::ConstPtr& pMsg)
{
if(pMsg != NULL){msgDWmonitorStatus = *pMsg;}
else {msgDWmonitorStatus.data=false;}
}

void dwMonitorSwitchCallback(\
  const landshark_msgs::BoolStamped::ConstPtr& pMsg)
{if(pMsg != NULL) msgDWMonitorSwitch = *pMsg;}

void getMonitorStatus(const cmu_sensor_monitor::AnomalyMsg::ConstPtr& pMsg)
{msgMonitorStatus = *pMsg;}

void getGps(const sensor_msgs::NavSatFix::ConstPtr& pMsg)
{msgGps = *pMsg;}

void getCmuVelocity(const geometry_msgs::TwistStamped::ConstPtr& pMsg)
{
//msgCmuVelocity = *pMsg;
//printf("msgCmuVelocity.twist.linear.x = %f", msgCmuVelocity.twist.linear.x);
if(pMsg != NULL)
{
msgCmuVelocity = *pMsg;
}
else 
{
msgCmuVelocity.twist.linear.x = 0.0;
msgCmuVelocity.twist.angular.z = 0.0;
}
}

//void getCmuDeadman(const cmu_velocity::cmu_deadman::ConstPtr& pMsg)
//{msgCmuDeadman = *pMsg;}

//void getDWmonitorSwitchCallback(\
//  const landshark_cmu_dwmonitor::dwMonitorSwitch::ConstPtr& pMsg)
//{if(pMsg != NULL)dwMonitorSwitch = pMsg->dwmonitor_switch;}

//void getObstacle(const landshark_cmu_dwmonitor::obstacleMsg::ConstPtr& pMsg)
//{msgObstacle = *pMsg;}


//--------------------------------------------------------------------------------
int main(int argc, char **argv)
{
//initialize ros
  ros::init(argc, argv, "cmu_monitor_monitor"); ros::NodeHandle node;
  ros::Rate rate(30);
//get parameters
  string topicBaseVelocity; string topicGps; string topicOdom;
  string topicDwmonitorStatus; string topicDwmonitorSwitch; 
  string topicDwmonitorObstacle;
  string topicSafeSpeed; string topicMinSpeed;
  string topicCmuVelocity; 
  string topicDeadman; string topicCmuDeadman;
  double dt;
  
//--------------------------------------------------------------------------------
//input params
//--------------------------------------------------------------------------------	 
  string paramName = "/cmu_dwmonitor/dt";
  if( !node.hasParam(paramName))
	 {printf("\nCan't get %s from parameter server. Setting to default value %f",paramName.c_str(),dt);}
	 else {node.getParam(paramName,dt);}

//--------------------------------------------------------------------------------
//topics
//--------------------------------------------------------------------------------	

  if(!(node.hasParam("/cmu_dwmonitor/status")))
  {cout<<"\nCan't find /cmu_dwmonitor/status. Setting to /cmu_dwmonitor/status";
    topicDwmonitorStatus = "/cmu_dwmonitor/status";}
  else{node.getParam("/cmu_dwmonitor/status", topicDwmonitorStatus);}
  ros::Subscriber subDWStatus =\
  node.subscribe < landshark_msgs::BoolStamped > (topicDwmonitorStatus, 1, getDWmonitorStatus);
  
  if(!(node.hasParam("/cmu_dwmonitor/dwswitch")))
  {cout<<"\nCan't find /cmu_dwmonitor/dwswitch. Setting to /cmu_dwmonitor/dwswitch";
    topicDwmonitorSwitch = "/cmu_dwmonitor/dwswitch";}
  else{node.getParam("/cmu_dwmonitor/dwswitch", topicDwmonitorSwitch);}  
  ros::Subscriber subSwitch =\
  node.subscribe <landshark_msgs::BoolStamped> (topicDwmonitorSwitch,1,\
  dwMonitorSwitchCallback);

  if (!node.hasParam("/landshark_control/base_velocity"))
  {cout << "\nCan't find /landshark_control/base_velocity. Setting to /landshark_control/base_velocity";
    topicBaseVelocity = "/landshark_control/base_velocity";
    cout << "\nPublishing to " << topicBaseVelocity << endl;}
  else{node.getParam("/landshark_control/base_velocity", topicBaseVelocity);
    cout << "\nSubscribing to " << topicBaseVelocity << endl;}
//  ros::Subscriber subBaseVelocity =\
//  node.subscribe < geometry_msgs::TwistStamped > (topicBaseVelocity, 1, getBaseVelocity); 

  ros::Publisher pubBaseVelocity =\
  node.advertise < geometry_msgs::TwistStamped > (topicBaseVelocity, 1);   
  msgBaseVelocity.twist.linear.x = 0.0; 
  msgBaseVelocity.twist.angular.z = 0.0;
  
  if(!(node.hasParam("/landshark_control/deadman")))
  {cout<<"\nCan't find /landshark_control/deadman. Setting to /landshark_control/deadman";
    topicDeadman = "/landshark_control/deadman";}
  else{node.getParam("/landshark_control/deadman", topicDeadman);}
  ros::Publisher pubDeadman =\
  node.advertise < landshark_msgs::BoolStamped > (topicDeadman, 1);
  
  if (!node.hasParam("/landshark_control_cmux/base_velocity"))
  {cout << "\nCan't find /landshark_control_cmux/base_velocity. Setting to /landshark_control_cmux/base_velocity";
    topicCmuVelocity = "/landshark_control_cmux/base_velocity";}
  else{node.getParam("/landshark_control_cmux/base_velocity", topicCmuVelocity);
    cout << "\nSubscribing to " << topicCmuVelocity << endl;}
  ros::Subscriber subCmuVelocity =\
  node.subscribe < geometry_msgs::TwistStamped > (topicCmuVelocity, 1, getCmuVelocity);
  
//  if (!node.hasParam("/cmu_velocity/cmu_deadman"))
//  {cout << "\nCan't find /cmu_velocity/cmu_deadman. Setting to /cmu_velocity/cmu_deadman";
//    topicCmuDeadman = "/cmu_velocity/cmu_deadman";}
//  else{node.getParam("/cmu_velocity/cmu_deadman", topicCmuDeadman);
//    cout << "\nSubscribing to " << topicCmuDeadman << endl;}
//  ros::Subscriber subCmuDeadman =\
//  node.subscribe < cmu_velocity::cmu_deadman > (topicCmuDeadman, 1, getCmuDeadman);
  
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
//  gpsProjection.SetLocalCoordinateSystemOrigin(origin_lon,origin_lat); 

//--------------------------------------------------------------------------------

  while (ros::ok())
  {
    ros::spinOnce();
//    msgBaseVelocity.twist.linear.x = 0.0; msgBaseVelocity.twist.angular.z = 0.0;
//    printf("msgCmuVelocity.twist.linear.x = %f", msgCmuVelocity.twist.linear.x);
    if (msgDWMonitorSwitch.data)
    {
      if (msgDWmonitorStatus.data)
      { 
        msgBaseVelocity.twist.linear.x = msgCmuVelocity.twist.linear.x; 
        msgBaseVelocity.twist.angular.z = msgCmuVelocity.twist.angular.z;
      }
      else
      {
        msgBaseVelocity.twist.linear.x = 0.0; 
        msgBaseVelocity.twist.angular.z = 0.0;
      }  
    }
    else
    {
      msgBaseVelocity.twist.linear.x = msgCmuVelocity.twist.linear.x; 
      msgBaseVelocity.twist.angular.z = msgCmuVelocity.twist.angular.z;
    }
    pubBaseVelocity.publish(msgBaseVelocity);
    rate.sleep();
  }
  return 0;
}
