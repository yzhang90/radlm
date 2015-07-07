/*
 *      getRosParameters.cpp
 *
 *	Subscribes to most sensors on the landshark
 *	Publishes to the base_velocity, turret and moog control signals
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */

#ifndef GETROSPARAMSPEED_H_
#define GETROSPARAMSPEED_H_

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <landshark_msgs/WheelEncoder.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <landshark_msgs/JointControl.h>
#include <math.h>

#include <nav_msgs/Odometry.h>

#include <stdio.h>
#include <iostream>
#include <fstream>

#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
//#include "GpsProjection.h"
#include <landshark_gps/GpsProjection.h>
#include <msg_auth/msg_auth.h>

const float BASELAT = -122.429988645;
const float BASELON = 37.78298205;

using namespace std;

class getROSParameters
{
public:
  // ros stuff
  // subscribers, publisher, node handle
  ros::NodeHandle nodeHandle;
  string gpsTopicName;
  string gpsOriginalTopicName;
  ros::Subscriber gpsTopSubscriber;
  ros::Subscriber gpsOriginalTopSubscriber;
  string wheelOdomTopicName;
  ros::Subscriber wheelOdomTopSubscriber;
  string basePublishTopic;
  string odomTopicName;
  ros::Subscriber odomTopSubscriber;
  string imuTopicName;
  string imuRPYTopicName;
  ros::Subscriber imuTopSubscriber;
  ros::Subscriber imuRPYTopSubscriber;

  ros::Publisher baseTopPublisher;

  string moogPublishTopic; 
  string turretPublishTopic; 

  ros::Publisher moogTopPublisher; 
  ros::Publisher turretTopPublisher;

  //ros::Publisher gpsLoc;

  // variables
  geometry_msgs::TwistStamped speedTwist;
  landshark_msgs::JointControl turretControl; 
  landshark_msgs::JointControl moogControl;

  //for now the gps position corresponds to the ros simulator position
  geometry_msgs::PointStamped gpsPosition;
  geometry_msgs::PointStamped gpsOriginalPosition;
  geometry_msgs::PointStamped oldGpsPosition;
  geometry_msgs::PointStamped targetPosition;
  geometry_msgs::PointStamped odomPosition;
  geometry_msgs::PointStamped oldOdomPosition;
  geometry_msgs::Quaternion odomOrientation;
  geometry_msgs::Twist speedFromOdometry;
  geometry_msgs::Point imuRPY;
  
  sensor_msgs::Imu imuData;
  sensor_msgs::NavSatFix gpsData;

  float currentLinearSpeed; 
  float currentAngularSpeed;

  float rightWheelEncoder;
  float leftWheelEncoder;

  bool convertGPS;
  float gpsConvertedX;
  float gpsConvertedY;

  //design decision later

  float imuYaw;
  float imuRoll;
  float imuPitch;
  //redundnacy
  float odomAngle;
  float oldOdomAngle;
  float gpsComputedVelocity;
  float computedAngularVelocity;
  float computedVelocity;

  landshark::GpsProjection gpsProjection;
  float SF_long;
  float SF_lat;
  bool gpsMode;

  //msg authentication 
  msg_auth::Publisher authCommandVelPub;
  //msg_auth::MsgAuth m_MsgAuth;
  //bool m_UseMsgAuth;

  void initGPSProjection(float sf_long =  -122.429988645 ,float sf_lat =  37.78298205);
  // call back functions
  // wheel odometry call back
  void wheelOdomCallBack(const landshark_msgs::WheelEncoder::ConstPtr& pMsg);

  // GPS call back
  void gpsOriginalCallBack(const geometry_msgs::PointStamped::ConstPtr& pMsg);

  void imuRPYCallBack(const geometry_msgs::Point::ConstPtr& pMsg);

  void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& pMsg);

  void imuCallBack(const sensor_msgs::Imu::ConstPtr& pMsg);

  // Odometry call back
  void odomCallBack(const nav_msgs::Odometry::ConstPtr& pMsg);

  void displayGPS();

  void displayOdomXYZ();
  void displayOdomQuat();
  void displayAngle();
  void getGPSDelta(float &dx, float &dy, float &dz);
  void displayGPSDelta(float &dx, float &dy, float &dz);
  void displayGPSDelta();
  void displayIMU();
  void displayImuAngle();

  void convertGPSToXY(float &x, float &y);
  void convertGPSToXY(float &x,float &y,float sf_long,float sf_lat);
  void convertGPSToXY();
  void convertAnyGPSToXY(float lon, float lat, float alt, float &x, float&y); 
  void convertAnyGPSToXY(float lon, float lat, float alt, float &x, float &y, float sf_long, float sf_lat);

  void displayConvertedGPS();

  float getDistanceBetweenPoints3(float x1, float y1, float z1, float x2, float y2, float z2); 

  getROSParameters(ros::NodeHandle nh);
  getROSParameters();
  void initClass(ros::NodeHandle nh);

  void moveRobot(float linearVelocity = 0, float angularVelocity = 0);
  void moveTurret(float turretPan = 0, float turretTilt = 0); 
  void moveMoog(float moogPan = 0, float moogTilt = 0);
  void moveRobotAuth(float linearVelocity = 0, float angularVelocity = 0);
  float getHeading2(geometry_msgs::PointStamped target, geometry_msgs::PointStamped robot, float theta, float v,
                    float w);
  float getHeading2(geometry_msgs::PointStamped target, geometry_msgs::Point robot, float theta, float v,
                     float w);
  float getHeading2(float xt, float yt, float zt, float xr, float yr, float zr, float thetar, float v, float w);
  float getOtherHeading(float heading);
  float rad2deg(float rad);
  
  void convertGPSToggle();

  };
  #endif
