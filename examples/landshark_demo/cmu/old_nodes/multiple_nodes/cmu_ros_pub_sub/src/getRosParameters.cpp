/*
 *      getRosParameters.cpp
 *
 *	Subscribes to most sensors on the landshark
 *	Publishes to the base_velocity, turret and moog control signals
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */
#include "cmu_ros_pub_sub/getRosParameters.h"

#include <exception>
//#define DEBUGTopics
using namespace std;

// call back functions
// wheel odometry call back
void getROSParameters::wheelOdomCallBack(const landshark_msgs::WheelEncoder::ConstPtr& pMsg)
{
#ifdef DEBUGTopics
  ROS_INFO( "Received Wheel Odometry message (seq=%d, stamp=%.2f)", pMsg->header.seq, pMsg->header.stamp.toSec());
#endif
  rightWheelEncoder = pMsg->right_encoder;
  leftWheelEncoder = pMsg->left_encoder;
}

// GPS call back
void getROSParameters::gpsOriginalCallBack(const geometry_msgs::PointStamped::ConstPtr& pMsg)
{
#ifdef DEBUGTopics
  ROS_INFO( "Received GPS message (seq=%d, stamp=%.2f)", pMsg->header.seq, pMsg->header.stamp.toSec());
#endif
  oldGpsPosition = gpsOriginalPosition;
  gpsOriginalPosition = *pMsg;
  if(!gpsMode)
	{
		gpsPosition = gpsOriginalPosition;
	}
  //velocity
  float dx = gpsOriginalPosition.point.x - oldGpsPosition.point.x;
  float dy = gpsOriginalPosition.point.y - oldGpsPosition.point.y;
  float dz = gpsOriginalPosition.point.z - oldGpsPosition.point.z;
  float dt = gpsOriginalPosition.header.stamp.sec - oldGpsPosition.header.stamp.sec;
  if (dt != 0)
  {
    float distance = sqrt(dx * dx + dy * dy/*+dz*dz*/);
    gpsComputedVelocity = distance / dt;
  }

}
void getROSParameters::gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr &pMsg)
{
#ifdef DEBUGTopics
  ROS_INFO("Recieved GPS");
#endif
  gpsData = *pMsg;
  if(convertGPS)
  {
    convertGPSToXY(gpsConvertedX,gpsConvertedY);
  }
  if(gpsMode)
	{
		gpsPosition.header.stamp = ros::Time::now();
		gpsPosition.point.x = gpsConvertedX; 
		gpsPosition.point.y = gpsConvertedY; 
		gpsPosition.point.z = 0; 
	}
}
/*
 * http://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/
 * #include "tf/transform_datatypes.h"
 * #include "LinearMath/btMatrix3x3.h"
 * // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
 tf::Quaternion quat;
 tf::quaternionMsgToTF(msg, quat);

 // the tf::Quaternion has a method to acess roll pitch and yaw
 double roll, pitch, yaw;
 tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

 // the found angles are written in a geometry_msgs::Vector3
 geometry_msgs::Vector3 rpy;
 rpy.x = roll;
 rpy.y = pitch;
 rpy.z = yaw;
 */
void getROSParameters::imuCallBack(const sensor_msgs::Imu::ConstPtr &pMsg)
{
#ifdef DEBUGTopics
  ROS_INFO("Received IMU");
#endif
  tf::Quaternion quat;
  tf::quaternionMsgToTF(pMsg->orientation, quat);
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  imuYaw = yaw;
  imuRoll = roll;
  imuPitch = pitch;
  imuData = *pMsg;
  float piby2 = (float)(float(22.0)/float(14)); 
  float piby2UpperLim = piby2; 
  float piby2LowerLim = -2*piby2;
  float toAdd = 0; 
//  if( imuYaw >= piby2LowerLim && imuYaw <= piby2UpperLim) 
//	toAdd = piby2; 
//  else 
//	toAdd = -3*piby2; 
  odomAngle = imuYaw + toAdd; 
  if(!gpsMode) 
	odomAngle = -1*odomAngle;

//  std::cout<<"\nI was here\n";
}

void getROSParameters::imuRPYCallBack(const geometry_msgs::Point::ConstPtr& pMsg)
{
//#ifdef DEBUGTopics
//  ROS_INFO("Imu ");
//#endif
//  oldOdomAngle = odomAngle;
//  imuRPY = *pMsg;
//  oldOdomAngle = pMsg->z;
//  float dt = odomPosition.header.stamp.sec - oldOdomPosition.header.stamp.sec;
//  if (dt != 0)
 // {
 //   computedAngularVelocity = (odomAngle - oldOdomAngle) / dt;
 // }

}

// Odometry call back
void getROSParameters::odomCallBack(const nav_msgs::Odometry::ConstPtr& pMsg)
{
#ifdef DEBUGTopics
  ROS_INFO( "Received Odometry message (seq=%d, stamp=%.2f)", pMsg->header.seq, pMsg->header.stamp.toSec());
#endif
  oldOdomPosition = odomPosition;
  odomPosition.point = pMsg->pose.pose.position;
  odomPosition.header = pMsg->header;
  odomOrientation = pMsg->pose.pose.orientation;

  speedFromOdometry = pMsg->twist.twist;
  currentLinearSpeed = pMsg->twist.twist.linear.x;
  currentAngularSpeed = pMsg->twist.twist.angular.z;
}

void getROSParameters::displayGPS()
{
  printf("\nGPS: x: %f, y: %f, z: %f\n", gpsPosition.point.x, gpsPosition.point.y, gpsPosition.point.z);
}

void getROSParameters::displayOdomXYZ()
{
  printf("\nOdom XYZ: x: %f, y: %f, z: %f\n", odomPosition.point.x, odomPosition.point.y, odomPosition.point.z);

}
void getROSParameters::displayOdomQuat()
{
  printf(
      "\nOdom Quat: x: %f, y: %f, z: %f, w: %f\n", odomOrientation.x, odomOrientation.y, odomOrientation.z, odomOrientation.w);
}
void getROSParameters::displayAngle()
{
  printf("\nAngle in a sun: %f\n", odomAngle);
}
void getROSParameters::displayIMU()
{
  printf("\nImu in a sun: %f %f %f %f\nImu Converted: roll: %f yaw: %f pitch: %f\n", imuData.orientation.x, imuData.orientation.y,
         imuData.orientation.z, imuData.orientation.w, imuRoll, imuYaw, imuPitch);
}
void getROSParameters::displayImuAngle()
{
  printf("\nAngle not in a bun: %f\nImu: %f %f %f %f\nImu Converted: roll: %f yaw: %f pitch: %f\n", odomAngle, imuData.orientation.x,
         imuData.orientation.y, imuData.orientation.z, imuData.orientation.w, imuRoll, imuYaw, imuPitch);
}
void getROSParameters::getGPSDelta(float &dx, float &dy, float &dz)
{
  dx = gpsPosition.point.x - oldGpsPosition.point.x;
  dy = gpsPosition.point.y - oldGpsPosition.point.y;
  dz = gpsPosition.point.z - oldGpsPosition.point.z;
}

void getROSParameters::displayGPSDelta(float &dx, float &dy, float &dz)
{
  getGPSDelta(dx, dy, dz);
  //////printf("\nGPS D: %f, %f, %f\n", dx, dy, dz);
}
void getROSParameters::displayGPSDelta()
{
  float dx = 0;
  float dy = 0;
  float dz = 0;
  getGPSDelta(dx, dy, dz);
  //////printf("\nGPS D: %f, %f, %f\n", dx, dy, dz);
}
getROSParameters::getROSParameters()
{
}
void getROSParameters::initClass(ros::NodeHandle nh)
{
  // initializing all variables
  nodeHandle = nh;
  gpsOriginalTopicName = "/landshark/gps_original";
  gpsTopicName = "/landshark/gps";
  wheelOdomTopicName = "/landshark/wheel_encoder";
  basePublishTopic = "/landshark_control/base_velocity";
  odomTopicName = "/landshark/odom";
  imuRPYTopicName = "/landshark/imu_rpy";
  imuTopicName = "/landshark/imu";
  moogPublishTopic = "/landshark_control/moog_joint_velocity";
  turretPublishTopic = "/landshark_control/turret_joint_velocity";
 
  speedTwist.twist.linear.x = 0;
  speedTwist.twist.linear.y = 0;
  speedTwist.twist.linear.z = 0;
  speedTwist.twist.angular.x = 0;
  speedTwist.twist.angular.y = 0;
  speedTwist.twist.angular.z = 0;

  imuYaw = 0;
  imuRoll = 0;
  imuPitch = 0;
  odomAngle = 0;

  initGPSProjection();
  convertGPS = false;
  //timeCycleInSeconds = 0.25; //from the paper for now
  //initializing subbscribers and publishers

  //Get gps topic from param server
  nodeHandle.getParam("gps_topic", gpsTopicName);
  // subscribe to topic
  gpsTopSubscriber = nodeHandle.subscribe<sensor_msgs::NavSatFix>(gpsTopicName, 1, &getROSParameters::gpsCallBack,
                                                                  this);

  gpsOriginalTopSubscriber = nodeHandle.subscribe<geometry_msgs::PointStamped>(gpsOriginalTopicName, 1,
                                                                               &getROSParameters::gpsOriginalCallBack,
                                                                               this);

  //get wheelodom topic from server
  nodeHandle.getParam("wheelodom_topic", wheelOdomTopicName);
  //subscribe to topic
  wheelOdomTopSubscriber = nodeHandle.subscribe<landshark_msgs::WheelEncoder>(wheelOdomTopicName, 1,
                                                                              &getROSParameters::wheelOdomCallBack,
                                                                              this);

  //subscribe to odometry topic
  odomTopSubscriber = nodeHandle.subscribe<nav_msgs::Odometry>(odomTopicName, 1, &getROSParameters::odomCallBack, this);

  imuTopSubscriber = nodeHandle.subscribe<sensor_msgs::Imu>(imuTopicName, 1, &getROSParameters::imuCallBack, this);

  imuRPYTopSubscriber = nodeHandle.subscribe<geometry_msgs::Point>(imuRPYTopicName, 1,
                                                                   &getROSParameters::imuRPYCallBack, this);
  // create a publisher for the motion topic
  baseTopPublisher = nodeHandle.advertise<geometry_msgs::TwistStamped>(basePublishTopic, 1);
  //gpsLoc = nodeHandle.advertise<geometry_msgs::TwistStamped>(gpsTopicName,1);

moogTopPublisher = nodeHandle.advertise<landshark_msgs::JointControl>(moogPublishTopic,1); 

  turretTopPublisher = nodeHandle.advertise<landshark_msgs::JointControl>(turretPublishTopic,1);

// msg_auth::MsgAuth msgAuth(nodeHandle); 
 //authCommandVelPub = msgAuth.advertise<geometry_msgs::TwistStamped>(basePublishTopic,1);//authBasePublishTopic,1);


  turretControl.name.push_back("turret_pan"); 
  turretControl.name.push_back("turrent_tilt"); 
  turretControl.value.resize(2);
  
  moogControl.name.push_back("moog_pan"); 
  moogControl.name.push_back("moog_tilt"); 
  moogControl.value.resize(2); 


}

getROSParameters::getROSParameters(ros::NodeHandle nh)
{

  // initializing all variables
  nodeHandle = nh;
  gpsOriginalTopicName = "/landshark/gps_original";
  gpsTopicName = "/landshark/gps";
  wheelOdomTopicName = "/landshark/wheel_encoder";
  basePublishTopic = "/landshark_control/base_velocity";
  odomTopicName = "/landshark/odom";
  imuRPYTopicName = "/landshark/imu_rpy";
  imuTopicName = "/landshark/imu";
  moogPublishTopic = "/landshark_control/moog_joint_velocity";
  turretPublishTopic = "/landshark_control/turret_joint_velocity";

  

  speedTwist.twist.linear.x = 0;
  speedTwist.twist.linear.y = 0;
  speedTwist.twist.linear.z = 0;
  speedTwist.twist.angular.x = 0;
  speedTwist.twist.angular.y = 0;
  speedTwist.twist.angular.z = 0;


  imuYaw = 0;
  imuRoll = 0;
  imuPitch = 0;
  odomAngle = 0;

  initGPSProjection();
  convertGPS = true;
  //timeCycleInSeconds = 0.25; //from the paper for now
  //initializing subbscribers and publishers

  //Get gps topic from param server
  nodeHandle.getParam("gps_topic", gpsTopicName);
  // subscribe to topic
  gpsTopSubscriber = nodeHandle.subscribe<sensor_msgs::NavSatFix>(gpsTopicName, 1, &getROSParameters::gpsCallBack,
                                                                  this);

  gpsOriginalTopSubscriber = nodeHandle.subscribe<geometry_msgs::PointStamped>(gpsOriginalTopicName, 1,
                                                                               &getROSParameters::gpsOriginalCallBack,
                                                                               this);

  //get wheelodom topic from server
  nodeHandle.getParam("wheelodom_topic", wheelOdomTopicName);
  //subscribe to topic
  wheelOdomTopSubscriber = nodeHandle.subscribe<landshark_msgs::WheelEncoder>(wheelOdomTopicName, 1,
                                                                              &getROSParameters::wheelOdomCallBack,
                                                                              this);

  //subscribe to odometry topic
  odomTopSubscriber = nodeHandle.subscribe<nav_msgs::Odometry>(odomTopicName, 1, &getROSParameters::odomCallBack, this);

  imuTopSubscriber = nodeHandle.subscribe<sensor_msgs::Imu>(imuTopicName, 1, &getROSParameters::imuCallBack, this);

  imuRPYTopSubscriber = nodeHandle.subscribe<geometry_msgs::Point>(imuRPYTopicName, 1,
                                                                   &getROSParameters::imuRPYCallBack, this);
  // create a publisher for the motion topic
  baseTopPublisher = nodeHandle.advertise<geometry_msgs::TwistStamped>(basePublishTopic, 1);
  //gpsLoc = nodeHandle.advertise<geometry_msgs::TwistStamped>(gpsTopicName,1);
 
  moogTopPublisher = nodeHandle.advertise<landshark_msgs::JointControl>(moogPublishTopic,1); 
  
  turretTopPublisher = nodeHandle.advertise<landshark_msgs::JointControl>(turretPublishTopic,1);

 //msg_auth::MsgAuth msgAuth(nodeHandle); 
 //authCommandVelPub = msgAuth.advertise<geometry_msgs::TwistStamped>(basePublishTopic,1);//authBasePublishTopic,1);
   

  turretControl.name.push_back("turret_pan"); 
  turretControl.name.push_back("turrent_tilt"); 
  turretControl.value.resize(2);
  
  moogControl.name.push_back("moog_pan"); 
  moogControl.name.push_back("moog_tilt"); 
  moogControl.value.resize(2); 

}

void getROSParameters::moveRobot(float linearVelocity, float angularVelocity)
{
  speedTwist.twist.linear.x = linearVelocity;
  speedTwist.twist.angular.z = angularVelocity;
  baseTopPublisher.publish(speedTwist);
}

void getROSParameters::moveRobotAuth(float linearVelocity, float angularVelocity)
{
  speedTwist.twist.linear.x = linearVelocity;
  speedTwist.twist.angular.z = angularVelocity;
 // authCommandVelPub.publish(speedTwist);
}


void getROSParameters::moveTurret(float turretPan, float turretTilt)
{
  turretControl.header.stamp = ros::Time::now();
  turretControl.mode = landshark_msgs::JointControl::VELOCITY_CONTROL; 
//  cout<<"turretControl Size: "<<turretControl.value.size();
//  cout<<endl;
  turretControl.value[0] = turretPan; 
  turretControl.value[1] = turretTilt;	
  turretTopPublisher.publish(turretControl);
}

void getROSParameters::moveMoog(float moogPan, float moogTilt)
{
  moogControl.header.stamp = ros::Time::now();
  moogControl.mode = landshark_msgs::JointControl::VELOCITY_CONTROL; 
  moogControl.value[0] = moogPan; 
  moogControl.value[1] = moogTilt; 
  moogTopPublisher.publish(moogControl);
}

float getROSParameters::getHeading2(geometry_msgs::PointStamped target, geometry_msgs::PointStamped robot, float theta,
                                    float v, float w)
{
  float xt = target.point.x;
  float yt = target.point.y;
  float zt = target.point.z;
  float xr = robot.point.x;
  float yr = robot.point.y;
  float zr = robot.point.z;
  float thetar = theta;
  return getHeading2(xt,yt,zt,xr,yr,zr,thetar,v,w);
}
float getROSParameters::getHeading2(geometry_msgs::PointStamped target, geometry_msgs::Point robot, float theta,
                                    float v, float w)
{
  float xt = target.point.x;
  float yt = target.point.y;
  float zt = target.point.z;
  float xr = robot.x;
  float yr = robot.y;
  float zr = robot.z;
  float thetar = theta;
  return getHeading2(xt,yt,zt,xr,yr,zr,thetar,v,w);
}
float getROSParameters::getHeading2(float xt, float yt, float zt, float xr, float yr, float zr, float thetar, float v, float w)
{
  float nx = xr;
  float ny = yr;
  float ntheta = thetar;
  float dy = -1 * (yt - ny);
  float dx = -1 * (xt - nx);
  float heading2 = atan2((ny - yt), (nx - xt));
  float phi = ntheta;
  float theeta = heading2;
  float pi = 3.142;
  if (dy >= 0)
  {
    if (dx >= 0)
    {
      if (phi >= 0)
      {
        heading2 = pi - theeta - phi;
        heading2 = -1 * heading2;

      }
      else
      {
        if (phi <= -pi / 2)
        {
          phi = -1 * phi;
          heading2 = pi - phi + theeta;
        }
        else
        {
          phi = phi * -1;
          heading2 = phi + pi - theeta;
          heading2 = heading2 * -1;
        }
      }
    }
    else
    { //second quadrant
      theeta = pi - theeta;
      if (phi >= 0)
      {
        heading2 = phi - theeta;
      }
      else
      {
        phi = phi * -1;
        heading2 = phi + theeta;
        heading2 = heading2 * -1;
      }

    }
  }
  else
  {
    if (dx >= 0)
    {
      theeta = -1 * theeta;
      if (phi >= 0)
      {
        if (phi <= pi / 2)
        {
          heading2 = phi + pi - theeta;
        }
        else
        {
          heading2 = pi - phi + theeta;
          heading2 = -1 * heading2;
        }
      }
      else
      {
        heading2 = pi + phi - theeta;
      }

    }
    else
    { // 4rth quadrant
      theeta = theeta * -1;
      theeta = pi - theeta;
      if (phi >= 0)
      {
        heading2 = phi + theeta;
      }
      else
      {
        heading2 = theeta + phi;
      }

    }
  }
  return heading2;

}
float getROSParameters::rad2deg(float rad)
{
  return (rad * 180.00 / 3.1415926);
}
float getROSParameters::getDistanceBetweenPoints3(float x1, float y1, float z1, float x2, float y2, float z2)
{
  float dx = x2 - x1;
  float dy = y2 - y1;
  float dz = z2 - z1;
  float dx2 = dx * dx;
  float dy2 = dy * dy;
  float dz2 = dz * dz;
  float distance = sqrt(dx2 + dy2 + dz2);
  return distance;
}
float getROSParameters::getOtherHeading(float heading)
{
  float headingToRet = 0;
  float pi = 3.142;
  if (heading > 0)
  {
    headingToRet = -1 * (2 * pi - heading);
  }
  else
  {
    headingToRet = -(2 * pi + heading);
  }
  return headingToRet;
}
void getROSParameters::convertGPSToXY()
{
 if(gpsMode)
	{
	float x; 
	float y; 
	double temp_x,temp_y,longitude,latitude;
  longitude = (float)gpsData.longitude;
  latitude = (float)gpsData.latitude;
  try
  {
  gpsProjection.GetLocalCoordinatesFromGps(longitude,latitude,temp_x,temp_y);
  }
  catch(std::exception& exception)
  {
    temp_x = -1000;
    temp_y = -1000;
    printf("\nException");
  }
  x = (float)temp_x;
  y = (float)temp_y;
 
		//gpsPosition.point.x = x; 
		//gpsPosition.point.y = -1*y; 
		//gpsPosition.point.z = 0;
	}	
}
  void getROSParameters::convertAnyGPSToXY(float lon, float lat, float alt, float &x, float&y)
{
double temp_x,temp_y,longitude,latitude;
  longitude = (double)lon;
  latitude = (double)lat;
  try
  {
  gpsProjection.GetLocalCoordinatesFromGps(longitude,latitude,temp_x,temp_y);
  }
  catch(std::exception& exception)
  {
    temp_x = -1000;
    temp_y = -1000;
    //cout<<exception.Message;
   cout<<exception.what();
    printf("\nException");
  }
  x = (float)temp_x;
  y = (float)temp_y;
  
}
  void getROSParameters::convertAnyGPSToXY(float lon, float lat, float alt, float &x, float &y, float sf_long, float sf_lat)
{
	initGPSProjection(sf_long,sf_lat);
convertAnyGPSToXY(lon, lat, alt, x, y);
}
void getROSParameters::convertGPSToXY(float &x, float &y)
{
  double temp_x,temp_y,longitude,latitude;
  longitude = (double)gpsData.longitude;
  latitude = (double)gpsData.latitude;
  try
  {
  gpsProjection.GetLocalCoordinatesFromGps(longitude,latitude,temp_x,temp_y);
  }
  catch(std::exception& exception)
  {
    temp_x = -1000;
    temp_y = -1000;
    printf("\nException");
  }
  x = (float)temp_x;
  y = (float)temp_y;
  if(gpsMode)
	{
		//gpsPosition.point.x = x; 
		//gpsPosition.point.y = -1*y; 
		//gpsPosition.point.z = 0;
	}
}

void getROSParameters::initGPSProjection(float sf_long, float sf_lat)
{
  SF_long =  sf_long;
  SF_lat =  sf_lat;
  gpsProjection.SetLocalCoordinateSystemOrigin(SF_long,SF_lat);
}

void getROSParameters::convertGPSToXY(float &x,float &y, float SF_long_temp, float SF_lat_temp)
{
  initGPSProjection(SF_long_temp,SF_lat_temp);
  convertGPSToXY(x,y);

}

void getROSParameters::displayConvertedGPS()
{
  float x = 0;
  float y = 0;
  convertGPSToXY(x,y);
  printf("\nGPS lat: %f lon: %f \nConverted x: %f y: %f \nActual x: %f y: %f",gpsData.latitude,
         gpsData.longitude,x,y,gpsPosition.point.x,gpsPosition.point.y);
}

void getROSParameters::convertGPSToggle()
{
  convertGPS = ~convertGPS;
}
