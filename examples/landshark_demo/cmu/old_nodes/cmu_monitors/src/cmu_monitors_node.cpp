/*      
 * Author: Fatma Faruq, Jason Larkin
 * HACMS CMU Team, Advisor Manuela Veloso, SpiralGen, Inc.
 */
//--------------------------------------------------------------------------------
#include "ros/ros.h"
#include "dwmonitor.h"
#include "multi_window_motion_gps_odo_anomaly_monitor.h"

#include <string>
#include <stdlib.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <landshark_msgs/BoolStamped.h>
#include <landshark_gps/GpsProjection.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <landshark_msgs/TriggerAlarm.h>
#include <landshark_msgs/ResetAlarm.h>
//#include <cmu_control_virtualizer_stub/TriggerAlarm.h>
//#include <cmu_control_virtualizer_stub/ResetAlarm.h>

#include "cmu_monitors/obstacle.h"
//--------------------------------------------------------------------------------
sensor_msgs::NavSatFix msgGps;
geometry_msgs::PointStamped msgGpsMeters;
sensor_msgs::NavSatFix msgObstacleGps;
landshark::GpsProjection gpsProjection;

nav_msgs::Odometry msgOdom;
nav_msgs::Odometry msgOdomObstacle;

geometry_msgs::TwistStamped msgBaseVelocity;

landshark_msgs::BoolStamped msgCmuDynamicWindowMonitorStatus;
landshark_msgs::BoolStamped msgCmuSensorZtestMonitorStatus;

landshark_msgs::TriggerAlarm msgTriggerAlarm;
landshark_msgs::TriggerAlarm msgCmuDynamicWindowMonitorTrigger;
landshark_msgs::TriggerAlarm msgCmuSensorZtestMonitorTrigger;

landshark_msgs::ResetAlarm msgResetAlarm;
landshark_msgs::ResetAlarm msgCmuDynamicWindowMonitorReset;
landshark_msgs::ResetAlarm msgCmuSensorZtestMonitorReset;

landshark_msgs::BoolStamped msgCmuDynamicWindowMonitorControl;
landshark_msgs::BoolStamped msgCmuSensorZtestMonitorControl;
//--------------------------------------------------------------------------------
void getGps(\
const sensor_msgs::NavSatFix::ConstPtr& pMsg)
{msgGps = *pMsg;}

void getObstacleGps(\
const sensor_msgs::NavSatFix::ConstPtr& pMsg)
{msgObstacleGps = *pMsg;}

void getOdom(\
const nav_msgs::Odometry::ConstPtr& pMsg)
{msgOdom = *pMsg;}

void getOdomObstacle(\
const nav_msgs::Odometry::ConstPtr& pMsg)
{msgOdomObstacle = *pMsg;}

void getBaseVelocity(\
const geometry_msgs::TwistStamped::ConstPtr& pMsg)
{msgBaseVelocity = *pMsg;}

void getCmuDynamicWindowMonitorReset(\
const landshark_msgs::ResetAlarm::ConstPtr& pMsg)
{msgCmuDynamicWindowMonitorReset = *pMsg;}

void getCmuDynamicWindowMonitorControl(\
const landshark_msgs::BoolStamped::ConstPtr& pMsg)
{msgCmuDynamicWindowMonitorControl = *pMsg;}

void getCmuSensorZtestMonitorReset(\
const landshark_msgs::ResetAlarm::ConstPtr& pMsg)
{msgCmuSensorZtestMonitorReset = *pMsg;}

void getCmuSensorZtestMonitorControl(\
const landshark_msgs::BoolStamped::ConstPtr& pMsg)
{msgCmuSensorZtestMonitorControl = *pMsg;}

void getResetAlarm(\
const landshark_msgs::ResetAlarm::ConstPtr& pMsg)
{msgResetAlarm = *pMsg;}

void okayToGo(\
float car_x, float car_y, float obs_x, float obs_y,\
float v, float w, float dt, float max_accv,\
float inner_radius, float outer_radius, 
bool& okayToGoBool1, bool& okayToGoBool2)
{
  
  double f_v = fabs(v);//f_V = maxObsVel;
  float accv = max_accv;
  float timePeriod = dt;
  float X[5];
  
  X[0] = fabs((float)v);
  X[1] = (float)car_x; X[2] = (float)car_y;
  X[3] = (float)obs_x; X[4] = (float)obs_y;
  double d0 = (double)((accv / accv + 1) *\
  (accv / 2 * timePeriod * timePeriod + f_v * timePeriod));
  double D[3];
  D[0] = d0 + outer_radius;
  D[1] = (double)((f_v / accv) + timePeriod * (accv / accv + 1));
  D[2] = (double)(1 / (2 * accv));

  int result = dwmonitor(X, D);
  
//  if (result == 1)
//  {
//    okayToGob = true;
//  }
  
  if (result != 1)
  {  
//you are going to be unsafe soon
    okayToGoBool1 = false; 
    D[0] = d0 + inner_radius;
    int result2 = dwmonitor(X, D);
//    printf("\n Conservative Not Passed");
    if (result2 == 1)
    {
//      printf("\n-------- back up you are too close to the obstacle -----");
      okayToGoBool2 = true;
    }
    else{okayToGoBool2 =false;}
  }
  else{okayToGoBool1 = true;okayToGoBool2 =true;}
  
return;
}

//--------------------------------------------------------------------------------
//from jp node
//--------------------------------------------------------------------------------
//rate at which monitors run
const unsigned int LOOPING_RATE = 20;
//only declare a failure if above this confidence
double anomaly_thresh = 0.7;
//play alarm sound?
bool play_alarm_sound = false;
//is the execution monitor currently paused?
bool paused = false;

MotionGpsOdoAnomalyMonitor motionMonitor;
//ros::Publisher faultPublisher;

void timerEvent() {
  if (!paused) motionMonitor.run();
//  printf("anomaly:%f\n",motionMonitor.getAnomaly());
  if(motionMonitor.getAnomaly()>anomaly_thresh && !paused){
    paused = true;
    //publish failure message
    //landshark_msgs::ExecutionFault faultMsg;
    //faultMsg.header.stamp = ros::Time::now();
    //faultMsg.faultInfo = "Motion failure detected";
    //faultPublisher.publish(faultMsg);
    if(play_alarm_sound) system("play ~/ros_groovy_base/cmu/data/alarm.wav");
    //printf("%f,%s\n",FAULT_ANOMALY_THRESH,motionMonitor.resultS);
    //exit(0);
  }
}
//--------------------------------------------------------------------------------
//from jp node
//--------------------------------------------------------------------------------

//--------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmu_monitors");
  ros::NodeHandle node;

  std::string topicGps;
  std::string topicGpsObstacle;
  std::string topicGpsMeters;
  
  std::string topicBaseVelocity;
  std::string topicOdom;

  std::string topicCmuDynamicWindowMonitorStatus;
  std::string topicCmuSensorZtestMonitorStatus;
  
  std::string topicCmuDynamicWindowMonitorTrigger;
  std::string topicCmuSensorZtestMonitorTrigger;
  
  std::string topicCmuDynamicWindowMonitorControl;
  std::string topicCmuSensorZtestMonitorControl;
  
  std::string topicCmuDynamicWindowMonitorReset;
  std::string topicCmuSensorZtestMonitorReset;
  
  std::string topicResetAlarm;
  std::string topicTriggerAlarm;
  
  bool latch;
  std::string paramName;
  
//  double origin_lat = 37.457119; double origin_lon = -122.173513;  
  double origin_lat = 37.4571014863; double origin_lon = -122.173496991;
  
  double clearanceDist1 = 0; double clearanceDist2 = 0;
  double accel_lin_max = 4.0; double accel_ang_max = 4.0; 
  double max_obs_vel = 1.0; double max_obs_dist = 10.0; 
  double vel_lin_max = 1.0; double vel_ang_max = 1.0; 
  double epsilon = 0.25; double dt = 0.05;
  double outer_radius = 10.0; double inner_radius = 2.5;
//set obstacle far away, same as during a ResetAlarm  
  double obstacle_meters_x_reset = 10000.0; 
  double obstacle_meters_y_reset = 10000.0;
  double obstacle_meters_x = 10000.0; 
  double obstacle_meters_y = 10000.0; 
  double obstacle_gps_x_reset = 40.443181; 
  double obstacle_gps_y_reset = -79.943060;
  double obstacle_gps_x = 40.443181; 
  double obstacle_gps_y = -79.943060; 
  double robot_meters_x,robot_meters_y;
  
//--------------------------------------------------------------------------------
//input params
//--------------------------------------------------------------------------------
  paramName = "origin_lat";
  if( !node.hasParam(paramName))
	 {ROS_INFO("Can't get %s from parameter server. Setting to default value %f",
		 paramName.c_str(),origin_lat);
	 }
	 else
	 node.getParam(paramName,origin_lat);
	 
	 paramName = "origin_lon";
  if( !node.hasParam(paramName))
	 {ROS_INFO("Can't get %s from parameter server. Setting to default value %f",
		 paramName.c_str(),origin_lon);
	 }
	 else
	 node.getParam(paramName,origin_lon);	
	 
	 paramName = "obstacle_meters_x_reset";
  if( !node.hasParam(paramName))
	 {ROS_INFO("Can't get %s from parameter server. Setting to default value %f",
		 paramName.c_str(),obstacle_meters_x_reset);
	 }
	 else
	 node.getParam(paramName,obstacle_meters_x_reset);	
	 
	 paramName = "obstacle_meters_y_reset";
  if( !node.hasParam(paramName))
	 {ROS_INFO("Can't get %s from parameter server. Setting to default value %f",
		 paramName.c_str(),obstacle_meters_y_reset);
	 }
	 else
	 node.getParam(paramName,obstacle_meters_y_reset);			 
	 	 
  paramName = "dt";
  if( !node.hasParam(paramName))
	   {
		  ROS_INFO(
		  "Can't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),dt);
	   }
	 else
	 node.getParam(paramName,dt);
	 
  if( !node.hasParam("outer_radius"))
	 {
		  ROS_INFO(
		  "Can't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),25.0);
		  outer_radius = 25.0;
	 }
	 else
	 node.getParam("outer_radius",outer_radius);

  paramName = "inner_radius";
  if( !node.hasParam(paramName))
	   {
		  ROS_INFO(
		  "Can't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),2.5);
		  inner_radius = 2.5;
 	  }
	 else
	 node.getParam(paramName,inner_radius);
	 
  paramName = "accel_lin_max";
  if( !node.hasParam(paramName))
	   {
		  ROS_INFO(
		  "Can't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),accel_lin_max);
	   }
	 else
	 node.getParam(paramName,accel_lin_max);

  paramName = "accel_ang_max";
  if( !node.hasParam(paramName))
	 {
		  ROS_INFO(
		  "Can't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),accel_ang_max);
	 }
	 else
	 node.getParam(paramName,accel_ang_max);

  paramName = "vel_ang_max";
  if( !node.hasParam(paramName))
	 {
		  ROS_INFO(
		  "Can't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),vel_ang_max);
	 }
	 else
	 node.getParam(paramName,vel_ang_max);

  paramName = "vel_lin_max";
  if( !node.hasParam(paramName))
	 {
		  ROS_INFO(
		  "Can't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),vel_lin_max);
	 }
	 else
	 node.getParam(paramName,vel_lin_max);

  paramName = "max_obs_vel";
  if( !node.hasParam(paramName))
	 {
		  ROS_INFO(
		  "Can't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),max_obs_vel);
	 }
	 else
	 node.getParam(paramName,max_obs_vel);

  paramName = "max_obs_dist";
  if( !node.hasParam(paramName))
	 {
		  ROS_INFO(
		  "Can't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),max_obs_dist);
	 }
	 else
	 node.getParam(paramName,max_obs_dist);

  paramName = "epsilon";
  if( !node.hasParam(paramName))
	 {
		  ROS_INFO(
		  "Can't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),epsilon);
	 }
	 else
	 node.getParam(paramName,epsilon); 

//--------------------------------------------------------------------------------
//from jp node
//--------------------------------------------------------------------------------	 
	 paramName = "max_anomaly";
  if( !node.hasParam(paramName))
	 {
		  ROS_INFO(
		  "Can't get %s from parameter server. Setting to default value %f",
		  paramName.c_str(),anomaly_thresh);
	 }
	 else
	 node.getParam(paramName,anomaly_thresh); 
	 
	 printf("\nmotionMonitor.init(&node);\n");	 
	 motionMonitor.init(&node, origin_lon, origin_lat);

  printf("\norigin_lon = %f origin_lat = %f \n",origin_lon,origin_lat);

//  n.param("/cmu_sensor_monitor/max_anomaly", anomaly_thresh,0.9);
//  n.param("/cmu_sensor_monitor/play_alarm_sound", play_alarm_sound,false); 
//--------------------------------------------------------------------------------
//from jp node
//--------------------------------------------------------------------------------  
  
//--------------------------------------------------------------------------------
//topics  
//--------------------------------------------------------------------------------
  paramName = "/landshark/gps";
  if (!node.hasParam(paramName))
  {
    ROS_INFO("Can't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicGps = paramName.c_str();
    ROS_INFO("Publishing to %s",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicGps);
    ROS_INFO("Subscribing to %s",paramName.c_str());
  }
  ros::Subscriber pubGps =\
  node.subscribe < sensor_msgs::NavSatFix > \
  (topicGps, 1, getGps);
//--------------------------------------------------------------------------------
  paramName = "/landshark/gps_meters";
  if (!node.hasParam(paramName))
  {
    ROS_INFO("Can't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicGpsMeters = paramName.c_str();
    ROS_INFO("Publishing to %s",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicGpsMeters);
    ROS_INFO("Subscribing to %s",paramName.c_str());
  }
  latch = true;
  ros::Publisher pubGpsMeters =\
  node.advertise < geometry_msgs::PointStamped > \
  (topicGpsMeters, 1, latch);    
//--------------------------------------------------------------------------------
  paramName = "/landshark/cmu_dynamic_window_monitor/obstacle";
  if (!node.hasParam(paramName))
  {
    ROS_INFO("Can't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicGpsObstacle = paramName.c_str();
    ROS_INFO("Publishing to %s",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicGpsObstacle);
    ROS_INFO("Subscribing to %s",paramName.c_str());
  }
  ros::Subscriber pubGpsObstacle =\
  node.subscribe < sensor_msgs::NavSatFix > \
  (topicGpsObstacle, 1, getObstacleGps);
//--------------------------------------------------------------------------------
  paramName = "/landshark_control/base_velocity";
  if (!node.hasParam(paramName))
  {
    ROS_INFO("Can't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicBaseVelocity = paramName.c_str();
    ROS_INFO("Publishing to %s",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicBaseVelocity);
    ROS_INFO("Subscribing to %s",paramName.c_str());
  }
  ros::Subscriber pubBaseVelocity =\
  node.subscribe < geometry_msgs::TwistStamped > \
  (topicBaseVelocity, 1, getBaseVelocity); 
//--------------------------------------------------------------------------------  
  paramName = "/landshark/odom";
  if (!node.hasParam(paramName))
  {
    ROS_INFO("Can't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicOdom = paramName.c_str();
    ROS_INFO("Publishing to %s",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicOdom);
    ROS_INFO("Subscribing to %s",paramName.c_str());
  }
  ros::Subscriber pubOdom =\
  node.subscribe < nav_msgs::Odometry > \
  (topicOdom, 1, getOdom); 
//--------------------------------------------------------------------------------
  paramName = "/landshark/cmu_dynamic_window_monitor/status";
  if (!node.hasParam(paramName))
  {
    ROS_INFO("Can't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicCmuDynamicWindowMonitorStatus = paramName.c_str();
    ROS_INFO("Publishing to %s",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicCmuDynamicWindowMonitorStatus);
    ROS_INFO("Subscribing to %s",paramName.c_str());
  }
  latch = true;
  ros::Publisher pubCmuDynamicWindowMonitorStatus =\
  node.advertise < landshark_msgs::BoolStamped > \
  (topicCmuDynamicWindowMonitorStatus, 1, latch);
//--------------------------------------------------------------------------------
  paramName = "/landshark/cmu_sensor_ztest_monitor/status";
  if (!node.hasParam(paramName))
  {
    ROS_INFO("Can't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicCmuSensorZtestMonitorStatus = paramName.c_str();
    ROS_INFO("Publishing to %s",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicCmuSensorZtestMonitorStatus);
    ROS_INFO("Subscribing to %s",paramName.c_str());
  }
  latch = true;
  ros::Publisher pubCmuSensorZtestMonitorStatus =\
  node.advertise < landshark_msgs::BoolStamped > \
  (topicCmuSensorZtestMonitorStatus, 1, latch);
//--------------------------------------------------------------------------------
//  paramName = "/landshark/cmu_dynamic_window_monitor/trigger";
//  if (!node.hasParam(paramName))
//  {
//    printf("\nCan't get %s from parameter server. Setting to default value %s",\
//    paramName.c_str(),paramName.c_str());
//    topicCmuDynamicWindowMonitorTrigger = paramName.c_str();
//    printf("\nPublishing to %s\n",paramName.c_str());
//  }
//  else
//  {
//    node.getParam(paramName.c_str(), topicCmuDynamicWindowMonitorTrigger);
//    printf("\nSubscribing to %s\n",paramName.c_str());
//  }
//  latch = true;
//  ros::Publisher pubCmuDynamicWindowMonitorTrigger =\
//  node.advertise < landshark_msgs::TriggerAlarm > \
//  (topicCmuDynamicWindowMonitorTrigger, 1, latch);
//--------------------------------------------------------------------------------
//  paramName = "/landshark/cmu_sensor_ztest_monitor/trigger";
//  if (!node.hasParam(paramName))
//  {
//    printf("\nCan't get %s from parameter server. Setting to default value %s",\
//    paramName.c_str(),paramName.c_str());
//    topicCmuSensorZtestMonitorTrigger = paramName.c_str();
//    printf("\nPublishing to %s\n",paramName.c_str());
//  }
//  else
//  {
//    node.getParam(paramName.c_str(), topicCmuSensorZtestMonitorTrigger);
//    printf("\nSubscribing to %s\n",paramName.c_str());
//  }
//  latch = true;
//  ros::Publisher pubCmuSensorZtestMonitorTrigger =\
//  node.advertise < landshark_msgs::TriggerAlarm > \
//  (topicCmuSensorZtestMonitorTrigger, 1, latch);
//--------------------------------------------------------------------------------
  paramName = "/landshark_control/trigger_alarm";
  if (!node.hasParam(paramName))
  {
    ROS_INFO("Can't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicTriggerAlarm = paramName.c_str();
    ROS_INFO("Publishing to %s",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicTriggerAlarm);
    ROS_INFO("Subscribing to %s",paramName.c_str());
  }
  latch = false;
  ros::Publisher pubTriggerAlarm =\
  node.advertise < landshark_msgs::TriggerAlarm > \
  (topicTriggerAlarm, 1, latch);
//--------------------------------------------------------------------------------
//  paramName = "/landshark/cmu_dynamic_window_monitor/reset";
//  if (!node.hasParam(paramName))
//  {
//    printf("\nCan't get %s from parameter server. Setting to default value %s",\
//    paramName.c_str(),paramName.c_str());
//    topicCmuDynamicWindowMonitorReset = paramName.c_str();
//    printf("\nPublishing to %s\n",paramName.c_str());
//  }
//  else
//  {
//    node.getParam(paramName.c_str(), topicCmuDynamicWindowMonitorReset);
//    printf("\nSubscribing to %s\n",paramName.c_str());
//  }
//  ros::Subscriber subCmuDynamicWindowMonitorReset =\
//  node.subscribe < landshark_msgs::ResetAlarm > \
//  (topicCmuDynamicWindowMonitorReset, 1, getCmuDynamicWindowMonitorReset);
//--------------------------------------------------------------------------------
//  paramName = "/landshark/cmu_sensor_ztest_monitor/reset";
//  if (!node.hasParam(paramName))
//  {
//    printf("\nCan't get %s from parameter server. Setting to default value %s",\
//    paramName.c_str(),paramName.c_str());
//    topicCmuSensorZtestMonitorReset = paramName.c_str();
//    printf("\nPublishing to %s\n",paramName.c_str());
//  }
//  else
//  {
//    node.getParam(paramName.c_str(), topicCmuSensorZtestMonitorReset);
//    printf("\nSubscribing to %s\n",paramName.c_str());
//  }
//  ros::Subscriber subCmuSensorZtestMonitorReset =\
//  node.subscribe < landshark_msgs::ResetAlarm > \
//  (topicCmuSensorZtestMonitorReset, 1, getCmuSensorZtestMonitorReset);
//--------------------------------------------------------------------------------
  paramName = "/landshark_control/reset_alarm";
  if (!node.hasParam(paramName))
  {
    ROS_INFO("Can't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicResetAlarm = paramName.c_str();
    ROS_INFO("Publishing to %s",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicResetAlarm);
    ROS_INFO("Subscribing to %s",paramName.c_str());
  }
  ros::Subscriber subResetAlarm =\
  node.subscribe < landshark_msgs::ResetAlarm > \
  (topicResetAlarm, 1, getResetAlarm);  
//--------------------------------------------------------------------------------
  paramName = "/landshark/cmu_dynamic_window_monitor/control";
  if (!node.hasParam(paramName))
  {
    ROS_INFO("Can't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicCmuDynamicWindowMonitorControl = paramName.c_str();
    ROS_INFO("Publishing to %s",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicCmuDynamicWindowMonitorControl);
    ROS_INFO("Subscribing to %s",paramName.c_str());
  }
  ros::Subscriber subCmuDynamicWindowMonitorControl =\
  node.subscribe < landshark_msgs::BoolStamped > \
  (topicCmuDynamicWindowMonitorControl, 1, getCmuDynamicWindowMonitorControl);
//--------------------------------------------------------------------------------
  paramName = "/landshark/cmu_sensor_ztest_monitor/control";
  if (!node.hasParam(paramName))
  {
    ROS_INFO("Can't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicCmuSensorZtestMonitorControl = paramName.c_str();
    ROS_INFO("Publishing to %s",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicCmuSensorZtestMonitorControl);
    ROS_INFO("Subscribing to %s",paramName.c_str());
  }
  ros::Subscriber subCmuSensorZtestMonitorControl =\
  node.subscribe < landshark_msgs::BoolStamped > \
  (topicCmuSensorZtestMonitorControl, 1, getCmuSensorZtestMonitorControl);
  
//--------------------------------------------------------------------------------

  bool okayBoolOuter; //= false;
  bool dwmonitorOkayBool; //= false; 
  bool sensorZtestOkayBool = true;
//initialize obstacle like a reset (far away)  
  msgObstacleGps.latitude = obstacle_gps_x_reset;
  msgObstacleGps.longitude = obstacle_gps_y_reset;

//sleep 1 sec to let topics get publish to
  ros::Duration(1.0).sleep();

  ros::Rate rate(5.0);
  while (ros::ok())
  {
    ros::spinOnce();
    
    msgTriggerAlarm.node_name = "/landshark/cmu_monitors";
    msgTriggerAlarm.description = "no alarm";
    
    gpsProjection.SetLocalCoordinateSystemOrigin(origin_lon,origin_lat); 
    gpsProjection.GetLocalCoordinatesFromGps(\
      (double)(msgGps.longitude),(double)(msgGps.latitude),\
      robot_meters_x,robot_meters_y);
    
    msgGpsMeters.header.stamp = msgGps.header.stamp; 
    msgGpsMeters.point.x = robot_meters_x;  
    msgGpsMeters.point.y = robot_meters_y;
    pubGpsMeters.publish(msgGpsMeters);

//--------------------------------------------------------------------------------
//DynamicWindow    
//-------------------------------------------------------------------------------- 
    if (msgCmuDynamicWindowMonitorControl.data)
    {
      msgCmuDynamicWindowMonitorStatus.data = true; 
//no conversion, msgObstacleMeters should already be in meters      
//      obstacle_gps_x = msgObstacleGps.point.x;
//      obstacle_gps_y = msgObstacleGps.point.y;
      
      gpsProjection.GetLocalCoordinatesFromGps(\
      (double)(msgObstacleGps.longitude),(double)(msgObstacleGps.latitude),\
      obstacle_meters_x,obstacle_meters_y);
    
      okayToGo(\
        robot_meters_x, robot_meters_y, obstacle_meters_x, obstacle_meters_y,\
        msgBaseVelocity.twist.linear.x, msgBaseVelocity.twist.angular.z, dt,\
        accel_lin_max, inner_radius, outer_radius,\
        okayBoolOuter, dwmonitorOkayBool);  
        ROS_INFO("dwmonitorOkayBool = %d",dwmonitorOkayBool);       
      if (!dwmonitorOkayBool) 
      {
        msgTriggerAlarm.node_name = "/landshark/cmu_dynamic_window_monitor";
        msgTriggerAlarm.description = "robot is inside dynamic window";        
        ROS_INFO("%s",msgTriggerAlarm.description.c_str());
//        msgCmuDynamicWindowMonitorTrigger.alarm = true;
//        pubCmuDynamicWindowMonitorTrigger.publish(\
//        msgCmuDynamicWindowMonitorTrigger);
      }
    } 
    else
    {
      msgCmuDynamicWindowMonitorStatus.data = false; 
    }
//--------------------------------------------------------------------------------
//SensorZtest    
//--------------------------------------------------------------------------------    
    if (msgCmuSensorZtestMonitorControl.data)  
    {
      msgCmuSensorZtestMonitorStatus.data = true;
      pubCmuSensorZtestMonitorStatus.publish(\
      msgCmuSensorZtestMonitorStatus);   
      
      motionMonitor.run();
      if(motionMonitor.getAnomaly()>anomaly_thresh) {sensorZtestOkayBool = false;}
      else {sensorZtestOkayBool=true;}
//      printf("\nmotionMonitor.getAnomaly() = %f",motionMonitor.getAnomaly());
      
      ROS_INFO("motionMonitor.getAnomaly() = %f",motionMonitor.getAnomaly()); 
      
      ROS_INFO("sensorZtestOkayBool = %d",sensorZtestOkayBool); 
      if ( !sensorZtestOkayBool )
      {
        msgTriggerAlarm.node_name = "/landshark/cmu_sensor_ztest_monitor";
        msgTriggerAlarm.description = "anomaly in sensor ztest data";
//        msgCmuSensorZtestMonitorTrigger.alarm = true;
//        pubCmuSensorZtestMonitorTrigger.publish(\
//        msgCmuSensorZtestMonitorTrigger);        
      }
    }
    else
    {
      motionMonitor.reset();
      msgCmuSensorZtestMonitorStatus.data = false;
    }
    
    pubCmuDynamicWindowMonitorStatus.publish(msgCmuDynamicWindowMonitorStatus); 
    pubCmuSensorZtestMonitorStatus.publish(msgCmuSensorZtestMonitorStatus); 
    pubTriggerAlarm.publish(msgTriggerAlarm);
    rate.sleep();
  }

//http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers
ros::shutdown();
//ros::Publisher::pubCmuDynamicWindowMonitorStatus::shutdown();
//ros::Publisher::pubCmuDynamicWindowMonitorTrigger::shutdown();
//ros::Publisher::pubCmuSensorZtestMonitorStatus::shutdown();
//ros::Publisher::pubCmuSensorZtestMonitorTrigger::shutdown();
 
//--------------------------------------------------------------------------------
return 0;
}
