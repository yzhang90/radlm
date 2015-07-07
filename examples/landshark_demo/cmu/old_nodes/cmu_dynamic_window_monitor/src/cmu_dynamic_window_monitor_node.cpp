/*      
 * Author: Fatma Faruq, Jason Larkin
 * HACMS CMU Team, Advisor Manuela Veloso, SpiralGen, Inc.
 */
//--------------------------------------------------------------------------------
#include "ros/ros.h"
#include <msg_auth/msg_auth.h>
#include "dwmonitor.h"

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

#include "cmu_dynamic_window_monitor/obstacle.h"
//#include "cmu_sensor_ztest_monitor/AnomalyMsg.h"
#include <cmu_z_test_monitor/AnomalyMsg.h>

//--------------------------------------------------------------------------------
sensor_msgs::NavSatFix msgGps;
geometry_msgs::PointStamped msgGpsMeters;
sensor_msgs::NavSatFix msgObstacleGps;
landshark::GpsProjection gpsProjection;

nav_msgs::Odometry msgOdom;
nav_msgs::Odometry msgOdomObstacle;

geometry_msgs::TwistStamped msgBaseVelocity;

landshark_msgs::BoolStamped msgCmuDynamicWindowMonitorStatus;

landshark_msgs::TriggerAlarm msgTriggerAlarm;
landshark_msgs::TriggerAlarm msgCmuDynamicWindowMonitorTrigger;

landshark_msgs::ResetAlarm msgResetAlarm;
landshark_msgs::ResetAlarm msgCmuDynamicWindowMonitorReset;

landshark_msgs::BoolStamped msgCmuDynamicWindowMonitorControl;

cmu_z_test_monitor::AnomalyMsg msgAnomaly;
//--------------------------------------------------------------------------------
void getGps(\
const sensor_msgs::NavSatFix::ConstPtr& pMsg)
{msgGps = *pMsg;}

void getGpsMeters(\
const geometry_msgs::PointStamped::ConstPtr& pMsg)
{msgGpsMeters = *pMsg;}

void getObstacleGps(\
const sensor_msgs::NavSatFix::ConstPtr& pMsg)
{msgObstacleGps = *pMsg;}

void getObstacleGps(\
const geometry_msgs::PointStamped::ConstPtr& pMsg)
{
  msgObstacleGps.header = pMsg->header;
  msgObstacleGps.longitude = pMsg->point.x;
  msgObstacleGps.latitude = pMsg->point.y;
}

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

void getCmuSensorZtestMonitorAnomalyMonitor(\
const cmu_z_test_monitor::AnomalyMsg::ConstPtr& pMsg)
{msgAnomaly = *pMsg;}

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
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmu_dynamic_window_monitor");
  ros::NodeHandle node;
  msg_auth::MsgAuth msgAuth(node);

  std::string topicGps;
  std::string topicGpsObstacle;
  std::string topicGpsMeters;
  
  std::string topicBaseVelocity;
  std::string topicOdom;

  std::string topicCmuDynamicWindowMonitorStatus;
  std::string topicCmuDynamicWindowMonitorTrigger;
  std::string topicCmuDynamicWindowMonitorControl;
  std::string topicCmuDynamicWindowMonitorReset;
  
  std::string topicResetAlarm;
  std::string topicTriggerAlarm;
  
  std::string topicCmuSensorZtestMonitorAnomalyMonitor;
  
  bool latch;
  std::string paramName;
  
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

  bool useMsgAuth = true;
  
//--------------------------------------------------------------------------------
//input params
//--------------------------------------------------------------------------------
  node.getParam("origin_lat",origin_lat);
	 node.getParam("origin_lon",origin_lon);	
	 node.getParam("obstacle_meters_x_reset",obstacle_meters_x_reset);	
	 node.getParam("obstacle_meters_y_reset",obstacle_meters_y_reset);			 
	 node.getParam("dt",dt);
	 node.getParam("outer_radius",outer_radius);
	 node.getParam("inner_radius",inner_radius);
	 node.getParam("accel_lin_max",accel_lin_max);
	 node.getParam("accel_ang_max",accel_ang_max);

	 node.getParam("vel_ang_max",vel_ang_max);
	 node.getParam("vel_lin_max",vel_lin_max);
	 node.getParam("max_obs_vel",max_obs_vel);
	 node.getParam("max_obs_dist",max_obs_dist);
	 node.getParam("epsilon",epsilon);
  node.getParam("use_msg_auth", useMsgAuth);

  node.getParam("/landshark/gps", topicGps);
  node.getParam("/landshark/gps_meters", topicGpsMeters);
  node.getParam("/landshark_control/base_velocity", topicBaseVelocity);
  node.getParam("/landshark/odom", topicOdom);
  node.getParam("/landshark/cmu_dynamic_window_monitor/status",\
  topicCmuDynamicWindowMonitorStatus);
  node.getParam("/landshark/cmu_dynamic_window_monitor/obstacle",\
  topicGpsObstacle);
  node.getParam("/landshark_control/trigger_alarm", topicTriggerAlarm);
  node.getParam("/landshark_control/reset_alarm", topicResetAlarm);
  node.getParam("/landshark/cmu_dynamic_window_monitor/control",\
  topicCmuDynamicWindowMonitorControl);
  node.getParam("/landshark/cmu_z_test_monitor/AnomalyMonitor",\
  topicCmuSensorZtestMonitorAnomalyMonitor);
//--------------------------------------------------------------------------------
//topics  
//--------------------------------------------------------------------------------

  msg_auth::Publisher authPubCmuDynamicWindowMonitorStatus;
  msg_auth::Subscriber authPubGpsObstacle;
  msg_auth::Publisher authPubTriggerAlarm;
  msg_auth::Subscriber authSubResetAlarm;
  msg_auth::Subscriber authSubCmuDynamicWindowMonitorControl;

  ros::Publisher pubCmuDynamicWindowMonitorStatus;
  ros::Subscriber pubGpsObstacle;
  ros::Publisher pubTriggerAlarm;
  ros::Subscriber subResetAlarm;
  ros::Subscriber subCmuDynamicWindowMonitorControl;
  
  if (useMsgAuth)
  {
    ROS_INFO("Publishing to authenticated %s",\
    topicCmuDynamicWindowMonitorStatus.c_str());
    authPubCmuDynamicWindowMonitorStatus = \
    msgAuth.advertise<landshark_msgs::BoolStamped>(\
    topicCmuDynamicWindowMonitorStatus, 1, true);

    ROS_INFO("Subscribing to authenticated %s",topicGpsObstacle.c_str());
    authPubGpsObstacle = \
    msgAuth.subscribe<geometry_msgs::PointStamped>(\
    topicGpsObstacle, 1, getObstacleGps);

    ROS_INFO("Subscribing to authenticated %s",topicTriggerAlarm.c_str());
    authPubTriggerAlarm =\
    msgAuth.advertise<landshark_msgs::TriggerAlarm>(topicTriggerAlarm, 1, false);

    ROS_INFO("Subscribing to authenticated %s",topicResetAlarm.c_str());
    authSubResetAlarm = \
    msgAuth.subscribe<landshark_msgs::ResetAlarm>(\
    topicResetAlarm, 1, getResetAlarm);

    ROS_INFO("Subscribing to authenticated %s",\
    topicCmuDynamicWindowMonitorControl.c_str());
    authSubCmuDynamicWindowMonitorControl = \
    msgAuth.subscribe<landshark_msgs::BoolStamped>(\
    topicCmuDynamicWindowMonitorControl, 1, getCmuDynamicWindowMonitorControl);  
  }
  else
  {
    ROS_INFO("Publishing to plain %s",topicCmuDynamicWindowMonitorStatus.c_str());
    pubCmuDynamicWindowMonitorStatus =\
    node.advertise<landshark_msgs::BoolStamped>(\
    topicCmuDynamicWindowMonitorStatus, 1, true);

    ROS_INFO("Subscribing to plain %s",topicGpsObstacle.c_str());
    pubGpsObstacle = node.subscribe<geometry_msgs::PointStamped>(\
    topicGpsObstacle, 1, getObstacleGps);

    ROS_INFO("Subscribing to plain %s",topicTriggerAlarm.c_str());
    pubTriggerAlarm = node.advertise<landshark_msgs::TriggerAlarm>(\
    topicTriggerAlarm, 1, false);

    ROS_INFO("Subscribing to plain %s",topicResetAlarm.c_str());
    subResetAlarm = node.subscribe<landshark_msgs::ResetAlarm>(\
    topicResetAlarm, 1, getResetAlarm);
    
    ROS_INFO("Subscribing to plain %s",\
    topicCmuDynamicWindowMonitorControl.c_str());
    subCmuDynamicWindowMonitorControl = \
    node.subscribe<landshark_msgs::BoolStamped>(\
    topicCmuDynamicWindowMonitorControl, 1, getCmuDynamicWindowMonitorControl);  
  }
  
  ROS_INFO("Subscribing to %s", topicGps.c_str());
  ros::Subscriber pubGps = node.subscribe < sensor_msgs::NavSatFix > \
  (topicGps, 1, getGps);
//--------------------------------------------------------------------------------
  ROS_INFO("Subscribing to %s",topicGpsMeters.c_str());
  ros::Subscriber subGpsMeters = node.subscribe < geometry_msgs::PointStamped > \
  (topicGpsMeters, 1, getGpsMeters);;    
//--------------------------------------------------------------------------------
  ROS_INFO("Subscribing to %s",topicBaseVelocity.c_str());
  ros::Subscriber pubBaseVelocity =\
  node.subscribe < geometry_msgs::TwistStamped > \
  (topicBaseVelocity, 1, getBaseVelocity); 
//--------------------------------------------------------------------------------  
  ROS_INFO("Subscribing to %s",topicOdom.c_str());
  ros::Subscriber pubOdom = node.subscribe < nav_msgs::Odometry > \
  (topicOdom, 1, getOdom); 
//--------------------------------------------------------------------------------
  latch = false;
//--------------------------------------------------------------------------------
 
  ROS_INFO("Subscribing to %s", topicCmuSensorZtestMonitorAnomalyMonitor.c_str());
  ros::Subscriber subCmuSensorZtestMonitorAnomalyMonitor =\
  node.subscribe < cmu_z_test_monitor::AnomalyMsg > \
  (topicCmuSensorZtestMonitorAnomalyMonitor, 1,\
   getCmuSensorZtestMonitorAnomalyMonitor);  
//--------------------------------------------------------------------------------
  bool okayBoolOuter; //= false;
  bool dwmonitorOkayBool; //= false; 
//initialize obstacle like a reset (far away)  
  msgObstacleGps.latitude = obstacle_gps_x_reset;
  msgObstacleGps.longitude = obstacle_gps_y_reset;

//sleep to let topics get published to
  ros::Duration(0.5).sleep();

  ros::Rate rate(30.0);
  while (ros::ok())
  {
    ros::spinOnce();
    
//    msgTriggerAlarm.node_name = "/landshark/cmu_monitors";
//    msgTriggerAlarm.description = "no alarm";
    
    gpsProjection.SetLocalCoordinateSystemOrigin(origin_lon,origin_lat); 
    gpsProjection.GetLocalCoordinatesFromGps(\
      (double)(msgGps.longitude),(double)(msgGps.latitude),\
      robot_meters_x,robot_meters_y);
    
    msgGpsMeters.header.stamp = msgGps.header.stamp; 
    msgGpsMeters.point.x = robot_meters_x;  
    msgGpsMeters.point.y = robot_meters_y;
//    pubGpsMeters.publish(msgGpsMeters);

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
      
      ROS_INFO("\n msgAnomaly.anomaly_detected = %d",msgAnomaly.anomaly_detected);
      
      if (msgAnomaly.anomaly_detected)
      {
        msgTriggerAlarm.node_name = "/landshark/cmu_z_test_monitor";
        msgTriggerAlarm.description = "robot sensors have failed ztest";

        if (useMsgAuth)
        {
          authPubTriggerAlarm.publish(msgTriggerAlarm);
        }
        else
        {
          pubTriggerAlarm.publish(msgTriggerAlarm);
        }
      }
    
      okayToGo(\
        robot_meters_x, robot_meters_y, obstacle_meters_x, obstacle_meters_y,\
        msgBaseVelocity.twist.linear.x, msgBaseVelocity.twist.angular.z, dt,\
        accel_lin_max, inner_radius, outer_radius,\
        okayBoolOuter, dwmonitorOkayBool);  
        
//      okayToGo(\
//        msgGpsMeters.point.x, msgGpsMeters.point.y, obstacle_meters_x, obstacle_meters_y,\
//        msgBaseVelocity.twist.linear.x, msgBaseVelocity.twist.angular.z, dt,\
//        accel_lin_max, inner_radius, outer_radius,\
//        okayBoolOuter, dwmonitorOkayBool);    
        
        ROS_INFO("dwmonitorOkayBool = %d",dwmonitorOkayBool);       
      if (!dwmonitorOkayBool) 
      {
        msgTriggerAlarm.node_name = "/landshark/cmu_dynamic_window_monitor";
        msgTriggerAlarm.description = "robot is inside dynamic window";        
        ROS_INFO("%s",msgTriggerAlarm.description.c_str());
//        msgCmuDynamicWindowMonitorTrigger.alarm = true;
//        pubCmuDynamicWindowMonitorTrigger.publish(\
//        msgCmuDynamicWindowMonitorTrigger);

        if (useMsgAuth)
        {
          authPubTriggerAlarm.publish(msgTriggerAlarm);
        }
        else
        {
          pubTriggerAlarm.publish(msgTriggerAlarm);
        }
      }
    } 
    else
    {
      msgCmuDynamicWindowMonitorStatus.data = false; 
    }

    if (useMsgAuth)
    {
      authPubCmuDynamicWindowMonitorStatus.publish(\
      msgCmuDynamicWindowMonitorStatus);
    }
    else
    {
      pubCmuDynamicWindowMonitorStatus.publish(\
      msgCmuDynamicWindowMonitorStatus);
    }
    
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
