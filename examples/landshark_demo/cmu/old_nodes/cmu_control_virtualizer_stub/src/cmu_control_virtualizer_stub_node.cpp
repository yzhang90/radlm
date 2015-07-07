/*      
 * Author: Fatma Faruq, Jason Larkin
 * HACMS CMU Team, Advisor Manuela Veloso, SpiralGen, Inc.
 */
//--------------------------------------------------------------------------------
#include "ros/ros.h"
#include <string>
#include <geometry_msgs/TwistStamped.h>
#include <landshark_msgs/BoolStamped.h>
#include <cmu_control_virtualizer_stub/TriggerAlarm.h>
#include <cmu_control_virtualizer_stub/ResetAlarm.h>
//--------------------------------------------------------------------------------
geometry_msgs::TwistStamped msgBaseVelocity;
geometry_msgs::TwistStamped msgCmuBaseVelocity;
landshark_msgs::BoolStamped msgDeadman;

landshark_msgs::BoolStamped msgCmuDynamicWindowMonitorStatus;
landshark_msgs::BoolStamped msgCmuSensorZtestMonitorStatus;

cmu_control_virtualizer_stub::TriggerAlarm msgTriggerAlarm;
cmu_control_virtualizer_stub::TriggerAlarm msgCmuDynamicWindowMonitorTrigger;
cmu_control_virtualizer_stub::TriggerAlarm msgCmuSensorZtestMonitorTrigger;
//--------------------------------------------------------------------------------
void getCmuBaseVelocity(const geometry_msgs::TwistStamped::ConstPtr& pMsg)
{
//printf("\n*pMsg.twist.linear.x = %f \n", *pMsg.twist.linear.x);

msgCmuBaseVelocity = *pMsg;

//if(pMsg != 0){
//msgCmuBaseVelocity = *pMsg;
//printf("\nif msgCmuBaseVelocity.twist.linear.x = %f \n",\
//msgCmuBaseVelocity.twist.linear.x);
//}
//else {
//msgCmuBaseVelocity.twist.linear.x = 0.0;
//msgCmuBaseVelocity.twist.angular.z = 0.0;
//printf("\nelse msgCmuBaseVelocity.twist.linear.x = %f \n",\
//msgCmuBaseVelocity.twist.linear.x);
//}

}

void getCmuDynamicWindowMonitorStatus(\
const landshark_msgs::BoolStamped::ConstPtr& pMsg)
{msgCmuDynamicWindowMonitorStatus = *pMsg;}

void getCmuSensorZtestMonitorStatus(\
const landshark_msgs::BoolStamped::ConstPtr& pMsg)
{msgCmuSensorZtestMonitorStatus = *pMsg;}

void getTriggerAlarm(\
const cmu_control_virtualizer_stub::TriggerAlarm::ConstPtr& pMsg)
{msgTriggerAlarm = *pMsg;}

void getCmuDynamicWindowMonitorTrigger(\
const cmu_control_virtualizer_stub::TriggerAlarm::ConstPtr& pMsg)
{msgCmuDynamicWindowMonitorTrigger = *pMsg;}

void getCmuSensorZtestMonitorTrigger(\
const cmu_control_virtualizer_stub::TriggerAlarm::ConstPtr& pMsg)
{msgCmuSensorZtestMonitorTrigger = *pMsg;}

//std::string getTopicFromParam(std::string &paramName)
//{

//  if (!node.hasParam(paramName))
//  {
//    printf("\nCan't get %s from parameter server. Setting to default value %s",\
//    paramName.c_str(),paramName.c_str());
//    topicBaseVelocity = paramName.c_str();
//    printf("\nPublishing to %s\n",paramName.c_str());
//  }
//  else
//  {
//    node.getParam(paramName.c_str(), topicBaseVelocity);
//    printf("\nSubscribing to %s\n",paramName.c_str());
//  }
//  
//  return paramName.c_str()
//}

//--------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmu_control_virtualizer_stub");
  ros::NodeHandle node;

  std::string topicBaseVelocity;
  std::string topicCmuBaseVelocity;
  std::string topicDeadman;
  std::string topicCmuDynamicWindowMonitorStatus;
  std::string topicCmuSensorZtestMonitorStatus;
  std::string topicTriggerAlarm;
  std::string topicCmuDynamicWindowMonitorTrigger;
  std::string topicCmuSensorZtestMonitorTrigger;

//--------------------------------------------------------------------------------
  std::string paramName = "/landshark_control/base_velocity";
//  topicBaseVelocity = getTopicFromParam(paramName);  
  if (!node.hasParam(paramName))
  {
    printf("\nCan't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicBaseVelocity = paramName.c_str();
    printf("\nPublishing to %s\n",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicBaseVelocity);
    printf("\nSubscribing to %s\n",paramName.c_str());
  }
  bool latch = true;
  ros::Publisher pubBaseVelocity =\
  node.advertise < geometry_msgs::TwistStamped > \
  (topicBaseVelocity, 1, latch);   
  msgBaseVelocity.twist.linear.x = 0.0;
  msgBaseVelocity.twist.angular.z = 0.0;
//--------------------------------------------------------------------------------
  paramName = "/landshark_control/deadman";
//  topicBaseVelocity = getTopicFromParam(paramName);  
  if (!node.hasParam(paramName))
  {
    printf("\nCan't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicDeadman = paramName.c_str();
    printf("\nPublishing to %s\n",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicDeadman);
    printf("\nSubscribing to %s\n",paramName.c_str());
  }
  latch = true;
  ros::Publisher pubDeadman =\
  node.advertise < landshark_msgs::BoolStamped > \
  (topicDeadman, 1, latch);   
  msgDeadman.data = false; 
//--------------------------------------------------------------------------------
  paramName = "/landshark_control_cmux/base_velocity";
  if (!node.hasParam(paramName))
  {
    printf("\nCan't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicCmuBaseVelocity = paramName.c_str();
    printf("\nPublishing to %s\n",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicCmuBaseVelocity);
    printf("\nSubscribing to %s\n",paramName.c_str());
  }
  ros::Subscriber subCmuBaseVelocity =\
  node.subscribe < geometry_msgs::TwistStamped > \
  (topicCmuBaseVelocity, 1, getCmuBaseVelocity);
//--------------------------------------------------------------------------------
  paramName = "/landshark/cmu_dynamic_window_monitor/status";
  if (!node.hasParam(paramName))
  {
    printf("\nCan't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicCmuDynamicWindowMonitorStatus = paramName.c_str();
    printf("\nPublishing to %s\n",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicCmuDynamicWindowMonitorStatus);
    printf("\nSubscribing to %s\n",paramName.c_str());
  }
  ros::Subscriber subCmuDynamicWindowMonitorStatus =\
  node.subscribe < landshark_msgs::BoolStamped > \
  (topicCmuDynamicWindowMonitorStatus, 1, getCmuDynamicWindowMonitorStatus);
//--------------------------------------------------------------------------------
  paramName = "/landshark/cmu_sensor_ztest_monitor/status";
  if (!node.hasParam(paramName))
  {
    printf("\nCan't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicCmuSensorZtestMonitorStatus = paramName.c_str();
    printf("\nPublishing to %s\n",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicCmuSensorZtestMonitorStatus);
    printf("\nSubscribing to %s\n",paramName.c_str());
  }
  ros::Subscriber subCmuSensorZtestMonitorStatus =\
  node.subscribe < landshark_msgs::BoolStamped > \
  (topicCmuSensorZtestMonitorStatus, 1, getCmuSensorZtestMonitorStatus);
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
//  ros::Subscriber subCmuDynamicWindowMonitorTrigger =\
//  node.subscribe < cmu_control_virtualizer_stub::TriggerAlarm > \
//  (topicCmuDynamicWindowMonitorTrigger, 1, getCmuDynamicWindowMonitorTrigger);
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
//  ros::Subscriber subCmuSensorZtestMonitorTrigger =\
//  node.subscribe < cmu_control_virtualizer_stub::TriggerAlarm > \
//  (topicCmuSensorZtestMonitorTrigger, 1, getCmuSensorZtestMonitorTrigger);
//--------------------------------------------------------------------------------
  paramName = "/landshark_control/trigger_alarm";
  if (!node.hasParam(paramName))
  {
    printf("\nCan't get %s from parameter server. Setting to default value %s",\
    paramName.c_str(),paramName.c_str());
    topicTriggerAlarm = paramName.c_str();
    printf("\nPublishing to %s\n",paramName.c_str());
  }
  else
  {
    node.getParam(paramName.c_str(), topicTriggerAlarm);
    printf("\nSubscribing to %s\n",paramName.c_str());
  }
  ros::Subscriber subTriggerAlarm =\
  node.subscribe < cmu_control_virtualizer_stub::TriggerAlarm > \
  (topicTriggerAlarm, 1, getTriggerAlarm);

//--------------------------------------------------------------------------------

//sleep 1 sec to let topics get publish to
  ros::Duration(1.0).sleep();

  ros::Rate rate(30);
  while (ros::ok())
  {
    ros::spinOnce();

    if (msgCmuDynamicWindowMonitorStatus.data ||\
    msgCmuSensorZtestMonitorStatus.data)
    {
      ROS_INFO("operating with monitors...");
//      msgBaseVelocity.twist.linear.x = msgCmuBaseVelocity.twist.linear.x; 
//      msgBaseVelocity.twist.angular.z = msgCmuBaseVelocity.twist.angular.z;
//      ROS_INFO("%s",msgTriggerAlarm.description.c_str());
      if (\
      msgTriggerAlarm.node_name == "/landshark/cmu_dynamic_window_monitor" ||\
      msgTriggerAlarm.node_name == "/landshark/cmu_sensor_ztest_monitor" ) 
      {
        ROS_INFO("alarm: %s",msgTriggerAlarm.description.c_str());
        msgBaseVelocity.twist.linear.x = 0.0; 
        msgBaseVelocity.twist.angular.z = 0.0;
        msgDeadman.data = false;
      }
      else
      {
        msgBaseVelocity.twist.linear.x = msgCmuBaseVelocity.twist.linear.x; 
        msgBaseVelocity.twist.angular.z = msgCmuBaseVelocity.twist.angular.z;
        msgDeadman.data = true;
      }
    }
    else
    {
      ROS_INFO("\noperating without monitors...");
      msgBaseVelocity.twist.linear.x = msgCmuBaseVelocity.twist.linear.x; 
      msgBaseVelocity.twist.angular.z = msgCmuBaseVelocity.twist.angular.z;
      msgDeadman.data = true;
//      printf("\nmsgCmuBaseVelocity.twist.linear.x = %f",\
//      msgCmuBaseVelocity.twist.linear.x);
//      printf("\nmsgCmuBaseVelocity.twist.angular.z = %f",\
//      msgCmuBaseVelocity.twist.angular.z);
//      printf("\nmsgBaseVelocity.twist.linear.x = %f",\
//      msgBaseVelocity.twist.linear.x);
//      printf("\nmsgBaseVelocity.twist.angular.z = %f",\
//      msgBaseVelocity.twist.angular.z);
    }
    pubDeadman.publish(msgDeadman);
    pubBaseVelocity.publish(msgBaseVelocity);
    rate.sleep();
  }

return 0;
}
