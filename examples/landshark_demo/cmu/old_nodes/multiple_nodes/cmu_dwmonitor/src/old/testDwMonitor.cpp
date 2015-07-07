/*
 *      testDwMonitor.cpp
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */

#include <ros/ros.h>
#include "teleopFilterCheck.h"
#include "landshark_cmu_monitor/AnomalyMsg.h"
#include "landshark_cmu_dwmonitor/dwMonitorSwitch.h"
#include "landshark_msgs/BoolStamped.h"

using namespace std;

static const double CMU_long = -79.945;
static const double CMU_lat = 40.443;

geometry_msgs::TwistStamped atempTwist;
nav_msgs::Odometry atempOdom;
bool execution_monitor_ok;
bool hrl_bool = false;
int dwMonitorSwitch = 0;

void getBaseVelocity(const geometry_msgs::TwistStamped::ConstPtr& pMsg)
{
    atempTwist = *pMsg;
}

void printBaseVelocity()
{
    printf("printBaseVelocity: atempTwist.twist.linear.x = %.2f\n", atempTwist.twist.linear.x);
}

void getOdomVelocity(const nav_msgs::Odometry::ConstPtr& pMsg)
{
    atempOdom = *pMsg;
}

//void getHrlGpsVelocity(const std_msgs::Float32::ConstPtr& pMsg)
//{
//    atempHrlGpsVelocity = *pMsg;
//}

void execution_monitor_callback(const
                                landshark_cmu_monitor::AnomalyMsgConstPtr& msg)
{
    bool debug = false;
    if(debug) printf("anomaly value: %.2f\n", msg->anomaly_value);
    if(msg->anomaly_detected) execution_monitor_ok = false;
}

void dwMonitorSwitchCallback(const landshark_cmu_dwmonitor::dwMonitorSwitch::ConstPtr& pMsg)
{
    if(pMsg != NULL)
        dwMonitorSwitch = pMsg->dwMonitorSwitch;

}

int main(int argc, char **argv)
{
//initialize ros
  ros::init(argc, argv, "telopFilter");
  ros::NodeHandle n;
  ros::Rate rate(1000);
//get parameters
  string topicToListenTo;
  string topicOdomListen;
  string topicToPublishTo;
  string topicPubMinSpeed;
  bool useDWmonitor;
  double clearanceDist1 = 0; 
  double clearanceDist2 = 0;
  double maxRvdot = 4; 
  double maxRwdot = 4; 
  double maxOv = 1; 
  double maxOdist = 10; 
  double maxRv = 1; 
  double maxRw = 1; 
  double epsilon = 0.25;

landshark_msgs::BoolStamped statusMessage;
string paramName = "safetyClearance1";
  if( !n.hasParam(paramName))
	   {
		  printf("\nCan't get %s from parameter server. Setting to default value %f",paramName.c_str(),1.5);
		  clearanceDist1 = 1.5;
 	  }
	 else
	 n.getParam(paramName,clearanceDist1);
	 
	 paramName = "hrl";
  if( !n.hasParam(paramName))
	   {
		  printf("\nCan't get %s from parameter server. Setting to default value: off",paramName.c_str());
	   }
	 else
	 n.getParam(paramName,hrl_bool);
	 
  paramName = "robot_max_acc_lin";
  if( !n.hasParam(paramName))
	   {
		  printf("\nCan't get %s from parameter server. Setting to default value %f",paramName.c_str(),maxRvdot);
	   }
	 else
	 n.getParam(paramName,maxRvdot);

  paramName = "robot_max_acc_ang";
  if( !n.hasParam(paramName))
	 {
		  printf("\nCan't get %s from parameter server. Setting to default value %f",paramName.c_str(),maxRwdot);
	 }
	 else
	 n.getParam(paramName,maxRwdot);

  paramName = "max_robot_vel_ang";
  if( !n.hasParam(paramName))
	 {
		  printf("\nCan't get %s from parameter server. Setting to default value %f",paramName.c_str(),maxRw);
	 }
	 else
	 n.getParam(paramName,maxRw);

  paramName = "max_robot_vel_lin";
  if( !n.hasParam(paramName))
	 {
		  printf("\nCan't get %s from parameter server. Setting to default value %f",paramName.c_str(),maxRv);
	 }
	 else
	 n.getParam(paramName,maxRv);

  paramName = "max_obs_vel";
  if( !n.hasParam(paramName))
	 {
		  printf("\nCan't get %s from parameter server. Setting to default value %f",paramName.c_str(),maxOv);
	 }
	 else
	 n.getParam(paramName,maxOv);

  paramName = "max_obs_dist";
  if( !n.hasParam(paramName))
	 {
		  printf("\nCan't get %s from parameter server. Setting to default value %f",paramName.c_str(),maxOdist);
	 }
	 else
	 n.getParam(paramName,maxOdist);

  paramName = "epsilon";
  if( !n.hasParam(paramName))
	 {
		  printf("\nCan't get %s from parameter server. Setting to default value %f",paramName.c_str(),epsilon);
	 }
	 else
	 n.getParam(paramName,epsilon);

  if( !n.hasParam("safetyClearance2"))
	 {
		  printf("\nCan't get %s from parameter server. Setting to default value %f","safetyClearance2",2.5);
		  clearanceDist1 = 2.5;
	 }
	 else
	 n.getParam("safetyClearance2",clearanceDist2);
	 
//	 if (hrl_bool)
//	 {
//	   printf("\n made it into hrl_bool");   
//	   topicToPublishTo = "/landshark/dwmonitor/safeSpeed";
//	   ros::Publisher baseVelPub = n.advertise < geometry_msgs::TwistStamped > (topicToPublishTo, 1);
//	   topicPubMinSpeed = "/landshark/dwmonitor/minSpeed";
//	   ros::Publisher pubMinSpeed = n.advertise < geometry_msgs::TwistStamped > (topicPubMinSpeed, 1);
////    printBaseVelocity();
////    exit(0);
//	 }
//	 else
//	 {
    if (!n.hasParam("velocity_sub_topic"))
    {
      cout << "\nCan't find velocity_sub_topic. Setting to /landshark_control/base_velocity";
      topicToListenTo = "/landshark_control/base_velocity";
      topicOdomListen = "/landshark/odom";
    }
    else
    {
      n.getParam("velocity_sub_topic", topicToListenTo);
      n.getParam("velocity_odom_topic", topicOdomListen);
      cout << "\nSubscribing to " << topicToListenTo << endl;
      //listen to the right topic
      ros::Subscriber baseVelTempSub = n.subscribe < geometry_msgs::TwistStamped > (topicToListenTo, 1, getBaseVelocity);
      ros::Subscriber odomVelTempSub = n.subscribe < nav_msgs::Odometry > (topicOdomListen, 1, getOdomVelocity); 
    }
//  }

  if ((!n.hasParam("velocity_pub_topic")))
  {
    cout << "\nCan't find velocity_pub_topic. Setting to /landshark/dwmonitor/safeSpeed";
    topicToPublishTo = "/landshark/dwmonitor/safeSpeed";
    topicPubMinSpeed = "/landshark/dwmonitor/minSpeed";
    cout << "\nPublishing to " << topicToPublishTo << endl;
    cout << "\nPublishing to " << topicPubMinSpeed << endl;
  }
  else
  {
    n.getParam("velocity_pub_topic", topicToPublishTo);
    n.getParam("velocity_min_pub_topic", topicPubMinSpeed);
  }
  
string statusMessagePublish = "/landshark/dwmonitor/status";
  if(!(n.hasParam("status_msgs_topic")))
  {
    cout<<"\nCan't find status_msgs_topic. Setting to "<<statusMessagePublish<<endl;
  }
  else
    n.getParam("status_msgs_topic", statusMessagePublish);
    ros::Publisher dwMonitorStatusPub = n.advertise < landshark_msgs::BoolStamped > (statusMessagePublish, 1);
    ros::Subscriber dwMonitorStatusSub = n.subscribe <landshark_cmu_dwmonitor::dwMonitorSwitch> ("/landshark_control/dwmonitor/monitorSwitch",1,&dwMonitorSwitchCallback);
    
//publish to a topic
  ros::Publisher baseVelPub = n.advertise < geometry_msgs::TwistStamped > (topicToPublishTo, 1);
  ros::Publisher pubMinSpeed = n.advertise < geometry_msgs::TwistStamped > (topicPubMinSpeed, 1);    

//initialize filter
  bool gpsMode = true;
  bool getObsFromTopic = true;
  teleopFilterCheck filter(gpsMode, n,getObsFromTopic,clearanceDist1,clearanceDist2);
//input params for dwmonitor
  filter.es.robotFunctions.initGPSProjection(CMU_long, CMU_lat);
  filter.initGPSConverter(CMU_lat,CMU_long);
  filter.initConstants(maxRv,maxRw,epsilon,maxRvdot,maxRwdot,maxOdist,maxOv);
//Hack added by JP to make robot stop when execution monitor says something is wrong
  ros::Subscriber execution_monitor_subscriber =
        n.subscribe("/landshark/ExecutionMonitor/AnomalyMonitor",1,
                    &execution_monitor_callback);
  execution_monitor_ok = true;


    while (ros::ok())
    {
        ros::spinOnce();
        statusMessage.header.stamp = ros::Time::now();
        statusMessage.data = false;
        geometry_msgs::TwistStamped twistToSend = atempTwist;
        geometry_msgs::TwistStamped twistMinSpeed = atempTwist;
        if(dwMonitorSwitch == landshark_cmu_dwmonitor::dwMonitorSwitch::ON)
        {
            statusMessage.data = true;
            bool debug = false;
            twistToSend = atempTwist;
            if(debug)
            {
                printf("\nDW monitor switch is on");
                printf("\nSending to filter v: %f w: %f ",atempTwist.twist.linear.x, atempTwist.twist.angular.z);
                printf("\nSending to filter v: %f w: %f ",twistToSend.twist.linear.x, twistToSend.twist.angular.z);
            }

            if (!filter.okayToGo(atempTwist.twist.linear.x, atempTwist.twist.angular.z) ||
                    !execution_monitor_ok)
            {
                twistToSend.twist.linear.x = 0;
                twistToSend.twist.angular.z = 0;
            }
//FIX ME            
//            if(hrl_bool)
//            {
//              atempHrlGpsVelocity
//              if (!filter.okayToGo(atempTwist.twist.linear.x, atempTwist.twist.angular.z) ||
//                    !execution_monitor_ok)
//            {
//                twistToSend.twist.linear.x = 0;
//                twistToSend.twist.angular.z = 0;
//            }
//            }
//FIX ME

            if(twistToSend.twist.linear.x < 0)
                twistToSend.twist.linear.x *= -1;
            twistToSend.twist.angular.z = 0;
        }
        
//FIX ME        
//        if (!hrl_bool){
          baseVelPub.publish(twistToSend);
          twistMinSpeed.twist.linear.x = std::min(twistToSend.twist.linear.x,atempOdom.twist.twist.linear.x); 
          twistMinSpeed.twist.angular.z = std::min(twistToSend.twist.angular.z,atempOdom.twist.twist.angular.z);
          pubMinSpeed.publish(twistMinSpeed);
//        }
//FIX ME

        dwMonitorStatusPub.publish(statusMessage);
        rate.sleep();
    }
    return 0;
}
