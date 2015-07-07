/*
 *      testDwMonitor.cpp
 *
 *
 *      Author: Fatma Faruq
 *      HACMS CMU Team, Advisor Manuela Veloso
 */

#include <ros/ros.h>
#include "teleopFilterCheck.h"
#include "std_msgs/String.h"
#include "landshark_cmu_monitor/AnomalyMsg.h"

using namespace std;

geometry_msgs::TwistStamped atempTwist;
bool execution_monitor_ok;

void getBaseVelocity(const geometry_msgs::TwistStamped::ConstPtr& pMsg)
{

    atempTwist = *pMsg;
}

void execution_monitor_callback(const
                                landshark_cmu_monitor::AnomalyMsgConstPtr& msg)
{
    bool debug = false;
    if(debug) printf("anomaly value: %.2f\n", msg->anomaly_value);
    if(msg->anomaly_detected) execution_monitor_ok = false;
}

int main(int argc, char **argv)
{
    //initialize ros
    ros::init(argc, argv, "telopFilter");
    ros::NodeHandle n;
    ros::Rate rate(1000);

    //initialize filter
    string gpsFilePath;
    if (!n.hasParam("gps_file_pathname"))
    {
        cout << "\nCan't find gps file path. will use default";
        gpsFilePath = "/home/fatma/ros_groovy_base/CMU/src/landshark_cmu_dwmonitor/src/gpsFile.txt";
    }
    else
        n.getParam("gps_file_pathname", gpsFilePath);
    teleopFilterCheck filter;
    bool gpsMode = true;
    filter.initClass(gpsMode, n);
    filter.initFromFile(gpsFilePath);

    //get parameters
    string topicToListenTo;
    string topicToPublishTo;
    bool useDWmonitor;
    std_msgs::String statusMessage;

    if (!n.hasParam("velocity_sub_topic"))
    {
        cout << "\nCan't find velocity_sub_topic. Setting to /landshark_control/base_velocity_temp";
        topicToListenTo = "/landshark_control/base_velocity_temp";
    }
    else
        n.getParam("velocity_sub_topic", topicToListenTo);
    if ((!n.hasParam("velocity_pub_topic")))
    {
        cout << "\nCan't find velocity_pub_topic. Setting to /landshark_control/base_velocity";
        topicToPublishTo = "/landshark_control/base_velocity";
    }
    else
        n.getParam("velocity_pub_topic", topicToPublishTo);
    if ((!n.hasParam("use_dw_monitor")))
    {
        cout << "\nCan't find use_dw_monitor. Setting to false";
        useDWmonitor = false;
    }
    else
        n.getParam("use_dw_monitor", useDWmonitor);

    cout << "\nSubscribing to " << topicToListenTo << endl;
    cout << "\nPublishing to " << topicToPublishTo << endl;
    cout << "\nReading from "<< gpsFilePath <<endl;
    //cin.get();

    if (useDWmonitor)
        statusMessage.data = "true";
    else
        statusMessage.data = "false";

    //listen to the right topic
    ros::Subscriber baseVelTempSub = n.subscribe < geometry_msgs::TwistStamped > (topicToListenTo, 1, getBaseVelocity);
//"/landshark_control/base_velocity_temp",1,getBaseVelocity);


    //publish to a topic
    ros::Publisher baseVelPub = n.advertise < geometry_msgs::TwistStamped > (topicToPublishTo, 1);
//"/landshark_control/base_velocity",1);

    string statusMessagePublish = "/landshark/dwMonitorStatus";
    n.getParam("status_msgs_topic", statusMessagePublish);
    ros::Publisher dwMonitorStatusPub = n.advertise < std_msgs::String > (statusMessagePublish, 1);
    //cin.get();

    //Hack added by JP to make robot stop when execution monitor says something is wrong
    ros::Subscriber execution_monitor_subscriber =
        n.subscribe("/landshark/ExecutionMonitor/AnomalyMonitor",1,
                    &execution_monitor_callback);
    execution_monitor_ok = true;

    ros::Publisher maxSafeDWSpeedPub = n.advertise<geometry_msgs::TwistStamped>("/landshark/dwmonitor/safeSpeed",1);

    while (ros::ok())
    {
        ros::spinOnce();
        if (useDWmonitor)
        {
            if (!filter.okayToGo(atempTwist.twist.linear.x, atempTwist.twist.angular.z) ||
                    !execution_monitor_ok)
            {
                atempTwist.twist.linear.x = 0;
                atempTwist.twist.angular.z = 0;
            }
        }
        baseVelPub.publish(atempTwist);
        if(atempTwist.twist.linear.x < 0)
            atempTwist.twist.linear.x *= -1;
        atempTwist.twist.angular.z = 0;
        maxSafeDWSpeedPub.publish(atempTwist);
        dwMonitorStatusPub.publish(statusMessage);
        rate.sleep();
    }
    return 0;
}
