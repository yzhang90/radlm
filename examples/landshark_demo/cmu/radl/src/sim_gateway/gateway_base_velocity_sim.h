#ifndef GATEWAY_BASE_VELOCITY_SIM_H
#define GATEWAY_BASE_VELOCITY_SIM_H

#include RADL_HEADER

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/NavSatFix.h"
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <landshark_2dnav/LandsharkNavWaypoint.h>
#include <nav_msgs/Odometry.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>

class Gateway {
public:
  Gateway() {

    pubGps = h.advertise<sensor_msgs::NavSatFix>("radl_gps", 2);
    pubOdom = h.advertise<nav_msgs::Odometry>("/landshark/odom", 2);
    subGpsMeters = h.subscribe("/landshark/gps_meters", 2, &Gateway::getGpsMeters, this);
    subLandsharkNavStatus = h.subscribe("/landshark_2dnav/status", 2, &Gateway::getLandsharkNavStatus, this);

    subBaseVelocity = h.subscribe("/landshark_control/base_velocity", 2, &Gateway::getBaseVelocity, this);

    subMoveBaseFeedback = h.subscribe("/move_base/feedback", 2, &Gateway::getMoveBaseFeedback, this);
    pubMoveBaseCancel = h.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 2);

  }

  void getGpsMeters(const geometry_msgs::PointStamped::ConstPtr& pMsg) {
    this->msgGpsMeters = *pMsg;
  }

  void getLandsharkNavStatus(const std_msgs::String::ConstPtr& pMsg) {
    this->msgLandsharkNavStatus = *pMsg;
  }

  void getBaseVelocity(const geometry_msgs::TwistStamped::ConstPtr& pMsg) {
    this->msgBaseVelocity = *pMsg;
  }

  void getMoveBaseFeedback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& pMsg) {
    this->msgMoveBaseFeedback = *pMsg;
  }

  void step(const radl_in_t* i, const radl_in_flags_t* i_f,
            radl_out_t* o, radl_out_flags_t* o_f) {
    //Forward inputs to sandbox
    std_msgs::Int32 out_msg;
//-------------------------------------------------------------------
//landshark_gps->landshark_gps_meters
//-------------------------------------------------------------------
    sensor_msgs::NavSatFix msgGps;
    msgGps.latitude = i->landshark_gps_stub->latitude;
    msgGps.longitude = i->landshark_gps_stub->longitude;
    msgGps.altitude = i->landshark_gps_stub->altitude;
//   msgGps.stamp = i->landshark_gps->stamp;

    printf("msgGps.latitude = %15.10f\n",msgGps.latitude);
   
    this->pubGps.publish(msgGps);
//re-publish to radl
    o->landshark_gps_meters_stub->x = this->msgGpsMeters.point.x;
    o->landshark_gps_meters_stub->y = this->msgGpsMeters.point.y;

//-------------------------------------------------------------------
//odom = gps + imu for landshark_2dnav
//-------------------------------------------------------------------

//obtained this covariance matrix from landshark_sim

    msgOdom.header.stamp = ros::Time::now();

    this->m_FootprintFrame = "/base_footprint";
    this->m_WheelOdomRefFrame = "/odom" ;

    msgOdom.child_frame_id = this->m_FootprintFrame;
    msgOdom.header.frame_id = this->m_WheelOdomRefFrame;

    msgOdom.pose.covariance = {0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03};

    msgOdom.pose.pose.position.x = msgGpsMeters.point.x;
    msgOdom.pose.pose.position.y = msgGpsMeters.point.y;
    msgOdom.pose.pose.position.z = msgGpsMeters.point.z;
//set to dummy values
    msgOdom.pose.pose.orientation.z = i->landshark_imu_stub->orientation.z;
    msgOdom.pose.pose.orientation.w = i->landshark_imu_stub->orientation.w;
//check: should use both (vx,vy)?
    msgOdom.twist.twist.linear.x = i->landshark_gps_vel_stub->x;
//    msgOdom.twist.twist.linear.y = i->landshark_gps_vel_stub->y;
    msgOdom.twist.twist.angular.z = i->landshark_imu_stub->orientation.w;

    printf("msgOdom.pose.pose.position.x = %15.10f\n",msgOdom.pose.pose.position.x);

    this->pubOdom.publish(msgOdom);

//-------------------------------------------------------------------
//base_velocity fwd from landshark_2dnav
//-------------------------------------------------------------------

    o->landshark_base_velocity->linear.x = msgBaseVelocity.twist.linear.x;
    o->landshark_base_velocity->angular.z = msgBaseVelocity.twist.angular.z;

    printf("o->landshark_base_velocity->linear.x = %15.10f\n",o->landshark_base_velocity->linear.x);
    printf("o->landshark_base_velocity->angular.z = %15.10f\n",o->landshark_base_velocity->angular.z);

  }

private:
  ros::NodeHandle h;

  ros::Publisher pubBaseVelocity;
  ros::Publisher pubDeadman;

  ros::Subscriber subGps;



  ros::Subscriber subLandsharkNavStatus;
  ros::Subscriber subBaseVelocity;
  ros::Subscriber subMoveBaseFeedback;
  ros::ServiceClient clientLandsharkNavWaypoint;
  geometry_msgs::PointStamped msgGpsMeters;
  geometry_msgs::TwistStamped msgBaseVelocity;
  move_base_msgs::MoveBaseActionFeedback msgMoveBaseFeedback;
  nav_msgs::Odometry msgOdom;
  std_msgs::String msgLandsharkNavStatus;
  landshark_2dnav::LandsharkNavWaypoint srvLandsharkWaypoint;
  std::string m_FootprintFrame;
  std::string m_WheelOdomRefFrame;

};

#endif //GATEWAY_BASE_VELOCITY_SIM_H












#topics to route
#/landshark_control/base_velocity

Type: geometry_msgs/TwistStamped

#/landshark_control/deadman

Type: landshark_msgs/BoolStamped


#/landshark/gps

jason@jason-ThinkPad-X1-Carbon:~/Desktop/ros_groovy_base_radl_cmu/cmu$ rostopic hz /landshark/gps
subscribed to [/landshark/gps]
average rate: 5.128
Type: sensor_msgs/NavSatFix
---
header: 
  seq: 878
  stamp: 
    secs: 1423269523
    nsecs: 436166779
  frame_id: ''
status: 
  status: 0
  service: 0
latitude: 37.4571195072
longitude: -122.173508294
altitude: 0.0
position_covariance: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
position_covariance_type: 0






#/landshark/imu

jason@jason-ThinkPad-X1-Carbon:~/Desktop/ros_groovy_base_radl_cmu/cmu$ rostopic hz /landshark/imu
subscribed to [/landshark/imu]
average rate: 49.730

Type: sensor_msgs/Imu

header: 
  seq: 17704
  stamp: 
    secs: 1423269702
    nsecs: 166852491
  frame_id: /landshark/imu
orientation: 
  x: -0.510182590366
  y: 0.860063301703
  z: 0.000942546737929
  w: 0.0019882545514
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity: 
  x: 0.0
  y: 0.0
  z: 0.0
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration: 
  x: -0.0429853774061
  y: -0.0039970068488
  z: -9.80990500878
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


/landshark/odom

average rate: 25.335

Type: nav_msgs/Odometry

header: 
  seq: 16437
  stamp: 
    secs: 1423270000
    nsecs: 938760469
  frame_id: /odom
child_frame_id: /base_footprint
pose: 
  pose: 
    position: 
      x: 1.17694047139
      y: 4.18738456163
      z: 0.366288408881
    orientation: 
      x: 0.0
      y: 0.0
      z: -0.609005819359
      w: 0.793165753161
  covariance: [0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 99999.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.03]
twist: 
  twist: 
    linear: 
      x: 0.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: 0.0
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

