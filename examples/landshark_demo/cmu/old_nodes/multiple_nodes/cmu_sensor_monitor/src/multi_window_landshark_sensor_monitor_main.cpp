//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    execution_monitor_main.cpp
\brief   Run different monitors for Landshark execution
\author  Juan Pablo Mendoza
*/
//========================================================================

#include "multi_window_motion_gps_odo_anomaly_monitor.h"
#include <stdlib.h>
#include <ros/ros.h>

//rate at which monitors run
const unsigned int LOOPING_RATE = 20;
//only declare a failure if above this confidence
double anomaly_thresh;
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

int main(int argc, char **argv) {
  //Make the text look nice
  printf("\nSensor Monitor\n\n");

  //initialize ROS and the monitor
  ros::init(argc, argv, "cmu_sensor_monitor");
  ros::NodeHandle n;
  motionMonitor.init(&n);
  n.param("/cmu_sensor_monitor/max_anomaly", anomaly_thresh,0.9);
  n.param("/cmu_sensor_monitor/play_alarm_sound", play_alarm_sound,false);
  //faultPublisher=n.advertise<landshark_msgs::ExecutionFault>(
  //   "/landshark/ExecutionMonitor/Fault",1);
  
  
  // main loop
  ros::Rate loop_rate(LOOPING_RATE);
  while(ros::ok()){
    ros::spinOnce();
    timerEvent();
    loop_rate.sleep();
  }
  return(0);
}

