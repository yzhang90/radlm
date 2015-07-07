/*
 Copyright (c) 2013, PRECISE Center, University of Pennsylvania
 All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

 Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 Neither the name of the University of Pennsylvania nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 Author(s): Nicola Bezzo (nicbezzo@seas.upenn.edu)
            Junkil Park (junkil.park@cis.upenn.edu)
 */





#include<ros/ros.h>
#include<sensor_msgs/Joy.h>
#include<geometry_msgs/TwistStamped.h>
#include<landshark_msgs/WheelEncoder.h>

bool cruise_mode = false;
double setpoint = 0.0, trim = 0.0, auto_trim = 0.0;

void joyCallback (const sensor_msgs::Joy::ConstPtr &m)
{
  if (m->axes[2] > -0.5) { cruise_mode = false; }
  else { cruise_mode = true; }

  if (m->buttons[0] == 1) { setpoint = 0.0; }
  if (m->buttons[4] == 1) { setpoint = 0.6; }
  if (m->buttons[5] == 1) { setpoint = 0.7; }
  if (m->buttons[1] == 1) { setpoint = 0.8; }
  if (m->buttons[2] == 1) { setpoint = 0.9; }
  if (m->buttons[3] == 1) { setpoint = 1.0; }

  if (m->axes[6] > 0.5) { trim += 0.01; }
  if (m->axes[6] < -0.5) { trim -= 0.01; }

  if (m->buttons[8] == 1) { trim = 0.0; }
}

int l = 0, r = 0;
int dl,dr;

void encoderCallback(const landshark_msgs::WheelEncoder::ConstPtr &m)
{
  //static int cnt = 0;
  //cnt++;
  //if(cnt < 30) return; else cnt=0;

 // static int l, r = 0;

  dl = m->left_encoder - l;
  dr = m->right_encoder - r;

  if (dl > dr) { auto_trim += 0.002; }
  else if (dl < dr) { auto_trim -= 0.002; }
  else if (dl == dr) ;

  ROS_INFO("%d %d %lf", dl, dr, auto_trim);

  l = m->left_encoder;
  r = m->right_encoder;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "landshark_cruise");
  ros::NodeHandle node;

  ros::Rate r(100);
  ros::Publisher cmd_vel_out = node.advertise<geometry_msgs::TwistStamped>("/landshark_control/base_velocity", 1);
  ros::Subscriber joy_in = node.subscribe<sensor_msgs::Joy>("/joy", 1, joyCallback);
  ros::Subscriber encoder_in = node.subscribe<landshark_msgs::WheelEncoder>("/landshark/wheel_encoder", 1, encoderCallback);

  while(ros::ok()) {
    ros::spinOnce();

    //ROS_INFO("cruise mode : %d", cruise_mode);

    if (cruise_mode) {
      geometry_msgs::TwistStamped cmd;

      cmd.header.stamp = ros::Time::now();
      cmd.twist.linear.x = setpoint;
      cmd.twist.angular.z = trim + auto_trim;
      cmd_vel_out.publish(cmd);
    }
    r.sleep();
  }

  return 0;
}
