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
 * \file    testRosParameters.cpp
 * \brief   tests ros parameters from package
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================

#include <landshark_cmu_ros_pub_sub/getRosParameters.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testGetRosParameters");
  ros::NodeHandle n;
  ros::Rate rate(1000);
  getROSParameters rP(n);
  string disp = ".";
  while (ros::ok())
  {
    ros::spinOnce();
    rP.displayGPS();
    rP.displayOdomXYZ();
    rP.displayAngle();
    rP.displayIMU();
    rP.displayImuAngle();
    ros::Time start = ros::Time::now();
    ros::Time end = ros::Time::now();
    cout << "\nSo now i'm going to move this thing okay..\n";
    while ((end - start).toSec() < 10)
    {
      rP.moveTurret(0.2, 0.1);
      rP.moveRobot(1, 0);
      end = ros::Time::now();
      if (disp == "...")
        cout << "\xd";
      else
      {
        cout << disp;
        disp += ".";
      }

    }
    cout << "\nDone\n";
    cin.get();
    rate.sleep();
  }

  return 0;
}
