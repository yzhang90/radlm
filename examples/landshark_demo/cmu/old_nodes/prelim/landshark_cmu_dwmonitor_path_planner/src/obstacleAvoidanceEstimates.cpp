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
 * \file    obstacleAvoidanceEstimates.cpp
 * \brief   performs naive obstacle avoidance and local planning
 * \author  Fatma Faruq 
 * \other   HACMS CMU Advisor Manuela Veloso
 */
//========================================================================

#include "obstacleAvoidanceEstimates.h"

void obstacleAvoidanceEst::goToTargetAvoidObstacles()
{
  double wc = 0.6;
  double wLimFacing = 0.1;
  double wLimSide = 0.8;
  double wLimMiddle = 0.4;
  double facingAngle = 1.5;
  double sideAngle = 1.2;
  double wDecidingClearance = 1.3;
  double extraClearance = 1.5;
  nodeHandle.getParam("wClearance",wDecidingClearance);
  nodeHandle.getParam("eClearance",extraClearance);
  float maxv, maxw, minv, minw;
  float currv = 0;
  float currw = 0;
  float robotR = 1;
  float obsR = 1;
  float acceptableError = (obsR + robotR) * 0.5;
  bool haveReachedTarget = false;
  bool tooClose = false;
  bool checkedOnce = false;
  eV.eS.generateVelocityRange(currv, currw, maxv, minv, maxw, minw);
  float numV = 5;
  float numW = numV;
  float vInc = (fabs(maxv - minv) / numV);
  float wInc = (fabs(maxw - minw) / numW);
  ros::spinOnce();
  bool displayStuff = false;
  float x1 = 0;
  float x2 = 0;
  float x3 = 0;
  float y1 = 0;
  float y2 = 0;
  float y3 = 0;
  float v1 = 0;
  float v2 = 0;
  float v3 = 0;
  float w1 = 0;
  float w2 = 0;
  float w3 = 0;
  bool rotateAway = false;
  float rv = 0;

  while (!haveReachedTarget && !tooClose)
  {
    float maxG = -10000;
    float currG = maxG;
    tooClose = false;
    bool anyChecked = false;
    checkedOnce = false;
    x1 = x2;
    y1 = y2;
    x2 = x3;
    y2 = y3;
    x3 = eV.eS.robotFunctions.gpsPosition.point.x;
    y3 = eV.eS.robotFunctions.gpsPosition.point.y;
    v1 = v2;
    v2 = v3;
    v3 = currv;
    w1 = w2;
    w2 = w3;
    w3 = currw;

    for (float v = minv; v <= maxv; v += vInc)
    {
      for (float w = minw; w <= maxw; w += wInc)
      {
        ros::spinOnce();
        float distToT = eV.eS.distanceBetweenPointStampedNoZ(eV.eS.target, eV.eS.robotFunctions.gpsPosition);
        float desV = 1.1;
        float stoppingDist = 10;
        if (distToT < stoppingDist)
        {
          desV = distToT / stoppingDist;
        }
        float reqHeading = eV.eS.robotFunctions.getHeading2(eV.eS.target, eV.eS.robotFunctions.gpsPosition,
                                                            eV.eS.robotFunctions.odomAngle, 0, 0);
        float otherHeading = eV.eS.robotFunctions.getOtherHeading(reqHeading);
        float headingDiff = fabs(w - reqHeading);
        float headingDiff2 = fabs(w - otherHeading);
        if (headingDiff2 < headingDiff)
          headingDiff = headingDiff2;
        float robot_x = eV.eS.robotFunctions.gpsPosition.point.x;
        float robot_y = eV.eS.robotFunctions.gpsPosition.point.y;
        float obs_x = 0;
        float obs_y = 0;
        int obsNo = -1;
        float distToObs = eV.eS.findClosestObstacleLine(robot_x, robot_y, obs_x, obs_y, obsNo);
        float breakDist = v / (2 * eV.eS.accv) + v * eV.eS.timePeriod
            + eV.eS.robotFunctions.currentLinearSpeed / (2 * eV.eS.accv);
        breakDist += robotR + obsR;
        float currAngle = eV.eS.robotFunctions.odomAngle;
        float newx = 0;
        float newy = 0;
        float error = 0;
        float currx = robot_x;
        float curry = robot_y;
        eV.eS.predictLocation(currx, curry, currAngle, v, w, newx, newy, error);
        float newObsx = 0;
        float newObsy = 0;
        int newObsNo = -1;
        distToObs = eV.eS.findClosestObstacleLine(newx, newy, newObsx, newObsy, newObsNo);
        
        bool considerThis = false;
       // printf("\nOdometry Angle = %f otherOne = %f roll = %f yaw = %f pitch = %f\n", eV.eS.robotFunctions.odomAngle,
       //        eV.eS.robotFunctions.oldOdomAngle, eV.eS.robotFunctions.imuRoll, eV.eS.robotFunctions.imuYaw,
       //        eV.eS.robotFunctions.imuPitch);
       // printf("\nBraking Distance %f \nClosest Obs right now %f %f\n Predicted %f %f", breakDist, obs_x, obs_y,
       //        newObsx, newObsy);
       // printf("\nDistance = %f %f\n", (newObsx - newx), (newObsy - newy));

        float angleToObs = eV.eS.robotFunctions.getHeading2(eV.eS.obstacles[obsNo], eV.eS.robotFunctions.gpsPosition,
                                                            eV.eS.robotFunctions.odomAngle, 0, 0);
        float newAngleToObs = eV.eS.robotFunctions.getHeading2(eV.eS.obstacles[newObsNo],
                                                               eV.eS.robotFunctions.gpsPosition,
                                                               eV.eS.robotFunctions.odomAngle, 0, 0);

        if (fabs(newObsx - newx) > (breakDist + extraClearance) || (fabs(newObsy - newy) > (breakDist + extraClearance)))
        {
          considerThis = true;
          anyChecked = true;
        }
        else
        {

          //if i am too close to the obstacle like within 0.1
          //then if i'm facing the obstacle I want to stop and rotate away
          //otherwise I want to turn at 1 so that i totally avoid it
          if (fabs(obs_x - robot_x) < wDecidingClearance && fabs(obs_y - robot_y) < wDecidingClearance)
          {
            //am I facing the obstacle
            if (fabs(angleToObs) < facingAngle)
            {
              wc = wLimFacing;
              if (fabs(v) <= 0)
              {
                if (fabs(v) < fabs(wc))
                {
                  if (angleToObs > 0)
                  {
                    if (w < -1 * wc)
                    {
                      considerThis = true;
                      anyChecked = true;
                    }
                  }
                  else
                  {
                    if (w > wc)
                    {
                      considerThis = true;
                      anyChecked = true;
                    }
                  }
                }
                else
                {
                  considerThis = false;
                }
              }
              else
              {
                considerThis = false;
              }
            }
            else
            {
              wc = wLimSide;
              if (angleToObs > 0)
              {
                if (w < -1 * wc)
                {
                  considerThis = true;
                  anyChecked = true;
                }
              }
              else
              {
                if (w > wc)
                {
                  considerThis = true;
                  anyChecked = true;
                }
              }
            }

          }
          else
          {
            if (fabs(angleToObs) < sideAngle || fabs(newAngleToObs) < sideAngle)
            {
              wc = wLimSide;
              if (angleToObs > 0)
              {
                if (w < -1 * wc)
                {
                  considerThis = true;
                  anyChecked = true;
                }
              }
              else
              {
                if (w > wc)
                {
                  considerThis = true;
                  anyChecked = true;
                }
              }
            }
            else
            {
              if (fabs(angleToObs) < 1.5 || fabs(newAngleToObs) < 1.5)
              {
                wc = wLimMiddle;
                if (angleToObs > 0)
                {
                  if (w < -1 * wc)
                  {
                    considerThis = true;
                    anyChecked = true;
                  }
                }
                else
                {
                  if (w > wc)
                  {
                    considerThis = true;
                    anyChecked = true;
                  }
                }
              }
              else
              {
                considerThis = true;
                anyChecked = true;

              }
            }
          }
        }
      //  cout << "\nFine Till here\n";
        if (considerThis)
        {
          double f_V = v;
          float X[5];
          X[0] = eV.eS.robotFunctions.currentLinearSpeed;
          X[1] = robot_x;
          X[2] = robot_y;
          X[3] = obs_x;
          X[4] = obs_y;
          double D[3];
          D[0] = (double)((eV.eS.accv / eV.eS.accv + 1) * (eV.eS.accv / 2 * eV.eS.timePeriod * eV.eS.timePeriod)
              + (f_V * eV.eS.timePeriod));
          D[1] = (double)((f_V / eV.eS.accv) + (eV.eS.timePeriod * (eV.eS.accv / eV.eS.accv + 1)));
          D[2] = (double)(1 / (2 * eV.eS.accv));
         // printf("\ncurrent Speed : %f %f\n", eV.eS.robotFunctions.currentLinearSpeed, currv);
          int result = dwmonitor(X, D);
          if (result == 1 || result == -1)
          {
            considerThis = true;
            checkedOnce = true;
          }
          else
          {
            considerThis = false;
          }
        }
        if (considerThis)
        {

          float clearance = (distToObs - breakDist) / (eV.eS.dmax - breakDist);
          currG = (1 / headingDiff) + clearance * 10 + 1 / (fabs(desV - v));
          if (currG > maxG)
          {
            maxG = currG;
            currv = v;
            currw = w;
          }
        }

      }
    }
    //  tooClose = !anyChecked;
    tooClose = !checkedOnce;
    if (tooClose)
    {
      printf(
          "\n-------------------------------------------------------\nToo Close\n------------------------------------------------------\n");
    }
    ros::Time begin = ros::Time::now();
    ros::Time end = ros::Time::now();
    while ((end - begin).toSec() < eV.eS.timePeriod)
    {
      ros::spinOnce();
      eV.eS.robotFunctions.moveRobot(currv, currw);
      end = ros::Time::now();
    }
    float distanceToTarget = eV.eS.distanceBetweenPointStampedNoZ(eV.eS.robotFunctions.gpsPosition, eV.eS.target);
    //eV.eS.robotFunctions.displayGPS();
    //printf("\nDistance To Target %f", distanceToTarget);
    //cin.get();
    float prevHeading = eV.eS.robotFunctions.getHeading2(eV.eS.target, eV.eS.robotFunctions.gpsPosition,
                                                         eV.eS.robotFunctions.odomAngle, 0, 0);
    if (distanceToTarget <= acceptableError)
    {
      haveReachedTarget = true;
      eV.eS.robotFunctions.moveRobot(0, 0);
      printf("\nReached Target\n");
    }
  }

}


obstacleAvoidanceEst::obstacleAvoidanceEst()
{

}
obstacleAvoidanceEst::obstacleAvoidanceEst(ros::NodeHandle nh)
{
  nodeHandle = nh;
  eV.initClass(nodeHandle);

}

void obstacleAvoidanceEst::initClass(ros::NodeHandle nh)
{
  nodeHandle = nh;
  eV.initClass(nodeHandle);

}

void obstacleAvoidanceEst::predictLocation(float currx, float curry, float currAngle, float v, float w, float &newx,
                                           float &newy, float& error)
{
  float dx = 0;
  float dy = 0;
  float dt = eV.eS.timePeriod;
  if (w == 0)
  {
    dx = currx + v * cos(currAngle) * dt;
    dy = curry + v * sin(currAngle) * dt;
  }
  else
  {
    float r = (v / w);
    float robotR = 0.35;
    if (r > 0)
    {
      r = r + robotR;
    }
    else
    {
      r = r - robotR;
    }
    float xc = currx - r * sin(currAngle);
    float yc = curry + r * cos(currAngle);
    dx = xc + r * sin(currAngle + w * dt);
    dy = yc - r * cos(currAngle + w * dt);
  }
  newx = dx;
  newy = dy;
  error = 0.3;

}

