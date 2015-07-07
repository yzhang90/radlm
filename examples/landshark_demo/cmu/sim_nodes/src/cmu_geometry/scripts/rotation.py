#!/usr/bin/env python

import sys
import rospy
from landshark_msgs.msg import JointControl 

class Rotation():
  def __init__(self):
    rospy.init_node('cmu_rotation')
    self.pubMoogJointVelocity = rospy.Publisher('/landshark_control/moog_joint_velocity', JointControl)
    self.pubTurretJointVelocity = rospy.Publisher('/landshark_control/turret_joint_velocity', JointControl)
    self.jointName = []
    self.jointName.append('moog_tilt')
    self.jointName.append('moog_pan')
    self.turretName = []
    self.turretName.append('turret_tilt')
    self.turretName.append('turret_pan')

    self.velocity = 0.1

  def PublishJointControl(self,angle_desired):

    print "im in PublishJointControl"

    time = abs((angle_desired*3.14/180.0)/self.velocity) #FIX ME pi
    if angle_desired <= 0.0:
      desired_velocity = -self.velocity
    else:
      desired_velocity = self.velocity  

    jointControlMoog = JointControl()
    jointControlMoog.header.stamp = rospy.get_rostime()
    jointControlMoog.mode = JointControl.VELOCITY_CONTROL
    jointControlMoog.name = [self.jointName[0]]
    jointControlMoog.value = [desired_velocity]

    jointControlTurret = JointControl()
    jointControlTurret.header.stamp = rospy.get_rostime()
    jointControlTurret.mode = JointControl.VELOCITY_CONTROL
    jointControlTurret.name = [self.turretName[0]]
    jointControlTurret.value = [desired_velocity] 

    print "time = ", time

    r = rospy.Rate(30)
    start = rospy.get_rostime() #now.secs, now.nsecs
    while not rospy.is_shutdown():
      now = rospy.get_rostime() #now.secs, now.nsecs

#      print "start.secs - now.secs + (start.nsecs - now.nsecs)/1.0e9 > time", start.secs - now.secs + (start.nsecs - now.nsecs)/1.0e9 > time

      print "now.secs - start.secs + (now.nsecs - start.nsecs)/1.0e9 = ", now.secs - start.secs + (now.nsecs - start.nsecs)/1.0e9 

      if now.secs - start.secs + (now.nsecs - start.nsecs)/1.0e9 < time:
        self.pubMoogJointVelocity.publish(jointControlMoog)  
        self.pubTurretJointVelocity.publish(jointControlTurret)
 
      r.sleep()

if __name__ == "__main__":
  print "im in main"
  
  myRotation = Rotation()  
  myRotation.PublishJointControl(float(sys.argv[1]))


