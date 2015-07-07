#!/usr/bin/env python

import re
import numpy 
import sys
import subprocess
from subprocess import check_output

import numpy
import scipy.misc
from matplotlib import pyplot
import yaml

import rospy
import roslaunch
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion

class MapServer():
  def __init__(self):
    rospy.init_node('landshark_2dnav_map_server')

    self.land_2d_dir = sys.argv[1]

    self.file_yaml = self.land_2d_dir + '/map/map.yaml'
    self.image_file = self.land_2d_dir + '/map/map_obstacles.pgm'
    rospy.loginfo("map.yaml =  %s", self.file_yaml)
    rospy.loginfo("*.png =  %s", self.image_file)
    self.dataMap = self.mapMetaRead()
    self.origin = self.dataMap['origin']
    self.res = self.dataMap['resolution']
    self.height = int(2*abs(self.origin[0])/self.res) + 1 #2* for origin centered at (0,0)
    self.width = int(2*abs(self.origin[1])/self.res) + 1 #2* for origin centered at (0,0)
#make white (unoccupied)
    self.image = numpy.zeros((self.height,self.width), dtype=numpy.uint8)
    self.image.fill(255)
    self.image_obs = numpy.zeros((self.height,self.width), dtype=numpy.uint8)
    self.image_obs.fill(255)   
#set system cmds to control map_server
    self.map_server_cmd = "rosrun map_server map_server " + self.file_yaml
    self.rosnode_list_cmd = "rosnode list"
    self.rosnode_kill_cmd = "rosnode kill "

    self.msgMapRadl = Path()
    self.msgMapRadl_old = Path()
    self.map_length = 16 #FIX ME

#zero initial position
    poseStamped = PoseStamped();
    poseStamped.pose.position.x = 0.0
    poseStamped.pose.position.y = 0.0
#    rospy.loginfo("create blank msgMapRadl_old")
    for idx in enumerate(numpy.arange(1,self.map_length)):
      self.msgMapRadl_old.poses.append(poseStamped)

#    rospy.loginfo("subscribe to /landshark/map_meters")
    rospy.Subscriber('/landshark/map_meters', Path, self.getMapRadl, queue_size=10)
#    rospy.loginfo("setObstacles")
#    self.setObstacles()    
#    rospy.loginfo("run map_server")
#    self.runMapServer()
    rospy.loginfo("spin()")
    rospy.spin()

  def getMapRadl(self,msg):
    self.msgMapRadl=msg
#    rospy.loginfo("check if map has changed")
    idx = 0
    for pose in self.msgMapRadl.poses:
      if pose.pose.position.x != self.msgMapRadl_old.poses[idx].pose.position.x or pose.pose.position.y != self.msgMapRadl_old.poses[idx].pose.position.y or pose.pose.position.z != self.msgMapRadl_old.poses[idx].pose.position.z:
        rospy.loginfo("map has changed")
        self.msgMapRadl_old = self.msgMapRadl
        rospy.loginfo("killMapServer")
        self.killMapServer()
#        rospy.loginfo("setObstacles")
        self.setObstacles()
#        rospy.loginfo("mapMetaWrite")
#        self.mapMetaWrite()
        rospy.loginfo("runMapServer")
        self.runMapServer()
        return
      idx=idx+1

  def mapMetaRead(self):
    f = open(self.file_yaml)
    return yaml.safe_load(f)

  def mapMetaWrite(self): #FIX ME: output.yaml config slightly different
    with open(self.file_yaml, 'w') as outfile:
      outfile.write(yaml.dump(self.dataMap, default_flow_style=False))
    return

  def runMapServer(self):
#    rospy.loginfo("subprocess...")
    self.p = subprocess.Popen(self.map_server_cmd, stdin=subprocess.PIPE, shell=True)
#    rospy.loginfo(self.p.pid)

  def killMapServer(self): 
#    rospy.loginfo("killing map_server")
    self.p_rosnode_list = subprocess.Popen(self.rosnode_list_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
    out, err = self.p_rosnode_list.communicate()
#    rospy.loginfo("rosnode list out.splitlines() = %s",out.splitlines())
    pattern="/map_server_*" #FIX ME
    for o in out.splitlines():
      match = re.search(pattern, o)
      if match:
#        rospy.loginfo("match.string = %s", match.string)
        self.p_rosnode_kill = subprocess.Popen(self.rosnode_kill_cmd+match.string, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)

  def setObstacles(self):
# create empty image_obs
    self.image_obs = numpy.zeros((self.height,self.width), dtype=numpy.uint8)
    self.image_obs.fill(255)
    self.image_obs.setflags(write=True)
    xmin = self.origin[0]
    ymin = self.origin[1]
    rospy.loginfo("int(xmin/res) = %s", int(xmin/self.res))
    rospy.loginfo("int(ymin/res) = %s", int(ymin/self.res))
    shape = self.image_obs.shape
    xmin_px = int(xmin/self.res)
    ymin_px = int(ymin/self.res)

#dummy set corner
#    self.image_obs[ 0 , 0 ] = 1

    for pose in self.msgMapRadl.poses:
      obs_x = pose.pose.position.x
      obs_y = pose.pose.position.y
      obs_r = pose.pose.position.z
      obs_x_px = 1*int(round(obs_x/self.res))
      obs_y_px = -1*int(round(obs_y/self.res)) #FIX ME flip for pixel representaiton
      obs_r_px = int(round(obs_r/self.res))
      range_x_lo = obs_x_px - xmin_px - obs_r_px
      range_x_hi = obs_x_px - xmin_px + obs_r_px
      range_y_lo = obs_y_px - ymin_px - obs_r_px
      range_y_hi = obs_y_px - ymin_px + obs_r_px
      if range_x_lo >= 0 and range_x_hi <= shape[0] and range_y_lo >= 0 and range_y_hi <= shape[1]:
        self.image_obs[ range_y_lo:range_y_hi + 1 , range_x_lo:range_x_hi + 1 ] = 1 #FIX ME y orientation, +1 for origin offset  
      else:
        rospy.loginfo("obstacle is off map: obs_x = %s, obs_y = %s, obs_r = %s", obs_x, obs_y, obs_r)
    scipy.misc.imsave(self.image_file, self.image_obs)  
    return
  

if __name__ == "__main__":
  try:
    mapServer = MapServer()
  except rospy.ROSInterruptException: 
    sys.exit

