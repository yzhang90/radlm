cmake_minimum_required(VERSION 2.8.3)
project(upenn_launch)

find_package(catkin REQUIRED COMPONENTS landshark_robot)

catkin_package(
  CATKIN_DEPENDS landshark_robot)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
