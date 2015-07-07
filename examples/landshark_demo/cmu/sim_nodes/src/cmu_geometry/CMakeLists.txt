cmake_minimum_required(VERSION 2.8.3)
project(cmu_geometry)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs sensor_msgs nav_msgs geometry_msgs
  landshark_msgs landshark_gps msg_auth
)

## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cmu_monitors
  CATKIN_DEPENDS message_runtime roscpp rospy
  std_msgs sensor_msgs nav_msgs geometry_msgs
  landshark_msgs landshark_gps msg_auth
  #cmu_control_virtualizer_stub
#  DEPENDS ${PROJECT_NAME}_generate_messages
#  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

#foreach( resource launch )
#  install(DIRECTORY ${resource}
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    PATTERN "*~" EXCLUDE
#    )
#endforeach()

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/geometry.py scripts/rotation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)

## Mark executables and/or libraries for installation
install(TARGETS 
#  ${PROJECT_NAME}_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/geometry.py scripts/rotation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

