cmake_minimum_required(VERSION 2.8.3)
project(landshark_gps_meters)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs sensor_msgs nav_msgs geometry_msgs
  message_generation #http://wiki.ros.org/msg#Building_.msg_Files
#  proj4
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cmu_monitors
  CATKIN_DEPENDS message_runtime roscpp rospy
  std_msgs sensor_msgs nav_msgs geometry_msgs
  LIBRARIES  ${PROJECT_NAME}
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

## Declare a cpp library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/${PROJECT_NAME}_node.cpp
# )

#SET_SOURCE_FILES_PROPERTIES(
#	src/dwmonitor.c
#	PROPERTIES
#        COMPILE_FLAGS "-std=gnu99 -pedantic-errors -msse4.2 -Wa,-ahl=dwmonitor.s"
#        COMPILE_FLAGS "-std=c99 -msse4.2"
#	EXTERNAL_OBJECT true
#	GENERATED true
#)


add_library( ${PROJECT_NAME} SHARED src/GpsProjection.cpp)
target_link_libraries( ${PROJECT_NAME} proj)

## Declare a cpp executable
 add_executable( 
  ${PROJECT_NAME}_node 
  src/${PROJECT_NAME}_node.cpp 
#  src/GpsProjection.cpp
)
## Specify libraries to link a library or executable target against
 target_link_libraries( ${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} )
 ## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies( 
# ${PROJECT_NAME}_node
# ${PROJECT_NAME}_generate_messages_cpp
# )

#add_executable(${PROJECT_NAME}_node src/LandsharkGpsNode.cpp)
#target_link_libraries(${PROJECT_NAME}_node 
#  landshark_gps_driver
#  ${catkin_LIBRARIES}
#)

# add_dependencies( ${PROJECT_NAME}_node 
# ${PROJECT_NAME}_generate_messages 
# ${PROJECT_NAME}_generate_messages_cpp
# ${PROJECT_NAME}_gencpp
# ${PROJECT_NAME}_genpy )
 
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS 
  ${PROJECT_NAME}
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach( resource launch )
  install(DIRECTORY ${resource}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE
    )
endforeach()

