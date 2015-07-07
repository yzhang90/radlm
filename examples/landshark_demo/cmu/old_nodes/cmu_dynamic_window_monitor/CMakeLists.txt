cmake_minimum_required(VERSION 2.8.3)
project(cmu_dynamic_window_monitor)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs sensor_msgs nav_msgs geometry_msgs
  landshark_msgs landshark_gps msg_auth
  cmu_control_virtualizer_stub
  cmu_z_test_monitor
  message_generation #http://wiki.ros.org/msg#Building_.msg_Files
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   DIRECTORY msg
   FILES obstacle.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs
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

## Declare a cpp library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/${PROJECT_NAME}_node.cpp
# )

SET_SOURCE_FILES_PROPERTIES(
	src/dwmonitor.c
	PROPERTIES
        COMPILE_FLAGS "-std=gnu99 -pedantic-errors -msse4.2 -Wa,-ahl=dwmonitor.s"
#        COMPILE_FLAGS "-std=c99 -msse4.2"
#	EXTERNAL_OBJECT true
#	GENERATED true
)

## Declare a cpp executable
 add_executable( 
  ${PROJECT_NAME}_node 
  src/${PROJECT_NAME}_node.cpp src/dwmonitor.c)
## Specify libraries to link a library or executable target against
 target_link_libraries( ${PROJECT_NAME}_node ${catkin_LIBRARIES} )
 ## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies( 
 ${PROJECT_NAME}_node
 ${PROJECT_NAME}_generate_messages_cpp
 )
# add_dependencies( ${PROJECT_NAME}_node 
# ${PROJECT_NAME}_generate_messages 
# ${PROJECT_NAME}_generate_messages_cpp
# ${PROJECT_NAME}_gencpp
# ${PROJECT_NAME}_genpy )
 

#add_library(dwmonitor SHARED src/dwmonitor.o)
#SET_SOURCE_FILES_PROPERTIES(
#	src/dwmonitor.c
#	PROPERTIES
#        COMPILE_FLAGS "-std=gnu99 -pedantic-errors -msse4.2 -Wa,-ahl=dwmonitor.s"
##        COMPILE_FLAGS "-std=c99 -msse4.2"
##	EXTERNAL_OBJECT true
##	GENERATED true
#)

# add_executable(dwmonitor src/dwmonitor.c)
# set_target_properties(dwmonitor PROPERTIES 
# COMPILE_FLAGS "-o dwmonitor.o -fPIC")
 
# add_library(dwmonitor SHARED src/dwmonitor.o)
# SET_SOURCE_FILES_PROPERTIES(
#	 src/dwmonitor.o
#	 PROPERTIES
#	 EXTERNAL_OBJECT true
#	 GENERATED true
# )

# SET_TARGET_PROPERTIES(
# 	dwMonitor
# 	PROPERTIES
# 	LINKER_LANGUAGE C
# )





#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

foreach( resource launch )
  install(DIRECTORY ${resource}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE
    )
endforeach()

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  script/change_obstacle_avoidance_state.py  
  script/reset_alarm.py  
  script/set_obstacle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS 
  ${PROJECT_NAME}_node 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cmu_monitors.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
