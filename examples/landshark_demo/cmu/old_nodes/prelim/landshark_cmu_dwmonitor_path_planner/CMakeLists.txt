cmake_minimum_required(VERSION 2.8.3)
project(landshark_cmu_dwmonitor_path_planner)

find_package(
 catkin REQUIRED COMPONENTS 
 geometry_msgs 
 cmu_ros_pub_sub 
 landshark_gps 
 landshark_msgs 
 nav_msgs 
 roscpp 
 sensor_msgs 
 std_msgs 
 tf
)

catkin_package(
 CATKIN_DEPENDS 
 geometry_msgs 
 cmu_ros_pub_sub 
 landshark_gps 
 landshark_msgs 
 nav_msgs 
 roscpp 
 sensor_msgs 
 std_msgs tf
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(testLib SHARED src/dwmonitor.o)
SET_SOURCE_FILES_PROPERTIES(
	src/dwmonitor.o
	PROPERTIES
	EXTERNAL_OBJECT true
	GENERATED true
)

SET_TARGET_PROPERTIES(
	testLib
	PROPERTIES
	LINKER_LANGUAGE C
)

add_executable(localPathPlanner src/runLocalPlanner.cpp src/obstacleAvoidanceEstimates.cpp src/EnvironmentVariables.cpp src/environmentSpecifics.cpp src/readGPSCoordinateFile.cpp)
target_link_libraries(localPathPlanner testLib ${catkin_LIBRARIES} cmu_ros_pub_sub)

