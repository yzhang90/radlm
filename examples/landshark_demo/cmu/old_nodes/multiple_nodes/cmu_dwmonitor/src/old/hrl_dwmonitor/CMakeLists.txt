 #      CMakeLists.txt
 #
 #      Author: Fatma Faruq, Jason Larkin 
 #      HACMS CMU Team, Advisor Manuela Veloso, SpiralGen, Inc.
 #/
cmake_minimum_required(VERSION 2.8.3)
project(hrl_dwmonitor)

find_package(
catkin REQUIRED COMPONENTS actionlib 
  geometry_msgs
 	landshark_common landshark_gps
		landshark_msgs
 	nav_msgs
 	roscpp
		sensor_msgs
		std_msgs
 	tf
 	visualization_msgs
		message_generation
)

add_message_files(
	FILES
	obstacleMsg.msg
	dwMonitorSwitch.msg
	ControlAccel.msg
	ControlSteer.msg
	GearNumber.msg
	GearRatio.msg
	IMU.msg
	Speed.msg
	WheelEncoderFront.msg
	WheelEncoderRear.msg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )

catkin_package(
  	CATKIN_DEPENDS actionlib message_runtime
  	CATKIN_DEPENDS actionlib 
	geometry_msgs 
	landshark_common 
	landshark_gps 
	landshark_msgs 
	nav_msgs 
	roscpp 
	sensor_msgs 
	tf 
	visualization_msgs 
	message_runtime
  )

add_library(dwmonitor SHARED src/dwmonitor.o)
SET_SOURCE_FILES_PROPERTIES(
	src/dwmonitor.o
	PROPERTIES
	EXTERNAL_OBJECT true
	GENERATED true
)

SET_TARGET_PROPERTIES(
	dwmonitor
	PROPERTIES
	LINKER_LANGUAGE C
)

add_executable(hrl_dwmonitor src/hrl_dwmonitor.cpp)
#src/convertToGPSClass.cpp)
#add_dependencies(${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(hrl_dwmonitor dwmonitor ${catkin_LIBRARIES})
