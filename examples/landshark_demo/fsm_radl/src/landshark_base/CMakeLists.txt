cmake_minimum_required(VERSION 2.8.3)
project(landshark_base)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs landshark_msgs )
find_package( Boost REQUIRED COMPONENTS thread )
set( CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake" )
find_package( USB4 REQUIRED )

include_directories( 
  src
  ros/src
  ${USB4_INCLUDE_DIR} 
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS})

# include radler header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../radler/radler/lib/)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs landshark_msgs
  )

add_executable( landshark_base_node 
  ros/src/landshark_base_node.cpp
  src/LandsharkBase.cpp
  src/usdigital.c
  )
target_link_libraries(landshark_base_node
  ${USB4_LIBRARIES} 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
  )

install( TARGETS 
  landshark_base_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

