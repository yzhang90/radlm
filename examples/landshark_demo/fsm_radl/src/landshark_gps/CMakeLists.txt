cmake_minimum_required(VERSION 2.8.3)
project(landshark_gps)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs geometry_msgs landshark_msgs )

include_directories( ${catkin_INCLUDE_DIRS} src ros/src )

# include radler header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../sri/radler/radler/lib/)

catkin_package(
  CATKIN_DEPENDS sensor_msgs geometry_msgs 
  )

add_executable( landshark_gps_node 
  ros/src/landshark_gps_node.cpp
  src/LandsharkGps.cpp )

target_link_libraries( landshark_gps_node
  ${catkin_LIBRARIES}
  )

foreach( resource launch )
  install(DIRECTORY ${resource}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE
    )
endforeach( )

install( TARGETS 
  landshark_gps_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

