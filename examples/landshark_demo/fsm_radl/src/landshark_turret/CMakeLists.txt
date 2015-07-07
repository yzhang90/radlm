cmake_minimum_required(VERSION 2.8.3)
project(landshark_turret)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs ) 
find_package(Boost REQUIRED COMPONENTS thread)

# include radler header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../sri/radler/radler/lib/)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs 
  )

include_directories( src 
  ros/src
  src/galil/dmclnxlib
  src/galil/PCIDriver
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS}
  )

add_library( landshark_turret STATIC
  src/galil/dmclnxlib/dmcbin.c
  src/galil/dmclnxlib/dmcdma.c  
  src/galil/dmclnxlib/dmceth.c
  src/galil/dmclnxlib/dmcfmw.c 
  src/galil/dmclnxlib/dmcint.c 
  src/galil/dmclnxlib/dmcisa.c 
  src/galil/dmclnxlib/dmclnx.c 
  src/galil/dmclnxlib/dmcpci.c 
  src/galil/dmclnxlib/dmcser.c 
  src/galil/dmclnxlib/dmcsup.c
  )

add_executable( landshark_turret_node
  ros/src/landshark_turret_node.cpp
  src/LandsharkTurret.cpp
  )

target_link_libraries( landshark_turret_node 
  landshark_turret
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  )

install( TARGETS 
  landshark_turret
  landshark_turret_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
