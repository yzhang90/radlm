cmake_minimum_required(VERSION 2.8.3)
project(landshark_moog)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs )
find_package(Boost REQUIRED COMPONENTS thread)

include_directories( src ros/src ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# include radler header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../sri/radler/radler/lib/)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  )

add_executable( landshark_moog_node
  ros/src/landshark_moog_node.cpp
  src/LandsharkMoog.cpp
  src/serial.c
  )
target_link_libraries( landshark_moog_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  )

foreach( resource launch )
  install(DIRECTORY ${resource}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE
    )
endforeach( )

install( TARGETS 
  landshark_moog_node 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

