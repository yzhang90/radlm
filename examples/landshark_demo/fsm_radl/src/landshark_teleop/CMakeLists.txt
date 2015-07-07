cmake_minimum_required(VERSION 2.8.3)
project(landshark_teleop)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs )

include_directories(${catkin_INCLUDE_DIRS} src ros/src )

# include radler header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../sri/radler/radler/lib/)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs 
  )

add_executable( landshark_teleop_node
  ros/src/landshark_teleop_node.cpp
  src/LandsharkJoystick.cpp
  )
target_link_libraries( landshark_teleop_node
  ${catkin_LIBRARIES}
  )
  
foreach( resource launch )
  install(DIRECTORY ${resource}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE
    )
endforeach( )

install( TARGETS 
  landshark_teleop_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

