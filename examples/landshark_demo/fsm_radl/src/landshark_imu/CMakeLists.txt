cmake_minimum_required(VERSION 2.8.3)
project(landshark_imu)

find_package( catkin REQUIRED COMPONENTS roscpp landshark_msgs geometry_msgs )
find_path( bullet_INCLUDE_DIRS LinearMath/btMatrix3x3.h 
  PATHS /usr/include/bullet )
if ( bullet_INCLUDE_DIRS )
  message( STATUS "bullet_INCLUDE_DIRS : ${bullet_INCLUDE_DIRS}" )
else()
  message( STATUS "bullet not found: Please install libbullet-dev" ) 
  message( STATUS "Please install libbullet-dev" )
  message( FATAL_ERROR "bullet_INCLUDE_DIRS : ${bullet_INCLUDE_DIRS}" )
endif ()

include_directories( ${catkin_INCLUDE_DIRS} src ros/src )

# include radler header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../sri/radler/radler/lib/)

catkin_package(
  CATKIN_DEPENDS geometry_msgs landshark_msgs 
  )

add_executable( landshark_imu_node 
  ros/src/landshark_imu_node.cpp
  src/LandsharkImu.cpp
  src/3dmgx2.cpp
  )

target_link_libraries( landshark_imu_node 
  ${catkin_LIBRARIES}
  )

foreach( resource launch )
  install(DIRECTORY ${resource}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE
    )
endforeach( )

install( TARGETS 
  landshark_imu_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

