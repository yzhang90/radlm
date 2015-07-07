cmake_minimum_required(VERSION 2.8.3)
project(landshark_hokuyo)

find_package(Boost REQUIRED COMPONENTS system thread)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_updater
  driver_base
  dynamic_reconfigure
  rosconsole
  roscpp
  self_test
  sensor_msgs
)

include_directories( ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ros/src src )

generate_dynamic_reconfigure_options(ros/cfg/Hokuyo.cfg)

# include radler header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../../radler/radler/lib/)

catkin_package(
  CATKIN_DEPENDS rosconsole
)

add_library( radl_hokuyo src/hokuyo.cpp src/HokuyoRadlNode.cpp )
add_executable( get_hokuyo_id src/hokuyo.cpp src/get_hokuyo_id.cpp)
target_link_libraries( get_hokuyo_id 
  ${Boost_LIBRARIES}
  )

if ( 1 )
  add_executable(landshark_hokuyo_node src/hokuyo.cpp ros/src/hokuyo_node.cpp)
  add_dependencies(landshark_hokuyo_node ${PROJECT_NAME}_gencfg)
  target_link_libraries(landshark_hokuyo_node
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    ${rosconsole_LIBRARIES}
  )

  install( TARGETS landshark_hokuyo_node
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install( DIRECTORY include/${PROJECT_NAME}/
          DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  foreach( resource launch)
    install(DIRECTORY ${resource}
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
      PATTERN "*~" EXCLUDE
      )
  endforeach( )
endif( 1 )
