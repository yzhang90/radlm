cmake_minimum_required(VERSION 2.8.3)
project(cmu_z_test_monitor)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation landshark_msgs msg_auth)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY msg
  FILES
  AnomalyMsg.msg
 )

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime landshark_msgs msg_auth
  DEPENDS ${PROJECT_NAME}_generate_messages
)

add_executable(${PROJECT_NAME}_node 
  src/multi_window_anomaly_monitor.cpp 
  src/multi_window_motion_gps_odo_anomaly_monitor.cpp 
  src/multi_window_landshark_execution_monitor_main)

add_dependencies( ${PROJECT_NAME}_node
  ${PROJECT_NAME}_generate_messages 
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_genpy
  )

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

add_executable(cmu_monitor_proxy
  src/monitor_proxy.cpp)

add_dependencies( cmu_monitor_proxy
  ${PROJECT_NAME}_generate_messages 
  ${PROJECT_NAME}_gencpp
  ${PROJECT_NAME}_genpy
  )

target_link_libraries(cmu_monitor_proxy ${catkin_LIBRARIES})

install( TARGETS 
  ${PROJECT_NAME}_node
  cmu_monitor_proxy
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

foreach( resource launch )
  install(DIRECTORY ${resource}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE
    )
endforeach()
