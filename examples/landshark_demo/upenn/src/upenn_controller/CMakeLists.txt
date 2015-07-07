cmake_minimum_required(VERSION 2.8.3)
project(upenn_controller)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs landshark_msgs visualization_msgs trajectory_msgs actionlib tf )#kdl)

catkin_package( INCLUDE_DIRS include
                CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs landshark_msgs visualization_msgs trajectory_msgs actionlib tf )#kdl )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/controller.cpp src/PID_d_data.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
