cmake_minimum_required(VERSION 2.8.3)
project(upenn_landshark_cruise)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs landshark_msgs)

catkin_package(CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs landshark_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/landshark_cruise.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

