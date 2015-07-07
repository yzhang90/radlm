cmake_minimum_required(VERSION 2.8.3)
project(cmu_keyboard_teleop)

find_package( catkin REQUIRED COMPONENTS geometry_msgs roscpp rospy std_msgs
  landshark_msgs )
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  #   INCLUDE_DIRS include
  #  LIBRARIES keyboard_teleop
  #  CATKIN_DEPENDS geometry_msgs landshark_msgs roscpp rospy sensor_msgs std_msgs visualization_msgs
  #  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmu_keyboard_teleop src/keyboard_teleop.cpp)
add_dependencies(cmu_keyboard_teleop landshark_msgs_generate_messages)

target_link_libraries(cmu_keyboard_teleop
  ${catkin_LIBRARIES}
  )

install(TARGETS 
  cmu_keyboard_teleop
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach( resource launch )
  install(DIRECTORY ${resource}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE
    )
endforeach()

