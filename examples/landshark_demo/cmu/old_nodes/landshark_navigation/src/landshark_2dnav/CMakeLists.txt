cmake_minimum_required(VERSION 2.8.3)
project(landshark_2dnav)

find_package(catkin REQUIRED COMPONENTS
#  landshark_msgs
  base_local_planner 
  cmake_modules
  costmap_2d
  dynamic_reconfigure
#  landshark_sim
  move_base
  move_base_msgs 
  actionlib 
  roscpp
  nav_core 
  tf 
  nav_msgs 
  pluginlib
  message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  LandsharkNav.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   LandsharkNavWaypoint.srv
 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES landshark_2dnav 
   CATKIN_DEPENDS 
#   landshark_msgs 
   base_local_planner cmake_modules costmap_2d  dynamic_reconfigure 
#   landshark_sim 
   move_base move_base_msgs actionlib roscpp nav_core tf nav_msgs pluginlib message_runtime
)




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

install( PROGRAMS 
  scripts/navigate.py scripts/cmdvel_to_basevelocity.py scripts/deadman_turn_on.py scripts/pub_odom_dummy.py scripts/vanilla_way_point_navigation.py scripts/vanilla_way_point_navigation_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

add_executable(navigation_goal src/navigation_goal.cpp)
target_link_libraries(navigation_goal ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS scripts/navigate.py scripts/cmdvel_to_basevelocity.py scripts/deadman_turn_on.py scripts/pub_odom_dummy.py scripts/vanilla_way_point_navigation.py scripts/vanilla_way_point_navigation_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)

install(DIRECTORY map
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)

