cmake_minimum_required(VERSION 2.8.3)
project(landshark_2dnav)

find_package(catkin REQUIRED COMPONENTS
  base_local_planner cmake_modules costmap_2d dynamic_reconfigure move_base 
  move_base_msgs actionlib actionlib_msgs roscpp nav_core tf nav_msgs pluginlib 
  message_generation
)

add_action_files(
  DIRECTORY action
  FILES LocalPP.action
)

generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs nav_msgs
)

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES landshark_2dnav 
   CATKIN_DEPENDS 
   base_local_planner cmake_modules costmap_2d  dynamic_reconfigure move_base 
   move_base_msgs actionlib actionlib_msgs roscpp nav_core tf nav_msgs pluginlib message_runtime
)




include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

install( PROGRAMS 
  scripts/control.py scripts/local_pp.py scripts/map_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS 
  scripts/control.py scripts/local_pp.py scripts/map_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

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

