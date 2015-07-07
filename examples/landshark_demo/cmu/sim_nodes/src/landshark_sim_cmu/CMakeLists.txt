cmake_minimum_required(VERSION 2.8.3)
project(landshark_sim_cmu)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES landshark_sim_cmu 
   CATKIN_DEPENDS 
)

link_directories(${catkin_LIBRARY_DIRS})

install( PROGRAMS 
  scripts/deadman_turn_on.py scripts/map_read_show.py scripts/plot.py scripts/pub_odom_dummy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS 
  scripts/deadman_turn_on.py scripts/map_read_show.py scripts/plot.py scripts/pub_odom_dummy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)

install(DIRECTORY protos
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)

install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)

install(DIRECTORY worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE
)
