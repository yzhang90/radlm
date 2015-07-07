cmake_minimum_required(VERSION 2.8.3)
project(landshark_description)

find_package(catkin REQUIRED COMPONENTS urdf)
include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS xacro
  CATKIN_DEPENDS urdf 
  )

foreach( resource config launch meshes robot urdf ) 
  install(DIRECTORY ${resource}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    PATTERN "*~" EXCLUDE
  )
endforeach()

