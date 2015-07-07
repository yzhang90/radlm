cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_AUTOMOC ON)

project(ocu_gui_client)
set( OCU ${CMAKE_CURRENT_SOURCE_DIR}/../src/ocu )
list( APPEND CMAKE_MODULE_PATH ${OCU}/cmake)
find_package( Boost REQUIRED COMPONENTS system thread )
find_package( ZMQ )
find_package( Protobuf )
find_package( Qt4 REQUIRED QtCore QtGui )
include(${QT_USE_FILE})
message( STATUS "Qt4_LIBRARIES: ${QT_LIBRARIES}" )

include_directories( ${OCU}/src
  ${Boost_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${Qt4_INCLUDE_DIRS}
  )

add_executable( ocu_gui_client 
  ${OCU}/src/OcuGuiWidget.cpp 
  ${OCU}/src/ocu_client.cpp 
  ${OCU}/src/read_map.cpp 
  ${OCU}/src/ocu.pb.cc )

target_link_libraries( ocu_gui_client 
  ${Boost_LIBRARIES}
  ${ZMQ_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${QT_LIBRARIES}
  )

add_executable( test_read_map 
  ${OCU}/src/test_read_map.cpp 
  ${OCU}/src/read_map.cpp 
  )

install( TARGETS
  ocu_gui_client
  RUNTIME DESTINATION bin
  )
