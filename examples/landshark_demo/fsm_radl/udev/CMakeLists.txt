cmake_minimum_required(VERSION 2.8.3)
project(landshark_hokuyo)
find_package(Boost REQUIRED COMPONENTS system thread)

set( HOKUYO_D ../src/landshark_hokuyo/src/driver )
set( HOKUYO ../src/landshark_hokuyo/src )
include_directories( ${Boost_INCLUDE_DIRS} ${HOKUYO_D}/include ${IMU})

add_executable( get_hokuyo_id ${HOKUYO_D}/src/hokuyo.cpp ${HOKUYO}/get_hokuyo_id.cpp)
target_link_libraries( get_hokuyo_id 
  ${Boost_LIBRARIES}
  )
 
set( IMU ../src/landshark_imu/src )
add_executable( get_imu_id ${IMU}/3dmgx2.cpp ${IMU}/get_imu_id.cpp)
target_link_libraries( get_imu_id 
  ${Boost_LIBRARIES}
  )
 
install( TARGETS
  get_hokuyo_id
  get_imu_id
  RUNTIME DESTINATION bin
  )
