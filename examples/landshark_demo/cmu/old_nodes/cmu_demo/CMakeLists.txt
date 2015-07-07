cmake_minimum_required(VERSION 2.8.3)
project(cmu_demo)

find_package(
 catkin REQUIRED COMPONENTS 
 )

include_directories(include
 ${catkin_INCLUDE_DIRS}
)

