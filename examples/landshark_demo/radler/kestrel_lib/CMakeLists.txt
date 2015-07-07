cmake_minimum_required(VERSION 2.8)

project(kestrel_lib)
catkin_package(LIBRARIES kestrel_lib)

add_library(kestrel_lib STATIC src/kestrel_comm.c src/kestrel_helpers.c src/kestrel_mbox.c)
target_include_directories(kestrel_lib PUBLIC include PRIVATE src kmods)
