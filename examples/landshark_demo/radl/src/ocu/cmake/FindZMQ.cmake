# A package finder for ZMQ
# 
set( name ZMQ )
set( ${name}_FOUND False )

if ( ${CMAKE_SYSTEM_NAME} STREQUAL Linux )
  find_path( ${name}_INCLUDE_DIRS zmq.hpp
    HINTS /usr/include /usr/local/include )

  find_library( ${name}_LIBRARIES
    NAMES zmq
    PATHS /usr/lib /usr/local/lib )

  if ( ${name}_INCLUDE_DIRS )
    if ( ${name}_LIBRARIES )
      mark_as_advanced( ${name}_INCLUDE_DIRS )
      mark_as_advanced( ${name}_LIBRARIES )
      set( ${name}_FOUND True )
    endif()
  endif()
elseif( ${CMAKE_SYSTEM_NAME} STREQUAL Darwin )
  message( STATUS "FindZMQ does not support Darwin... yet." )
endif()


if ( ${name}_FOUND )
  message( STATUS "Found ${name}: (${${name}_INCLUDE_DIR}, ${${name}_LIBRARIES})" )
else()
  message( STATUS "Unable to find ${name}" )
  message( STATUS "  Please install the zmq package: " )
  message( STATUS "  sudo apt-get install libzmqpp-dev" )
  if ( ${name}_FIND_REQUIRED )
    message( FATAL_ERROR "Failed to find ${name}" )
  endif()
endif()

