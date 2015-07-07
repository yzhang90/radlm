# Wrapper for FindQt4
#
find_package( Qt4 REQUIRED QtCore QtGui )
include(${QT_USE_FILE})
message( STATUS "Qt4_LIBRARIES: ${QT_LIBRARIES}" )
include_directories( ${Qt4_INCLUDE_DIRS} )

  
