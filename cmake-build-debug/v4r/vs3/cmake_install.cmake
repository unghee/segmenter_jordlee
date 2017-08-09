# Install script for directory: /home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/usr/local")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Debug")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/vs3" TYPE DIRECTORY FILES "")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/vs3" TYPE FILE FILES
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/AJunction.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Arc.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/CEdge.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Circle.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Closure.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Collinearity.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Cone.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/ConvexArcGroup.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Corner.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Cube.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Cylinder.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/EJunction.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Ellipse.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FlapAri.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Flap.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormArcJunctions.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormArcs.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormCircles.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormClosures.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormCones.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormConvexArcGroups.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormCorners.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormCubes.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormEllipses.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormEJunctions.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormCylinders.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormFlapsAri.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormFlaps.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormJunctions.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormLines.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormRectangles.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/FormSegments.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Gestalt.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/GestaltPrinciple.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/IdImage.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Line.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/LJunction.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Rectangle.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/rosin_arcline.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/rosin_lines.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Segment.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/TJunction.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/VisionCore.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/VoteImage.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/CDataFile.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Color.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/ColorHistogram.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Config.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/ConfigFile.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Draw.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Histogram.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Math.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/Math.ic"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/vs3/MathUtils.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rvs3.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rvs3.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rvs3.so"
         RPATH "/usr/local/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/devel/lib/libv4rvs3.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rvs3.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rvs3.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rvs3.so"
         OLD_RPATH "/usr/local/lib:/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/devel/lib:"
         NEW_RPATH "/usr/local/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rvs3.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

