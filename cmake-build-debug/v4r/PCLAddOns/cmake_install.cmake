# Install script for directory: /home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns

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
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/PCLAddOns" TYPE DIRECTORY FILES "")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/PCLAddOns" TYPE FILE FILES
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/BilateralFilter.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/CCEuclideanClustering.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/CCLabeling.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/ContourDetection.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/EuclideanClustering.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/ModelFitter.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/NormalsClustering.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/NormalsEstimationNR.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/PCLFunctions.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/PCLUtils.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/PCLCommonHeaders.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/PlanePopout.hh"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/PCLAddOns/SubsamplePointCloud.hh"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rPCLAddOns.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rPCLAddOns.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rPCLAddOns.so"
         RPATH "/usr/local/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/devel/lib/libv4rPCLAddOns.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rPCLAddOns.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rPCLAddOns.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rPCLAddOns.so"
         OLD_RPATH "/usr/local/lib:"
         NEW_RPATH "/usr/local/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rPCLAddOns.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

