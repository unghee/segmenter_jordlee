# Install script for directory: /home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4rexternal/TTL-1.1.0" TYPE DIRECTORY FILES "")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4rexternal/TTL-1.1.0" TYPE FILE FILES
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./api.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./ttl_constr.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./Handle.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./HeTriang.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./HandleId.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./HeTraits.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./HeDart.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./ttl_util.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./ttl.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./mainpage_ttl.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./main_he_ref.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4rexternal/TTL-1.1.0/./mainpage_hed.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libttl.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libttl.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libttl.so"
         RPATH "/usr/local/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/devel/lib/libttl.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libttl.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libttl.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libttl.so"
         OLD_RPATH "::::::::::::::"
         NEW_RPATH "/usr/local/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libttl.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

