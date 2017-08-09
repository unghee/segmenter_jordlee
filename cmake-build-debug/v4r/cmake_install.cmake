# Install script for directory: /home/fetch/catkin_ws/src/segmenter_jordlee/v4r

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

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/GraphCut/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/KinectInterface/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/matas/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/PCLAddOns/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/SurfaceClustering/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/SurfaceModeling/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/SurfaceRelations/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/SurfaceSegmenter/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/SurfaceUtils/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/svm/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/TomGine/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/TomGinePCL/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/ObjectModeling/cmake_install.cmake")
  INCLUDE("/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/v4r/vs3/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

