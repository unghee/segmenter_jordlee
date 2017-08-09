# Install script for directory: /home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine

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
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/TomGine" TYPE DIRECTORY FILES "")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/TomGine" TYPE FILE FILES
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/GLEvent.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/GLInput.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/GLWindow.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/headers.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/ply.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/PlyStructure.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgCamera.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgCollission.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgEngine.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgError.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgErrorMetric.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgFont.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgFrameBufferObject.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgFrustum.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgHistDesc2D.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgHistogram.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgImageProcessor.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgLight.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgMaterial.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgMathlib.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgModel.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgModelLoader.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgNurbsCurve.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgNurbsSurface.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgNurbsVolume.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgPlot2D.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgPose.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgQuaternion.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgRenderModel.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgShader.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgShapeCreator.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgSingleton.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgTexture.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgTextureModel.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgTimer.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgTomGine.h"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/tgTomGineThread.h"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rTomGine.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rTomGine.so")
    FILE(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rTomGine.so"
         RPATH "/usr/local/lib")
  ENDIF()
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/fetch/catkin_ws/src/segmenter_jordlee/cmake-build-debug/devel/lib/libv4rTomGine.so")
  IF(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rTomGine.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rTomGine.so")
    FILE(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rTomGine.so"
         OLD_RPATH "/usr/local/lib:"
         NEW_RPATH "/usr/local/lib")
    IF(CMAKE_INSTALL_DO_STRIP)
      EXECUTE_PROCESS(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libv4rTomGine.so")
    ENDIF(CMAKE_INSTALL_DO_STRIP)
  ENDIF()
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/TomGine/shader" TYPE DIRECTORY FILES "")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/v4r/TomGine/shader" TYPE FILE FILES
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/color.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/coxdeboor.c"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/edgetest.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/edgetest.vert"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipAdd.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipDilate.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipGauss.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipInvert.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipMultiply.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipParam2Polar.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipRGB2HSV.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipSobel.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipSobelNormal.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipSpreading.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipThinning.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/ipThreshold.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/mmConfidence.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/mmConfidence.vert"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/nurbscurve.vert"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/nurbssurface.vert"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/nurbsvolume.vert"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/texColorTest.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/texColorTest.vert"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/texDepthTest.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/texDepthTest.vert"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/texEdgeTest.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/texEdgeTest.vert"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/texturing.frag"
    "/home/fetch/catkin_ws/src/segmenter_jordlee/v4r/TomGine/shader/texturing.vert"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

