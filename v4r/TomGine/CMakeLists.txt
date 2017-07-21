cmake_minimum_required(VERSION 2.8)

set(V4R_TOMGINE 1 CACHE BOOL "OpenGL-GPU rendering, geometry and image processing library." )
IF(V4R_TOMGINE)
project(v4rTomGine)
add_definitions(-DLINUX)
add_definitions(-DGL_GLEXT_PROTOTYPES)

# find dependent packages
find_package(PkgConfig REQUIRED)
pkg_search_module(GL REQUIRED gl)
pkg_search_module(GLU REQUIRED glu)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

if(NOT GL_FOUND)
	message(FATAL "libgl not found, required by v4rTomGine")
endif(NOT GL_FOUND)

set(ENABLE_FTGL_FONTS 0 CACHE BOOL "Enable ftgl fonts do draw OpenGL fonts?" )
IF(ENABLE_FTGL_FONTS)
	pkg_search_module(FTGL REQUIRED ftgl)
	if(NOT FTGL_FOUND)
		message(FATAL "libftgl not found, required by v4rTomGine")
	endif(NOT FTGL_FOUND)
  set(TRUETYPE_FONT /usr/share/fonts/truetype/freefont/FreeSans.ttf CACHE PATH "A valid truetype font on your system")
  add_definitions(-DTTF_FONT="${TRUETYPE_FONT}")
  include_directories(${FTGL_INCLUDE_DIRS})
  add_definitions(-DUSE_FTGL_FONT)
ELSE(ENABLE_FTGL_FONTS)
  set(GLX_FONT_NAME 10x20 CACHE PATH "A valid system font (shell command: xlsfonts)")
  add_definitions(-DGLX_FONT="${GLX_FONT_NAME}")
ENDIF(ENABLE_FTGL_FONTS)

# create tomgine include directory name
string(REGEX REPLACE "v4r" "" TOMGINE_INCLUDE_NAME ${PROJECT_NAME})

# define shader path
set(TOMGINE_SHADER ${CMAKE_INSTALL_PREFIX}/include/v4r/${TOMGINE_INCLUDE_NAME}/shader/)
add_definitions(-DTOMGINE_SHADER="${TOMGINE_SHADER}")
message( STATUS "TomGine_SHADER is ${TOMGINE_SHADER}")

set(SOURCE_CPP 
	GLXWindow.cpp
	GLXWindowGetEvent.cpp
	ply.c
	tgCamera.cpp
	tgCollission.cpp
	tgEngine.cpp
	tgError.cpp
	tgErrorMetric.cpp
	tgFont.cpp
	tgFrameBufferObject.cpp
	tgFrustum.cpp
	tgHistDesc2D.cpp
	tgHistogram.cpp
	tgImageProcessor.cpp
	tgLight.cpp
	tgMaterial.cpp
	tgMathlib.cpp
	tgModel.cpp
	tgModelLoader.cpp
	tgNurbsCurve.cpp
	tgNurbsSurface.cpp
	tgNurbsVolume.cpp
	tgPlot2D.cpp
	tgPose.cpp
	tgQuaternion.cpp
	tgRenderModel.cpp
	tgShader.cpp
	tgShapeCreator.cpp
	tgTexture.cpp
	tgTextureModel.cpp
	tgTimer.cpp
	tgTomGineThread.cpp)

set(SOURCE_H
	GLEvent.h
	GLInput.h
	GLWindow.h
	headers.h
	ply.h
	PlyStructure.h
	tgCamera.h
	tgCollission.h
	tgEngine.h
	tgError.h
	tgErrorMetric.h
	tgFont.h
	tgFrameBufferObject.h
	tgFrustum.h
	tgHistDesc2D.h
	tgHistogram.h
	tgImageProcessor.h
	tgLight.h
	tgMaterial.h
	tgMathlib.h
	tgModel.h
	tgModelLoader.h
	tgNurbsCurve.h
	tgNurbsSurface.h
	tgNurbsVolume.h
	tgPlot2D.h
	tgPose.h
	tgQuaternion.h
	tgRenderModel.h
	tgShader.h
	tgShapeCreator.h
	tgSingleton.h
	tgTexture.h
	tgTextureModel.h
	tgTimer.h
	tgTomGine.h
	tgTomGineThread.h
)

set(SOURCE_SHADER
  shader/color.frag
  shader/coxdeboor.c
  shader/edgetest.frag
  shader/edgetest.vert
  shader/ipAdd.frag
  shader/ipDilate.frag
  shader/ipGauss.frag
  shader/ipInvert.frag
  shader/ipMultiply.frag
  shader/ipParam2Polar.frag
  shader/ipRGB2HSV.frag
  shader/ipSobel.frag
  shader/ipSobelNormal.frag
  shader/ipSpreading.frag
  shader/ipThinning.frag
  shader/ipThreshold.frag
  shader/mmConfidence.frag
  shader/mmConfidence.vert
  shader/nurbscurve.vert
  shader/nurbssurface.vert
  shader/nurbsvolume.vert
  shader/texColorTest.frag
  shader/texColorTest.vert
  shader/texDepthTest.frag
  shader/texDepthTest.vert
  shader/texEdgeTest.frag
  shader/texEdgeTest.vert
  shader/texturing.frag
  shader/texturing.vert
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${GL_LIBRARIES} ${GLU_LIBRARIES} pthread X11 rt)
if(ENABLE_FTGL_FONTS)
target_link_libraries(${PROJECT_NAME} ${FTGL_LIBRARIES})
endif(ENABLE_FTGL_FONTS)

if(${V4R_BUILD})

  v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")
  
else(${V4R_BUILD})

  install(DIRECTORY DESTINATION include/v4r/${TOMGINE_INCLUDE_NAME})
  install(FILES ${SOURCE_H} DESTINATION include/v4r/${TOMGINE_INCLUDE_NAME})
  install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
  
  # uninstall target
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
  add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)
  
endif(${V4R_BUILD})

install(DIRECTORY DESTINATION include/v4r/${TOMGINE_INCLUDE_NAME}/shader)
install(FILES ${SOURCE_SHADER} DESTINATION include/v4r/${TOMGINE_INCLUDE_NAME}/shader)

ENDIF(V4R_TOMGINE)
