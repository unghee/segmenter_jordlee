SET(V4R_SURFACE_UTILS 1 CACHE BOOL "Utils for surface abstraction." )
IF(V4R_SURFACE_UTILS)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)

PROJECT(v4rSurfaceUtils)
SET(SOURCE_CPP
  FileSystem.cc
  Utils.cc
)

SET(SOURCE_H
  FileSystem.hh
  Relation.h
  SurfaceModel.hpp
  Utils.hh
)

IF(V4R_TOMGINE)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rTomGine)
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ELSE(V4R_TOMGINE)
  MESSAGE(STATUS "********************************************\n   v4r/SurfaceUtils: missing V4R_TOMGINE\n   ********************************************\n")
ENDIF(V4R_TOMGINE)

ENDIF(V4R_SURFACE_UTILS)

