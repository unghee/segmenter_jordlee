SET(V4R_SURFACE_CLUSTERING 1 CACHE BOOL "Normals-clustering of point cloud." )
IF(V4R_SURFACE_CLUSTERING)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)

PROJECT(v4rSurfaceClustering)
SET(SOURCE_CPP
  ClusterNormalsToPlanes.cc
  ZAdaptiveNormals.cc
)

SET(SOURCE_H
  ClusterNormalsToPlanes.hh
  PPlane.h
  ZAdaptiveNormals.hh
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES})
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_SURFACE_CLUSTERING)

