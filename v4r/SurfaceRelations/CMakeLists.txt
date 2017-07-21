SET(V4R_SURFACE_RELATIONS 1 CACHE BOOL "Relation estimation between parametric surface patches." )
IF(V4R_SURFACE_RELATIONS)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)

PROJECT(v4rSurfaceRelations)

SET(SOURCE_CPP
  CEdgeExtractor.cpp
  ColorHistogram.cpp
  ColorHistogram2D.cpp
  ColorHistogram3D.cpp
  ContourNormalsDistance.cc
  cvgabor.cpp
  Fourier.cpp
  Gabor.cpp
  MaskDilationErosion.cpp
  PatchRelations.cpp
  Surf.cpp
)

SET(SOURCE_H
  CEdgeExtractor.h
  ColorHistogram.h
  ColorHistogram2D.h
  ColorHistogram3D.h
  ContourNormalsDistance.hh
  cvgabor.h
  Fourier.h
  Gabor.h
  MaskDilationErosion.h
  PatchRelations.h
  Relation.h
  Surf.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rSurfaceModeling v4rPCLAddOns v4rvs3) 
target_link_libraries(${PROJECT_NAME}  v4rTomGine)# v4rObjectModeling #v4rPCore)
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_SURFACE_RELATIONS)

