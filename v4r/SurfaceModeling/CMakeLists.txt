SET(V4R_SURFACE_MODELING 1 CACHE BOOL "Model selection and parametric surface estimation (planes and NURBS)." )
IF(V4R_SURFACE_MODELING)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(${V4R_DIR}/external/opennurbs)
include_directories(${V4R_DIR}/external/opennurbs)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
SET(CMAKE_BUILD_TYPE Release)

PROJECT(v4rSurfaceModeling)
SET(SOURCE_CPP
  SurfaceModeling.cc
)

SET(SOURCE_H
  SurfaceModeling.hh
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rSurfaceClustering v4rPCLAddOns v4rObjectModeling) # v4rTomGine v4rPCore 
if(DEBUG)
  target_link_libraries(${PROJECT_NAME} ${OPENCV_LIBRARIES} ${PCL_LIBRARIES} v4rTomGine)
endif(DEBUG)

v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_SURFACE_MODELING)

