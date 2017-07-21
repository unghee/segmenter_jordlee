SET(V4R_OBJECT_MODELING 1 CACHE BOOL "Object refinement, closing of gaps, texturing, global optimization." )
IF(V4R_OBJECT_MODELING)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")

PROJECT(v4rObjectModeling)
SET(SOURCE_CPP
	ContourRefinement.cpp
	CreateMeshModel.cc
	SurfaceBoundary.cpp
	Texturing.cpp
	Triangulation.cpp
)

SET(SOURCE_H
	ContourRefinement.h
	CreateMeshModel.hh
	SurfaceBoundary.h
	Texturing.h
	Triangulation.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ttl)
target_link_libraries(${PROJECT_NAME} v4rTomGine v4rPCLAddOns)
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_OBJECT_MODELING)

