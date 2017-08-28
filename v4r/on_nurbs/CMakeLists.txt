SET(V4R_ON_NURBS 1 CACHE BOOL "Interface based on Onnurbs." )
IF(V4R_ON_NURBS)
cmake_minimum_required (VERSION 2.6)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


PROJECT(v4rOnNurbs)

include("on_nurbs.cmake")
add_library(${PROJECT_NAME} SHARED ${ON_NURBS_INCLUDES} ${ON_NURBS_SOURCES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
v4r_add_library(${PROJECT_NAME} "${ON_NURBS_INCLUDES}")

ENDIF(V4R_ON_NURBS)



