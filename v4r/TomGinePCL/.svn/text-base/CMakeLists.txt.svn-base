cmake_minimum_required(VERSION 2.8)

set(V4R_TOMGINE_PCL 1 CACHE BOOL "TomGine Wrapper for PCL types" )
IF(V4R_TOMGINE_PCL)
project(v4rTomGinePCL)

find_package(PCL 1.6 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


set(SOURCE_CPP 
	tgPCL2TomGine.cpp
	tgTomGineThreadPCL.cpp
	)

set(SOURCE_H
	tgPCL2TomGine.h
	tgTomGineThreadPCL.h
	)

add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} v4rTomGine ${PCL_LIBRARIES})

message(STATUS "TomGine: Building within v4r.")
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")
install(DIRECTORY DESTINATION include/v4r/${TOMGINE_INCLUDE_NAME}/shader)
install(FILES ${SOURCE_SHADER} DESTINATION include/v4r/${TOMGINE_INCLUDE_NAME}/shader)

ENDIF(V4R_TOMGINE_PCL)
