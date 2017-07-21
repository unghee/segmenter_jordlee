cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(segmenter_jordlee)

include_directories(${OPENCV_INCLUDE_DIRS} )
#include pcl
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# include project libraries
include_directories(${PROJECT_SOURCE_DIR} v4r include)

add_executable ( ${PROJECT_NAME} src/segmenter_jordlee.cpp)
target_link_libraries (segmenter_jordlee ${PCL_LIBRARIES} ${OPENCV_LIBRARIES} v4rKinectInterface v4rSurfaceModeling v4rSurfaceClustering v4rObjectModeling v4rSurfaceRelations v4rsvm v4rGraphCut v4rSurfaceUtils v4rvs3)



