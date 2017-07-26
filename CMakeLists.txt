cmake_minimum_required(VERSION 2.8.3)
project(segmenter_jordlee)

## find catkin and required packages
include(opencv.cmake)
find_package(
  catkin REQUIRED 
  COMPONENTS 
  eigen_conversions 
  geometry_msgs 
  message_generation   
  roscpp 
  pcl_ros
  pcl_conversions
  std_msgs 
  tf
  tf_conversions
  visualization_msgs
  cmake_modules
)

# find opencfind_package(OPENCV REQUIRED)
find_package(OpenCV REQUIRED)
## find Lapack
find_package(LAPACK REQUIRED)

find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

#find pcl library
find_package(PCL 1.7 REQUIRED )

## compiler optimization flags
#set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp -Wno-deprecated")

## add messages
#add_message_files(
#  FILES
#  CylinderMsg.msg
#  CylinderArrayMsg.msg
#  HandleListMsg.msg
#)
#
#generate_messages(DEPENDENCIES geometry_msgs)
#

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS 
    eigen_conversions 
    geometry_msgs 
    message_runtime 
    roscpp 
    pcl_ros
    pcl_conversions
    std_msgs 
    tf
    tf_conversions
    visualization_msgs
  DEPENDS
    Eigen
    LAPACK
    )


include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS})
# include project libraries
include_directories( ${PROJECT_SOURCE_DIR} v4r include)

## create executables
add_executable(${PROJECT_NAME} src/segmenter_jordlee.cpp)

## link libraries to localization executable
 target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OPENCV_LIBRARIES} v4rKinectInterface v4rSurfaceModeling v4rSurfaceClustering v4rObjectModeling v4rSurfaceRelations v4rsvm v4rGraphCut v4rSurfaceUtils v4rvs3 v4rPCLAddOns v4rTomGine)


