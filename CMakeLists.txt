cmake_minimum_required(VERSION 2.8.3)
project(segmenter_jordlee)

## find catkin and required packages
include(cmake/opencv.cmake)
include(cmake/v4rutils.cmake) # V4R utils

SET(LIBRARY_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/lib")
SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/bin")
# don't put the above two lines.. it causes cannot find surfaceutils error 


#SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(V4R_BUILD 1)

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
  rail_manipulation_msgs
#  rospy
#  genmsg
#  sensor_msgs
)
# find opencfind_package(OPENCV REQUIRED)
find_package(OpenCV REQUIRED)
## find Lapack
find_package(LAPACK REQUIRED)

find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

#find pcl library
find_package(PCL 1.7 REQUIRED )

add_service_files(
 FILES
 SegmentObject.srv

)

generate_messages(
DEPENDENCIES
std_msgs
rail_manipulation_msgs
)
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
include_directories( ${PROJECT_SOURCE_DIR} v4r v4rexternal include)
#include_directories( include) 

link_directories(${PCL_LIBRARY_DIRS})

#include_directories(.)
add_subdirectory(v4r)
add_subdirectory(v4rexternal)
                 
#install target
configure_file(v4r.pc.in "${PROJECT_BINARY_DIR}/v4r.pc" @ONLY)
INSTALL(FILES ${PROJECT_BINARY_DIR}/v4r.pc DESTINATION lib/pkgconfig)



# uninstall target
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)


## create executables
add_executable(${PROJECT_NAME} src/segmenter_jordlee.cpp )
#add_executable(${PROJECT_NAME} src/segmenter_servicetest.cpp)
add_executable(svm_model_creator src/svm_model_creator.cpp)

target_link_libraries ( ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OPENCV_LIBRARIES} v4rKinectInterface v4rSurfaceModeling v4rSurfaceClustering v4rObjectModeling v4rSurfaceRelations v4rsvm v4rGraphCut v4rTomGine v4rSurfaceUtils v4rOnNurbs v4rOpennurbs)

target_link_libraries (svm_model_creator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OPENCV_LIBRARIES} v4rKinectInterface v4rSurfaceModeling v4rSurfaceClustering v4rObjectModeling v4rSurfaceRelations v4rsvm v4rGraphCut v4rSurfaceUtils v4rOnNurbs v4rOpennurbs)









