SET(V4R_SVM 1 CACHE BOOL "Support Vector Machine - based on libsvm." )
IF(V4R_SVM)

PROJECT(v4rsvm)

#include_directories(.)
find_package(OpenCV REQUIRED)
message(STATUS "Statistics_OpenCV_LIBS: ${OpenCV_LIBS}")
message(STATUS "Statistics_OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
message(STATUS "Statistics_OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

SET(SOURCE_CPP
#  Statistics.cpp
  svm.cpp
  SVMFileCreator.cpp
#  SVMPredictor.cpp
  SVMPredictorSingle.cpp
)

SET(SOURCE_H
#  Statistics.h
  svm.h
  SVMFileCreator.h
#  SVMPredictor.h
  SVMPredictorSingle.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_SVM)

