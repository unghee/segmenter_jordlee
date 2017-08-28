SET(V4R_OPENNURBS 1 CACHE BOOL "Interface based on Onnurbs." )
IF(V4R_OPENNURBS)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

PROJECT(v4rOpennurbs)
include("openNURBS.cmake")
add_library(${PROJECT_NAME} SHARED ${OPENNURBS_INCLUDES} ${OPENNURBS_SOURCES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
v4r_add_library(${PROJECT_NAME} "${OPENNURBS_INCLUDES}")

ENDIF(V4R_OPENNURBS)


