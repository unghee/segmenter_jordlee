set(OPENNURBS_LIBRARY opennurbs)
set(OPENNURBS_LIBRARIES ${OPENNURBS_LIBRARY})
#set(OPENNURBS_INCLUDE_DIRS "${V4R_SOURCE_DIR}/3rdparty")
#set(OPENNURBS_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty")
set(OPENNURBS_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}")
set(OPENNURBS_VERSION "1.0") # TODO
#set(3P_LIBRARY_OUTPUT_PATH      "${V4R_BINARY_DIR}/3rdparty/lib${LIB_SUFFIX}")
set(3P_LIBRARY_OUTPUT_PATH      "${CMAKE_BINARY_DIR}/lib${LIB_SUFFIX}")

set(ON_NURBS_LIBRARY on_nurbs)
Set(ON_NURBS_LIBRARIES ${ON_NURBS_LIBRARY})
#set(ON_NURBS_INCLUDE_DIRS "${V4R_SOURCE_DIR}/3rdparty")
#set(ON_NURBS_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/3rdparty")
set(ON_NURBS_INCLUDE_DIRS "${CMAKE_SOURCE_DIR}")
set(ON_NURBS_VERSION "1.0") # TODO




add_subdirectory(GraphCut)
add_subdirectory(KinectInterface)
add_subdirectory(matas)
add_subdirectory(PCLAddOns)
add_subdirectory(SurfaceClustering)
add_subdirectory(SurfaceModeling)
add_subdirectory(SurfaceRelations)
add_subdirectory(SurfaceSegmenter)
add_subdirectory(SurfaceUtils)
add_subdirectory(svm)
add_subdirectory(TomGine)
add_subdirectory(TomGinePCL)
add_subdirectory(ObjectModeling)
add_subdirectory(vs3)
add_subdirectory(on_nurbs)
add_subdirectory(opennurbs)
