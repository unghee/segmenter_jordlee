SET(V4R_VS3 1 CACHE BOOL "Vision System 3 - Perceptual Grouping in 2D" )
IF(V4R_VS3)

PROJECT(v4rvs3)

#include_directories(.)
find_package(OpenCV REQUIRED)
#message(STATUS "VS3_OpenCV_LIBS: ${OpenCV_LIBS}")
#message(STATUS "VS3_OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
#message(STATUS "VS3_OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

SET(SOURCE_CPP 
	betacf.c
	betai.c
	gammln.c
	nrutil.c
	AJunction.cc
	Arc.cc
	CEdge.cc
	Circle.cc
	Closure.cc
	Collinearity.cc
	Cone.cc
	ConvexArcGroup.cc
	Corner.cc
	Cube.cc
	Cylinder.cc
	EJunction.cc
	Ellipse.cc
	FlapAri.cc
	Flap.cc
	FormArcJunctions.cc
	FormArcs.cc
	FormCircles.cc
	FormClosures.cc
	FormCones.cc
	FormConvexArcGroups.cc
	FormCorners.cc
	FormCubes.cc
	FormEllipses.cc
	FormEJunctions.cc
	FormCylinders.cc
	FormFlapsAri.cc
	FormFlaps.cc
	FormJunctions.cc
	FormLines.cc
	FormRectangles.cc
	FormSegments.cc
	Gestalt.cc
	GestaltPrinciple.cc
 	IdImage.cc
	Line.cc
	LJunction.cc
	Rectangle.cc
	rosin_arcline.cc
	rosin_lines.cc
	Segment.cc
	TJunction.cc
	VisionCore.cc
	VoteImage.cc
# vs3 utils:
	CDataFile.cpp
	Color.cc
	ColorHistogram.cpp
	Config.cc
	ConfigFile.cc
	Draw.cc
	Histogram.cpp
)

SET(SOURCE_H
	AJunction.hh
	Arc.hh
	CEdge.hh
	Circle.hh
	Closure.hh
	Collinearity.hh
	Cone.hh
	ConvexArcGroup.hh
	Corner.hh
	Cube.hh
	Cylinder.hh
	EJunction.hh
	Ellipse.hh
	FlapAri.hh
	Flap.hh
	FormArcJunctions.hh
	FormArcs.hh
	FormCircles.hh
	FormClosures.hh
	FormCones.hh
	FormConvexArcGroups.hh
	FormCorners.hh
	FormCubes.hh
	FormEllipses.hh
	FormEJunctions.hh
	FormCylinders.hh
	FormFlapsAri.hh
	FormFlaps.hh
	FormJunctions.hh
	FormLines.hh
	FormRectangles.hh
	FormSegments.hh
	Gestalt.hh
	GestaltPrinciple.hh
 	IdImage.hh
	Line.hh
	LJunction.hh
	Rectangle.hh
	rosin_arcline.hh
	rosin_lines.hh
	Segment.hh
	TJunction.hh
	VisionCore.hh
	VoteImage.hh
# vs3 utils:
	CDataFile.h
	Color.hh
	ColorHistogram.h
	Config.hh
	ConfigFile.hh
	Draw.hh
	Histogram.h
  Math.hh
  Math.ic
  MathUtils.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} v4rmatas)
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

#INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

ENDIF(V4R_VS3)
















