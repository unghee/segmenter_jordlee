SET(V4R_MATAS 1 CACHE BOOL "Matas Canny Edge Detector." )
IF(V4R_MATAS)

PROJECT(v4rmatas)

#include_directories(.)
find_package(OpenCV REQUIRED)
#message(STATUS "MATAS_OpenCV_LIBS: ${OpenCV_LIBS}")
#message(STATUS "MATAS_OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")
#message(STATUS "MATAS_OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

SET(SOURCE_CPP
	arg2Str.c
	ary.c
	buffStr.c
	canny_body.c
	colorquant.c
	consP.c
	consStr.c
	ConstSet.c
	consVectsP.c
	convertAry.c
	copyAry.c
	destSets.c
	dupStr.c
	entityP.c
	errorHand.c
	extendAry.c
	formatP.c
	getSetP.c
	gradient.c
	graph.c
	initAry.c
	LL.c
	LLconsis.c
	LLfile.c
	LLio.c
	LLmergeSort.c
	LLstr.c
	makary.c
	normFary.c
	option.c
	optionDouble.c
	optionInt.c
	optionList.c
	optionMisc.c
	PGerror.c
	PGio.c
	PGutil.c
	quantAry.c
	readAry.c
	RWSets.c
	scaleFary.c
	StdGeomDefP.c
	strcasecmp.c
	suffStr.c
	thrLink.c
	tokenPref.c
	tokenStr.c
	vector2.c
	vector2i.c
	writeAry.c
	writeGF.c
	xyEdge2LL.c
)

SET(SOURCE_H
	ary.h
	aryio.h
	canny.h
	cannyGradGlob.h
	colorquant.h
	gfLL.h
	gfLLconv.h
	gfLLio.h
	gfPars.h
	graph.h
	linkLL.h
	LL.h
	option.h
	optionPriv.h
	strGM.h
	vector2.h
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_H} ${SOURCE_CPP})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
v4r_add_library(${PROJECT_NAME} "${SOURCE_H}")

ENDIF(V4R_MATAS)


