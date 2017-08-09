SET(EXT_TTL 1 CACHE BOOL "Triangulation Template Library (http://www.sintef.no/Projectweb/Geometry-Toolkits/)" )
IF(EXT_TTL)

PROJECT(ttl)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

FIND_PACKAGE(Boost REQUIRED)

# Include directories

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})


# Make the ttl library

FILE(GLOB_RECURSE ttl_SRC ./*.C ./*.cpp)
FILE(GLOB_RECURSE ttl_INC ./*.h)

ADD_LIBRARY(ttl SHARED ${ttl_SRC})
INSTALL(DIRECTORY DESTINATION include/v4rexternal/TTL-1.1.0)
INSTALL(FILES ${ttl_INC} DESTINATION include/v4rexternal/TTL-1.1.0)
INSTALL(TARGETS ttl LIBRARY DESTINATION lib)

ENDIF(EXT_TTL)
