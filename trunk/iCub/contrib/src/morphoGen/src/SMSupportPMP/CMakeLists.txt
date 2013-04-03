# Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: 
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

IF(WIN32)
cmake_minimum_required(VERSION 2.6)
ELSE(WIN32)
cmake_minimum_required(VERSION 2.4)
ENDIF(WIN32)

SET(KEYWORD "PMPTest")
PROJECT(${KEYWORD})

FIND_PACKAGE(YARP REQUIRED)
LINK_LIBRARIES(${YARP_LIBRARIES})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}
)

# Search for source code.
FILE(GLOB folder_source *.cpp)
FILE(GLOB folder_header *.h)

LINK_LIBRARIES(

)


# Set up the main executable.

ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )

TARGET_LINK_LIBRARIES(${KEYWORD} logpolar ${OpenCV_LIBS} ${YARP_LIBRARIES} )	

INSTALL_TARGETS(/bin ${KEYWORD})