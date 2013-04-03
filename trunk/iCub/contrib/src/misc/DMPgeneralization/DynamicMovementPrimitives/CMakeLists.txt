# Copyright: (C) 2012 Robotics Brain and Cognitive Sciences, Istituto Italiano di Tecnologia
# Authors: Elena Ceseracciu
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(PROJECTNAME DynamicMovementPrimitives)

PROJECT(${PROJECTNAME})
SET(folder_source src/DMPstructure.cpp src/DMPstate.cpp)
SET(folder_header include/iCub/DMP/DMPstructure.h include/iCub/DMP/DMPstate.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include ${YARP_INCLUDE_DIRS})
ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})
icub_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                   DESTINATION include/iCub/DMP 
                                   FILES ${folder_header} VERBOSE) 
#set(DynamicMovementPrimitives_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE STRING "DynamicMovementPrimitives
#     include directory")  

