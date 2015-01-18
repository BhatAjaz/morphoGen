# Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vishwanathan Mohan
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# Includes Whole body coordination + Trajectory synthesis for tool use # obstacle avoidance, new itnerface to Observer and the Body

SET(KEYWORD "ObserverIndustrial")
PROJECT(${KEYWORD})
cmake_minimum_required(VERSION 2.6)

FIND_PACKAGE(YARP REQUIRED)
LINK_LIBRARIES(${YARP_LIBRARIES})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})


set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)

# Setup info to include the KinematicsThread into the PMP Project
find_path(MODULES_KCL  src/Common/PortMap.h HINTS "${DARWIN_COMMON}/../modules/kcl")

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS} 	
	${MODULES_KCL}/src/Common
    ${MODULES_KCL}/src/ObserverStub/include
    ${ICUB_INCLUDE_DIRS}	
    ${ICUB_CONTRIB_DIRS}
)

# Search for source code.
SET(folder_effector_source
   "${MODULES_KCL}/src/ObserverStub/src/ConfigMessages.xml"
   "${MODULES_KCL}/src/ObserverStub/src/EffectorThread.cpp"
   "${MODULES_KCL}/src/ObserverStub/src/AttentionAccessImpl.cpp"
   "${MODULES_KCL}/src/ObserverStub/src/AffordanceAccessImpl.cpp"
   "${MODULES_KCL}/src/ObserverStub/src/WorkspaceCalculationsImpl.cpp")
   
SET(folder_effector_header
   "${MODULES_KCL}/src/ObserverStub/include/PrintFunctions.h"
   "${MODULES_KCL}/src/ObserverStub/include/Vocabs.h"
   "${MODULES_KCL}/src/ObserverStub/include/ConfigMessages.h"
   "${MODULES_KCL}/src/ObserverStub/include/EffectorAccess.h"
   "${MODULES_KCL}/src/ObserverStub/include/EffectorThread.h"
   "${MODULES_KCL}/src/ObserverStub/include/AttentionAccess.h"
   "${MODULES_KCL}/src/ObserverStub/include/AttentionAccessImpl.h"
   "${MODULES_KCL}/src/ObserverStub/include/AffordanceAccess.h"
   "${MODULES_KCL}/src/ObserverStub/include/AffordanceAccessImpl.h"
   "${MODULES_KCL}/src/ObserverStub/include/WorkspaceCalculations.h"
   "${MODULES_KCL}/src/ObserverStub/include/WorkspaceCalculationsImpl.h")

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("KCL Header Files" FILES ${folder_effector_header})
SOURCE_GROUP("KCL Source Files" FILES ${folder_effector_source})

# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
		${folder_effector_source}
		${folder_effector_header}
    )

    TARGET_LINK_LIBRARIES(${KEYWORD}        

      ${YARP_LIBRARIES}
      )	

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
