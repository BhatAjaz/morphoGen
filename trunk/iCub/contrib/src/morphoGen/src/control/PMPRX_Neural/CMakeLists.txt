# Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Vishwanathan Mohan
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(KEYWORD "PMPRX_Neural")
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
    ${MODULES_KCL}/src/Kinematics/include
    ${ICUB_INCLUDE_DIRS}	
    ${ICUB_CONTRIB_DIRS}
)

# Search for source code.
SET(folder_kinematics_source
   "${MODULES_KCL}/src/Kinematics/src/ConfigMessages.xml"
   "${MODULES_KCL}/src/Kinematics/src/KinematicsController.cpp"
   "${MODULES_KCL}/src/Kinematics/src/KinematicsThread.cpp"
   "${MODULES_KCL}/src/Kinematics/src/KinematicsFactory.cpp")
   
SET(folder_kinematics_header
   "${MODULES_KCL}/src/Kinematics/include/PrintFunctions.h"
   "${MODULES_KCL}/src/Kinematics/include/Vocabs.h"
   "${MODULES_KCL}/src/Kinematics/include/ConfigMessages.h"
   "${MODULES_KCL}/src/Kinematics/include/vecmath3D.h"
   "${MODULES_KCL}/src/Kinematics/include/KinematicsBase.h"
   "${MODULES_KCL}/src/Kinematics/include/KinematicsRX130.h"
   "${MODULES_KCL}/src/Kinematics/include/KinematicsTX90.h"
   "${MODULES_KCL}/src/Kinematics/include/KinematicsController.h"
   "${MODULES_KCL}/src/Kinematics/include/KinematicsAccess.h"
   "${MODULES_KCL}/src/Kinematics/include/PMPAccess.h"
   "${MODULES_KCL}/src/Kinematics/include/KinematicsThread.h"
   "${MODULES_KCL}/src/Kinematics/include/KinematicsFactory.h")

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("KCL Kinematics Header Files" FILES ${folder_kinematics_header})
SOURCE_GROUP("KCL Kinematics Source Files" FILES ${folder_kinematics_source})


# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
		${folder_kinematics_source}
		${folder_kinematics_header}
    )

    TARGET_LINK_LIBRARIES(${KEYWORD}        

      ${YARP_LIBRARIES}
      )	

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
