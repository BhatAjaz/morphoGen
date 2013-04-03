# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Alessandro Roncone
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
set(PROJECTNAME doubleTouch)
project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

# add include directories
include_directories(. ${YARP_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)

# add required linker flags
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

add_executable(${PROJECTNAME} ./include/doubleTouchThread.h ./src/doubleTouchThread.cpp ./include/utils.h ./src/utils.cpp ./src/main.cpp)
add_executable(doubleTouchTest ./src/maintest.cpp)

# we now add the YARP and iCub libraries to our project.
target_link_libraries(${PROJECTNAME} icubmod ${YARP_LIBRARIES} ${ICUB_LIBRARIES} pmp)
target_link_libraries(doubleTouchTest icubmod ${YARP_LIBRARIES} ${ICUB_LIBRARIES} pmp)