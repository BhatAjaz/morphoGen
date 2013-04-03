cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME 3DObjectReconstruction)

project(${PROJECTNAME})

find_package(PCL 1.6 REQUIRED)

find_package(YARP)
find_package(ICUB)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubHelpers)

find_package(OpenCV)
find_package(stereoVision)

#thrift file generation -- remember to compile YARP with CREATE_IDLS and ENABLE_yarpidl_thrift options, and to define ALLOW_IDL_GENERATION=ON when configuring this project
INCLUDE(${YARP_MODULE_PATH}/YarpIDL.cmake)
SET(generated_files_dir "${CMAKE_CURRENT_SOURCE_DIR}")
#yarp_idl_to_dir(3Ddata/3DObjects.thrift ${generated_files_dir})

INCLUDE(3Ddata_3DObjects_thrift.cmake)
set(3d_data_thrift_sources ${sources})
set(3d_data_thrift_headers ${headers})
SOURCE_GROUP("3D Thrift Source Files" FILES ${3d_data_thrift_sources})
SOURCE_GROUP("3D Thrift Header Files" FILES ${3d_data_thrift_headers})


set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

set(segm_interface_dir ${CMAKE_CURRENT_SOURCE_DIR})

include(${segm_interface_dir}/graphBasedSegmentationInterface_thrift.cmake)
set(segm_thrift_sources ${sources})
set(segm_thrift_headers ${headers})
SOURCE_GROUP("Segmentation Thrift Source Files" FILES ${segm_thrift_sources})
SOURCE_GROUP("Segmentation Thrift Header Files" FILES ${segm_thrift_headers})

include_directories(${PCL_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} ${YARP_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include ${ICUB_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${stereoVision_INCLUDE_DIRS} ${generated_files_dir}/include ${segm_interface_dir}/include)

#file(GLOB source src/*.cpp)
set(source src/main.cpp src/communicator.cpp src/helpers.cpp src/minBoundBox.cpp src/objectReconstr.cpp src/reconstructionRoutine.cpp)
#file(GLOB header include/*.h)
set(header include/communicator.h include/helpers.h include/minBoundBox.h include/objectReconstr.h include/reconstructionRoutine.h)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(${PROJECTNAME} ${source} ${header} ${3d_data_thrift_sources} ${3d_data_thrift_headers} ${segm_thrift_sources} ${segm_thrift_headers})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} stereoVision ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
install(TARGETS ${PROJECTNAME} DESTINATION bin)
