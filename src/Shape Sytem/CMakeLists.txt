include(${ROBVIS_CMAKE_DIRECTORY}/externals/lib_opencv/head.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../yarp/CMakeLists.txt)

robvis_create_executable_project(cvutShapeDet)

robvis_target_link_libraries(cvutShapeDet ${OPENCV_ALL_TARGETS})
robvis_target_link_libraries(cvutShapeDet ${YARP_ALL_TARGETS})
