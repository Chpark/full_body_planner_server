cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

link_directories(/usr/local/lib)
find_library(ZMQ_LIB zmq)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB_RECURSE PROJECT_HEADER_FILES RELATIVE ${PROJECT_SOURCE_DIR} *.h)
file(GLOB_RECURSE PROJECT_HEADER_FILES2 RELATIVE ${PROJECT_SOURCE_DIR} *.hpp)

rosbuild_add_library(menge_3d_interface
src/3DInterface.cpp
src/3DInterfaceClient.cpp
src/3DInterfaceComponent.cpp
src/3DInterfaceServer.cpp
${PROJECT_HEADER_FILES}
${PROJECT_HEADER_FILES2}
)

rosbuild_add_executable(test_client src/TestClient.cpp)
target_link_libraries(test_client menge_3d_interface)
target_link_libraries(test_client ${ZMQ_LIB})

rosbuild_add_executable(test_server src/TestServer.cpp)
target_link_libraries(test_server menge_3d_interface)
target_link_libraries(test_server ${ZMQ_LIB})

