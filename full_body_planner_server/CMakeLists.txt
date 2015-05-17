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

link_directories(/usr/local/lib)
find_library(ZMQ_LIB zmq)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

file(GLOB_RECURSE MOVE_ITOMP_HEADER_FILES RELATIVE ${PROJECT_SOURCE_DIR} *.h)

rosbuild_add_executable(full_body_planner_server
src/full_body_planner_server.cpp
src/waypoint_2d.cpp
${MOVE_ITOMP_HEADER_FILES}
)

target_link_libraries(full_body_planner_server menge_3d_interface)
target_link_libraries(full_body_planner_server ${ZMQ_LIB})

rosbuild_add_executable(test_client
src/test_client.cpp
src/waypoint_2d.cpp
${MOVE_ITOMP_HEADER_FILES}
)

target_link_libraries(test_client menge_3d_interface)
target_link_libraries(test_client ${ZMQ_LIB})

rosbuild_add_executable(play_trajectory
src/play_trajectory.cpp
${MOVE_ITOMP_HEADER_FILES}
)

