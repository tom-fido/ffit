cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# Create sub directory     
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bin/test)

# Boost
rosbuild_add_boost_directories()

# REEM head scan & snapshot Action Server
rosbuild_add_executable(scan_snapshot_action_node src/scan_snapshot_action_node.cpp)

# Example Action Client
rosbuild_add_executable(test/scan_table src/test/scan_table.cpp)

# Boost
rosbuild_link_boost(scan_snapshot_action_node thread)
rosbuild_link_boost(test/scan_table thread)
