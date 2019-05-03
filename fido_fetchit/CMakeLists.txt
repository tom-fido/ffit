cmake_minimum_required(VERSION 2.8.3)
project(fido_fetchit)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  #gmapping
)

catkin_package(
)
