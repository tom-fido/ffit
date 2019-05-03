cmake_minimum_required(VERSION 2.8.3)
project(fetch_gazebo_world_grasping)

find_package(gazebo REQUIRED)

find_package(catkin
  REQUIRED
    gazebo_ros
    fetch_gazebo
)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS
    #control_toolbox
    #gazebo_ros
    #robot_controllers
    #robot_controllers_interface
  DEPENDS
    gazebo
  LIBRARIES
    #fetch_gazebo_plugin
  INCLUDE_DIRS
    #include
)



install(
  DIRECTORY launch worlds models
  #config include launch robots worlds models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
