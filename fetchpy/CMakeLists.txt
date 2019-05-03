cmake_minimum_required(VERSION 2.8.3)
project(fetchpy)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  #fetch_base_controller
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_python_setup()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY config/
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}/config"
)
install(PROGRAMS scripts/console.py
  DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)
