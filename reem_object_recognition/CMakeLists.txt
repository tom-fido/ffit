cmake_minimum_required(VERSION 2.8.3)
project(reem_object_recognition)

find_package(catkin REQUIRED COMPONENTS
  object_recognition_ros
  object_recognition_tabletop
)

catkin_package()

foreach(dir config rviz)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
