reem_object_recognition
=======================

Configuration files for REEM and Object Recognition Kitchen. 

Install dependencies: 
sudo apt-get install ros-hydro-object_recognition_*

The following rosinstall file will get you an up-to-date REEM simulation: 
https://github.com/pal-robotics/pal-ros-pkg/blob/master/reem-sim-hydro.rosinstall  

To launch visualized detections:
rosrun object_recognition_core detection -c \`rospack find reem_object_recognition\`/config/tabletop/detection.clusters.ros.ork.reem --graphviz

rosrun object_recognition_ros server -c \`rospack find reem_object_recognition\`/config/tabletop/detection.clusters.ros.ork.reem

