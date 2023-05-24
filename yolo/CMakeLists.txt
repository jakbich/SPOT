cmake_minimum_required(VERSION 3.0.2)
project(yolo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ros_numpy
  cv_bridge
  sensor_msgs
  message_filters
  vision_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    rospkg
    ros_numpy
    cv_bridge
    sensor_msgs
    message_filters
    vision_msgs
    geometry_msgs
)

catkin_install_python(PROGRAMS
   src/yolo_detection.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

