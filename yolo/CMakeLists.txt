cmake_minimum_required(VERSION 3.0.2)
project(yolo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  ros_numpy
  cv_bridge
  sensor_msgs
  message_filters
  geometry_msgs
  move_base_msgs
)

find_package(message_generation)
find_package(map_msgs)


add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msg FILES DetectionInfo.msg DetectionArray.msg)
generate_messages(DEPENDENCIES geometry_msgs map_msgs move_base_msgs)

catkin_package(
  CATKIN_DEPENDS
    rospy
    rospkg
    ros_numpy
    cv_bridge
    sensor_msgs
    message_filters
    geometry_msgs
)

catkin_install_python(PROGRAMS
   src/yolo_detection.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

