cmake_minimum_required(VERSION 3.0.2)
project(cv_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation  # Hinzugefügte Abhängigkeit für die Nachrichtengenerierung
)

# Nachrichten- und Service-Dateien generieren
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge message_runtime  # Hinzugefügte Abhängigkeit für die Nachrichtengenerierung
) 

include_directories(
  ${catkin_INCLUDE_DIRS}
)
