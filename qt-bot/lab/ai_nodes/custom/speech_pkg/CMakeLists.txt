cmake_minimum_required(VERSION 3.0.2)
project(speech_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  custom_interfaces # Füge das Paket hinzu, das den Service definiert
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime custom_interfaces
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
