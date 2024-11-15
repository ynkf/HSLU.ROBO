#!/bin/bash

source /opt/ros/noetic/setup.bash

cd /root/catkin_ws
catkin_make  

source devel/setup.bash

sleep 15
# rosrun speech_pkg speech_recognition_topic_node.py &
rosrun speech_pkg audio_to_virtual_device_node.py &
rosrun speech_pkg speech_recognition_microphone_node.py _microphone_device_index:=1 &
rosrun cv_pkg detectron_node.py &
rosrun cv_pkg deep_face_node.py &

while true; do
    sleep 1
done
