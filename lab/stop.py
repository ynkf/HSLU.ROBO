#!/usr/bin/python3

import os
import rospy
from wheel_command_publisher import WheelCommandPublisher

if __name__ == "__main__":
    robot_name = os.environ["VEHICLE_NAME"]
    
    rospy.init_node("stop_all", anonymous=False)
    wcp = WheelCommandPublisher(robot_name)
    
    wcp.set_left_right(0, 0)
    rospy.loginfo(f"Stopped movement. Done!")
    