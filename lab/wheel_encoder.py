#!/usr/bin/python3

import os
import rospy
from duckietown_msgs.msg import WheelEncoderStamped

class WheelEncoderTracker:
    
    def __init__(self, robot_name):
        self.left_ticks = None
        self.right_ticks = None      
    
        # Subscribe to the left and right wheel encoder topics
        rospy.Subscriber(f"{robot_name}/left_wheel_encoder_node/tick", WheelEncoderStamped, self.left_wheel_callback)
        rospy.Subscriber(f"{robot_name}/right_wheel_encoder_node/tick", WheelEncoderStamped, self.right_wheel_callback)
        
    def left_wheel_callback(self, msg):
        self.left_ticks = msg.data
        
    def right_wheel_callback(self, msg):
        self.right_ticks = msg.data

if __name__ == "__main__":
    robot_name = os.environ["VEHICLE_NAME"]
    
    rospy.init_node('wheel_encoder_listener', anonymous=True)
    
    we = WheelEncoderTracker(robot_name)
    
    from wheel_command_publisher import WheelCommandPublisher
    wcp = WheelCommandPublisher(robot_name)
    
    init_left = we.left_ticks
    NTICKS = 380 * 1
    if init_left is not None:
        rospy.loginfo(f"Turning {NTICKS} ticks left...")
        wcp.set_left_right(0.1, 0)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if we.left_ticks - NTICKS > init_left:
                break
            rate.sleep()
        wcp.set_left_right(0, 0)
        rospy.loginfo("Finished turning")
    else:
        rospy.loginfo("Left_ticks is None.")
    