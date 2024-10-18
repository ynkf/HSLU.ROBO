#!/usr/bin/python3

import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header



class WheelCommandPublisher:

    def __init__(self, robot_name): 
        # create a publisher for a topic of type duckietown_msgs.msg.WheelsCmdStamped
        topic = '/' + robot_name + '/wheels_driver_node/wheels_cmd'
        self.publisher = rospy.Publisher(topic, WheelsCmdStamped, queue_size = 10)
        rospy.sleep(2.0)
        
        # create a WheelsCmdStamped message
        self.command = WheelsCmdStamped()
        self.command.header = Header()
        
    def set_left_right(self, vel_left, vel_right):
        self.command.vel_left = vel_left
        self.command.vel_right = vel_right

        self.command.header.stamp = rospy.Time.now()
        self.publisher.publish(self.command)