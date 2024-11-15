#!/usr/bin/python3

import rospy
from wheel_command_publisher import WheelCommandPublisher

class Turning:
    
    def __init__(self, robot_name):
        self.wheel_command_publisher = WheelCommandPublisher(robot_name)
        self.turning_rules = {
            ('RIGHT', 'UP'): 'LEFT',
            ('UP', 'LEFT'): 'LEFT',
            ('LEFT', 'DOWN'): 'LEFT',
            ('DOWN', 'RIGHT'): 'LEFT',
        
            ('UP', 'RIGHT'): 'RIGHT',
            ('LEFT', 'UP'): 'RIGHT',
            ('DOWN', 'LEFT'): 'RIGHT',
            ('RIGHT', 'DOWN'): 'RIGHT',
        }
    
    def turning_direction(self, current_direction, next_direction):
        rospy.loginfo(f'calculate turning direction: {(current_direction, next_direction)}')
        rospy.loginfo(f'turning direction: {self.turning_rules.get((current_direction, next_direction))}')
        return self.turning_rules.get((current_direction, next_direction))
    
    def turn(self, turning_direction):
        rospy.loginfo(turning_direction)
        self.wheel_command_publisher.turn_wheels(0.1, 0.1)
        rospy.sleep(0.3)
        self.wheel_command_publisher.stop_wheels()
        rospy.sleep(1)
        if turning_direction == 'LEFT':
            self.wheel_command_publisher.turn_wheels(-0.2, 0.2)
        elif turning_direction == 'RIGHT':
            self.wheel_command_publisher.turn_wheels(0.2, -0.2)
        rospy.sleep(0.9) # TEST! -> should be 90°
        self.wheel_command_publisher.stop_wheels()
        rospy.sleep(1)
