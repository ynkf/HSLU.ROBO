#!/usr/bin/python3

import numpy as np
import rospy

from wheel_command_publisher import WheelCommandPublisher
# from lane_follower_02 import LaneFollower
# from line_follower import LineFollower
from line_follower import LineFollower
from turning import Turning
from bfs import BFS

from utils.labyrinths import *
from utils.directions import direction_map

class LabyrinthNavigator:
        
    def __init__(self, labyrinth, robot_name):
        rospy.init_node('LabyrinthNavigator', anonymous=True)

        self.wheel_command_publisher = WheelCommandPublisher(robot_name)
        # self.line_follower = LaneFollower(robot_name)
        self.line_follower = LineFollower(robot_name)
        self.turning = Turning(robot_name)
        self.bfs = BFS(labyrinth)

        self.path = self.bfs.shortest_path()
        self.bfs.visualize(self.path)
        rospy.loginfo(self.path)

        self.current_node_index = 0
        self.current_direction = self.get_direction(self.path[0], self.path[1])
        

    def run(self):
        while (self.current_node_index < len(self.path) - 1):
            rospy.loginfo(f'Start Iteration: {self.current_node_index}')
            current_node = self.path[self.current_node_index]
            next_node = self.path[self.current_node_index + 1]            

            # calculate direction for next node
            next_direction = self.get_direction(current_node, next_node)
            rospy.loginfo(f'Next Direction: {next_direction}')

            # if next node is not same direction as before -> turn to the correct direction
            if next_direction != self.current_direction:
                turning_direction = self.turning.turning_direction(self.current_direction, next_direction)
                self.turning.turn(turning_direction)
            
            # call line follower and wait until it reached the next node
            self.line_follower.follow_line_until_node()
            self.current_direction = next_direction
            
            # stop at each node for 1 second
            # self.wheel_command_publisher.stop_wheels()
            # rospy.sleep(1)
            
            self.current_node_index += 1
        
        rospy.sleep(1)
        self.wheel_command_publisher.stop_wheels()


    def get_direction(self, current_node, next_node):
        return direction_map.get((next_node[0] - current_node[0], next_node[1] - current_node[1]))
        

if __name__ == '__main__':
    matrix = simple_matrix()
    # rospy.loginfo("Start labyrinth navigator with matrix:")
    # rospy.loginfo(matrix)

    labyrinthNavigator = LabyrinthNavigator(matrix, "alpha")
    labyrinthNavigator.run()