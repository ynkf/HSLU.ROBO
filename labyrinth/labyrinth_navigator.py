import numpy as np
import rospy

from wheel_command_publisher import WheelCommandPublisher
from line_follower import LineFollower
from turning import Turning
from bfs import BFS

from utils.labyrinths import *
from utils.directions import direction_map

class LabyrinthNavigator:
        
    def __init__(self, labyrinth, robot_name):
        rospy.init_node('LabyrinthNavigator', anonymous=True)

        self.wheel_command_publisher = WheelCommandPublisher(robot_name)
        self.line_follower = LineFollower(robot_name)
        self.turning = Turning(robot_name)
        self.bfs = BFS(labyrinth)

        self.path = self.bfs.shortest_path()
        self.current_node_index = 0
        self.current_direction = self.get_direction(self.path[0], self.path[1])
        

    def run(self):
        while (self.current_node_index < self.path[-1].index):
            current_node = self.path[self.current_node_index]
            next_node = self.path[self.current_node_index + 1]            

            # calculate direction for next node
            next_direction = self.get_direction(current_node, next_node)

            # if next node is not same direction as before -> turn to the correct direction
            if next_direction != self.current_direction:
                turning_direction = self.turning.turning_direction(self.current_direction, next_direction)
                self.turning.turn(turning_direction)
            
            # call line follower and wait until it reached the next node
            self.line_follower.follow_line_until_node()
            
            self.current_node_index += 1


    def get_direction(self, current_node, next_node):
        return direction_map.get((next_node[0] - current_node[0], next_node[1] - current_node[1]))
        

if __name__ == 'main':
    matrix = simple_matrix()

    labyrinthNavigator = LabyrinthNavigator(simple_matrix, "alpha")
    labyrinthNavigator.run()