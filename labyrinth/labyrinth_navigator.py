import numpy as np
import rospy

from camera_subscriber import CameraSubscriber
from wheel_command_publisher import WheelCommandPublisher
from node_detection import NodeDetection
from line_follower import LineFollower
from bfs import BFS

class LabyrinthNavigator:
    
    # Dictionary for directions
    direction_map = {
        (0, -1): 'LEFT',
        (0, 1): 'RIGHT',
        (-1, 0): 'UP',
        (1, 0): 'DOWN'
    }
        
    def __init__(self, robot_name):
        rospy.init_node('LabyrinthNavigator', anonymous=True)
        self.wheel_command_publisher = WheelCommandPublisher(robot_name)
        self.camera_subscriber = CameraSubscriber(robot_name)
        self.node_detection = NodeDetection()
        self.line_follower = LineFollower()
        simple_matrix = np.array([
            [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '],
            [' ', 'X', 'X', 'X', ' ', 'X', 'X', ' ', ' '],
            [' ', 'X', ' ', 'X', ' ', 'X', ' ', ' ', ' '],
            [' ', 'X', ' ', 'X', 'X', 'X', 'X', 'E', ' '],
            [' ', 'X', ' ', 'X', ' ', 'X', ' ', ' ', ' '],
            [' ', 'S', ' ', 'X', 'X', 'X', ' ', ' ', ' '],
            [' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ']
        ])
        self.bfs = BFS(simple_matrix)
        self.path = self.bfs.shortest_path()
        self.current_node_index = 0
        self.current_direction = self.get_direction(self.path[0], self.path[1])
        


    def run(self):
        current_node = self.path[self.current_node_index]
        next_node = self.path[self.current_node_index + 1]
        # is current node end node -> stop labyrinth navigator

        # calculate direction for next node
        next_direction = self.get_direction(current_node, next_node)

        # if next node is not same direction as before -> turn to the correct direction
        if next_direction != self.current_direction:
            # call turn script
            # turningLogic.turn(current_direction, next_direction)
            pass

        # call line follower and wait until it reached the next node
        

        while(not is_node):
            self.line_follower.follow_line()
        pass
        
        self.current_node_index += 1


        # image = self.camera_subscriber.get_image()
        # node_index = self.node_detection.get_node_index(image)
        
        # if node_index:
        #     current_node = path[node_index]
        #     next_node = path[node_index + 1]
            
        #     current_direction = self.get_direction(current_node, next_node)

        #     if current_direction != self.previous_direction:
        #         # stop wheels

        #     self.previous_direction = current_direction


    def follow_line(self):
        #node detection
        return
    
    def decide_direction(self):
        return

    def turn_on_node(self):
        return
        


    def get_direction(self, current_node, next_node):
        return direction_map.get((next_node[0] - current_node[0], next_node[1] - current_node[1]))
    
    def navigate(self, node_image, node_count):
        
                    
                    
                    
                    
                    
                    
# Variable to track the previous direction
previous_direction = None

# Single loop to calculate directions and detect turns
for i in range(len(path) - 1):
    path_direction = (path[i + 1][0] - path[i][0], path[i][1] - path[i + 1][1])
    
    # Get the corresponding direction from the map
    current_direction = direction_map.get(path_direction, 'UNKNOWN')

    # Check if the direction has changed (indicating a turn)
    if previous_direction and current_direction != previous_direction:
        turn_type, turn_angle = turning_rules.get((previous_direction, current_direction), ('UNKNOWN', 0))
        turn_nodes.append((i, turn_type, turn_angle))
        print(f'Node: {i} is a turn node! Turn: {turn_type} by {turn_angle}°')

    # Update previous direction for the next iteration
    previous_direction = current_direction

# Output result
print("Turn Nodes:", turn_nodes)

if __name__ == 'main':
    labyrinthNavigator = LabyrinthNavigator("alpha")
    x = 0
    while (x < 100):
        labyrinthNavigator.run(x)
        rospy.sleep(0.2)
        x += 1
    labyrinthNavigator.stop()