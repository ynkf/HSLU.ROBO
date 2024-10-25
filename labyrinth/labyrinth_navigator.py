import numpy as np
import rospy

from camera_subscriber import CameraSubscriber
from wheel_command_publisher import WheelCommandPublisher
from node_detection import NodeDetection
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
        self.previous_direction = self.get_direction(path[0], path[1])
        


    def run(self):
        image = self.camera_subscriber.get_image()
        node_index = self.node_detection.get_node_index(image)
        
        if node_index:
            #stop wheels
            current_node = path[node_index]
            next_node = path[node_index + 1]
            
            current_direction = self.get_direction(current_node, next_node)

        
        
        


    def get_direction(self, current_node, next_node):
        return direction_map.get((next_node[0] - current_node[0], next_node[1] - current_node[1]))
    
    def navigate(self, node_image, node_count):
        
                    
                    
    def is_turnnode(self, node):
        
    def direction(self, path):
        
    def turning_direction(self):
        
    def stop(self):
        self.wheel_command_publisher.turn_wheels(0, 0)
        
                    
                    
                    
                    
                    
                    
# Simulated path
path = [(10, 0), (10, 1), (10, 2), (10, 3), (10, 4), (9, 4), (8, 4), (8, 3)]

# Dictionary for directions
direction_map = {
    (0, -1): 'LEFT',
    (0, 1): 'RIGHT',
    (-1, 0): 'UP',
    (1, 0): 'DOWN'
}

# Define possible turning directions
turning_rules = {
    ('RIGHT', 'UP'): ('LEFT', 90),
    ('UP', 'LEFT'): ('LEFT', 90),
    ('LEFT', 'DOWN'): ('LEFT', 90),
    ('DOWN', 'RIGHT'): ('LEFT', 90),
    
    ('UP', 'RIGHT'): ('RIGHT', 90),
    ('LEFT', 'UP'): ('RIGHT', 90),
    ('DOWN', 'LEFT'): ('RIGHT', 90),
    ('RIGHT', 'DOWN'): ('RIGHT', 90),
}

# List to store turn nodes and details
turn_nodes = []

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