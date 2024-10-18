from enum import Enum
from typing import List, Dict
from collections import deque

class direction(Enum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

class Dirs(Enum):
    LEFT = 1
    FORWARDS = 2
    RIGHT = 3
    BACK = 4

class Edge:
    def __init__(self, origin: str, target: str, direction: direction) -> None:
        self.origin = origin
        self.target = target
        self.direction = direction

class Pathfinder:
    def __init__(self) -> None:
        edges = [
            Edge('A', 'D', direction.SOUTH),
            Edge('B', 'C', direction.EAST),
            Edge('C', 'F', direction.SOUTH),
            Edge('D', 'E', direction.EAST),
            Edge('D', 'G', direction.SOUTH),
            Edge('E', 'H', direction.SOUTH),
            Edge('F', 'I', direction.SOUTH),
            Edge('H', 'I', direction.EAST),
            
            # Dead ends
            Edge('A', 'DeadEnd1', direction.NORTH),
            Edge('A', 'DeadEnd2', direction.WEST),
            Edge('D', 'DeadEnd3', direction.WEST),
            Edge('G', 'DeadEnd4', direction.WEST),
            Edge('G', 'DeadEnd5', direction.SOUTH),
            Edge('H', 'DeadEnd6', direction.SOUTH),
            Edge('I', 'DeadEnd7', direction.SOUTH),
            Edge('I', 'DeadEnd8', direction.EAST),
            Edge('F', 'DeadEnd9', direction.EAST),
            Edge('C', 'DeadEnd10', direction.EAST),
            Edge('C', 'DeadEnd11', direction.NORTH),
            Edge('B', 'DeadEnd12', direction.NORTH)
        ]
        self.edges = edges
        self.graph = self.build_graph(edges)

    def build_graph(self, edges: List[Edge]) -> Dict[str, List[Edge]]:
        """
        Builds a graph represented as an adjacency list from the list of edges.
        """
        graph = {}
        for edge in edges:
            graph.setdefault(edge.origin, []).append(edge)
            # If the graph is undirected, add the reverse edge as well
            # For directed graphs, comment out the following two lines
            reverse_direction = direction((edge.direction.value + 2) % 4)
            graph.setdefault(edge.target, []).append(Edge(edge.target, edge.origin, reverse_direction))
        return graph

    def find_shortest_path(self, start: str, end: str) -> List[Edge]:
        """
        Uses BFS to find the shortest path from start to end node.
        """
        queue = deque([(start, [])])
        visited = set()

        while queue:
            current_node, path = queue.popleft()
            if current_node == end:
                return path
            if current_node in visited:
                continue
            visited.add(current_node)
            for edge in self.graph.get(current_node, []):
                if edge.target not in visited:
                    queue.append((edge.target, path + [edge]))
        return []

    def get_directions(self, path: List[Edge]) -> List[str]:
        """
        Given a path (list of Edges), returns a list of directions (Left, Right, Straight).
        """
        directions = []
        if not path:
            return directions

        for i in range(len(path) - 1):
            current_edge = path[i]
            next_edge = path[i + 1]
            turn = self.get_turn(current_edge.direction, next_edge.direction)
            directions.append(turn)
        return directions

    def get_turn(self, current_dir: direction, next_dir: direction) -> str:
        """
        Determines the turn needed to go from current_dir to next_dir.
        """
        direction_to_degrees = {
            direction.NORTH: 0,
            direction.EAST: 90,
            direction.SOUTH: 180,
            direction.WEST: 270,
        }

        current_angle = direction_to_degrees[current_dir]
        next_angle = direction_to_degrees[next_dir]

        angle_diff = (next_angle - current_angle) % 360

        if angle_diff == 0:
            return Dirs.FORWARDS
        elif angle_diff == 90:
            return Dirs.RIGHT
        elif angle_diff == 180:
            return Dirs.BACK
        elif angle_diff == 270:
            return Dirs.LEFT
        else:
            return 'Unknown'

# Example usage:

if __name__ == '__main__':
    # Define the labyrinth based on the image
    # The big nodes are letters
    # The on way street are called DeadEndx according to number
    pf = Pathfinder()
    start_node = 'DeadEnd2'
    end_node = 'DeadEnd12'
    path_edges = pf.find_shortest_path(start_node, end_node)

    if path_edges:
        # Print the sequence of nodes
        nodes_in_path = [edge.origin for edge in path_edges] + [path_edges[-1].target]
        print(f"Shortest path from {start_node} to {end_node}: {nodes_in_path}")

        # Get directions along the path
        directions = pf.get_directions(path_edges)
        print(f"Directions: {directions}")
    else:
        print(f"No path found from {start_node} to {end_node}.")
