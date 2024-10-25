#!/usr/bin/python3

import rospy
import numpy as np

class BFS:

    def __init__(self, matrix):
        self.graph = self.matrix_to_graph(matrix)


    def shortest_path(self, start_node, end_node):
        """
            Find the shortest path in a graph.
            Parameters:
                start_node: key for the start node in the graph
                end_node: key for the end node in the graph
        """
        path_list = [[start_node]]
        path_index = 0
        # To keep track of previously visited nodes
        previous_nodes = {start_node}
        if start_node == end_node:
            return path_list[0]
            
        while path_index < len(path_list):
            current_path = path_list[path_index]
            last_node = current_path[-1]
            next_nodes = self.graph[last_node]
            # Search goal node
            if end_node in next_nodes:
                current_path.append(end_node)
                return current_path
            # Add new paths
            for next_node in next_nodes:
                if not next_node in previous_nodes:
                    new_path = current_path[:]
                    new_path.append(next_node)
                    path_list.append(new_path)
                    # To avoid backtracking
                    previous_nodes.add(next_node)
            # Continue to next path in list
            path_index += 1
        # No path is found
        return []


    def matrix_to_graph(self, matrix):
        graph = {}

        for row in range(labyrinth_matrix.shape[0]):
            for column in range(labyrinth_matrix.shape[1]):
                if labyrinth_matrix[row][column] == ' ':
                    continue
                neighbors = list()
                if row - 1 >= 0 and labyrinth_matrix[row - 1][column] == 'X':
                    neighbors.append((row - 1, column))
                if row + 1 < labyrinth_matrix.shape[0] and labyrinth_matrix[row + 1][column] == 'X':
                    neighbors.append((row + 1, column))
                if column - 1 >= 0 and labyrinth_matrix[row][column - 1] == 'X':
                    neighbors.append((row, column - 1))
                if column + 1 < labyrinth_matrix.shape[1] and labyrinth_matrix[row][column + 1] == 'X':
                    neighbors.append((row, column + 1))
                
                graph[(row, column)] = neighbors

        return graph
        


if __name__ == '__main__':
    labyrinth_matrix = np.array([
        ['X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'],
        ['X', ' ', ' ', ' ', ' ', ' ', ' ', ' ', 'X', ' ', ' ', ' ', ' '],
        ['X', ' ', ' ', 'X', ' ', ' ', ' ', ' ', 'X', ' ', ' ', ' ', 'X'],
        ['X', ' ', ' ', 'X', ' ', ' ', ' ', ' ', 'X', ' ', ' ', ' ', 'X'],
        ['X', 'X', 'X', 'X', 'X', 'X', 'X', ' ', 'X', 'X', 'X', 'X', 'X'],
        ['X', ' ', ' ', ' ', ' ', ' ', 'X', ' ', 'X', ' ', ' ', ' ', 'X'],
        ['X', ' ', 'X', 'X', 'X', ' ', 'X', ' ', 'X', 'X', 'X', ' ', 'X'],
        ['X', ' ', 'X', ' ', 'X', ' ', 'X', ' ', ' ', ' ', ' ', ' ', 'X'],
        ['X', 'X', 'X', ' ', 'X', ' ', 'X', ' ', 'X', 'X', 'X', 'X', 'X'],
        [' ', ' ', ' ', ' ', 'X', ' ', ' ', ' ', 'X', ' ', ' ', ' ', ' '],
        ['X', 'X', 'X', 'X', 'X', 'X', 'X', ' ', 'X', 'X', 'X', 'X', 'X']
    ])

    bfs = BFS(labyrinth_matrix)
    path = bfs.shortest_path((10, 0), (10, 12))

    result = labyrinth_matrix.copy()
    for row, column in path:
        result[row][column] = '0'
    print(result)
