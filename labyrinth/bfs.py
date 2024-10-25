#!/usr/bin/python3

import rospy
import numpy as np

class BFS:

    def __init__(self, matrix):
        graph, start_node, end_node = self.matrix_to_graph(matrix)
        self.graph = graph
        self.start_node = start_node
        self.end_node = end_node


    def shortest_path(self):
        """
            Find the shortest path in a graph.
            Parameters:
                start_node: key for the start node in the graph
                end_node: key for the end node in the graph
        """
        path_list = [[self.start_node]]
        path_index = 0
        # To keep track of previously visited nodes
        previous_nodes = {self.start_node}
        if self.start_node == self.end_node:
            return path_list[0]
            
        while path_index < len(path_list):
            current_path = path_list[path_index]
            last_node = current_path[-1]
            next_nodes = self.graph[last_node]
            # Search goal node
            if self.end_node in next_nodes:
                current_path.append(self.end_node)
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
        start_node = None
        end_node = None

        for row in range(matrix.shape[0]):
            for column in range(matrix.shape[1]):
                node = matrix[row][column]
                if node == ' ':
                    continue
                if node == 'S':
                    start_node = (row, column)
                if node == 'E':
                    end_node = (row, column)

                neighbors = list()
                if row - 1 >= 0 and matrix[row - 1][column] in ['X', 'S', 'E']:
                    neighbors.append((row - 1, column))
                if row + 1 < matrix.shape[0] and matrix[row + 1][column] in ['X', 'S', 'E']:
                    neighbors.append((row + 1, column))
                if column - 1 >= 0 and matrix[row][column - 1] in ['X', 'S', 'E']:
                    neighbors.append((row, column - 1))
                if column + 1 < matrix.shape[1] and matrix[row][column + 1] in ['X', 'S', 'E']:
                    neighbors.append((row, column + 1))
                
                graph[(row, column)] = neighbors

        return graph, start_node, end_node
        


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
        ['S', 'X', 'X', 'X', 'X', 'X', 'X', ' ', 'X', 'X', 'X', 'X', 'E']
    ])

    bfs = BFS(labyrinth_matrix)
    path = bfs.shortest_path()

    result = labyrinth_matrix.copy()
    for row, column in path:
        result[row][column] = '0'
    result[bfs.start_node[0]][bfs.start_node[1]] = 'S'
    result[bfs.end_node[0]][bfs.end_node[1]] = 'E'
    print(result)
