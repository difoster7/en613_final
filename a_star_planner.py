import signal
import sys

import numpy as np
from typing import List, Tuple

from heapq import heappush, heappop


class Edge: 
    """
    This class provides a basic data structure for representing
    a directional edge in a graph. Travel is possible between
    the starting node to the ending node at the given cost
    but travel in the opposite direction is not allowed.
    """
    def __init__(self,starting_node, ending_node, cost):
        self.start = starting_node
        self.end = ending_node 
        self.cost = cost 

    def __repr__(self):
        return 'Node'+self.__str__()
    def __str__(self):
        return f'({self.start.name},{self.end.name},{self.cost})'

    def __eq__(self,obj):
        if  isinstance(obj, Edge):
            return self.start == obj.start and obj.end == obj.end and self.cost == self.cost 
        return False

class Node:
    """
    This class provides a basic data structure for representing
    a node in A* Graph
    """
    def __init__(self, name, h):
        #The name of the node (can be anything, just for human readable output)
        self.name = name
        #The current best cost-to-come for the node
        self.g = np.inf 
        #The current best estimate of the node's total cost
        self.f = np.inf 
        #The heuristic estimate of the cost-to-go for the node
        self.h = h 
        #The list of edges which connect the node 
        self.edges = []
        #The previous node in path to the goal
        self.previous = None

    def add_neighbor(self, node, cost):
        new_edge = Edge(self, node, cost)
        self.edges.append(new_edge)

    def add_neighbor_bidirectional(self, node, cost):
        self.add_neighbor(node, cost)
        node.add_neighbor(self, cost)


    def __str__(self):
        return f'({self.name},{self.f},{self.g},{self.h})'

    def __eq__(self,obj):
        if  isinstance(obj, Node):
            return self.name == obj.name and self.f == self.f  and obj.g == obj.g and self.h == self.h 
        return False

    def __ge__(self, other):
        return self.f >= other.f

    def __lt__(self, other):
        return self.f < other.f

def get_node_previous_list(current_node: Node):
    return_list = [current_node]
    while current_node.previous != None:
        return_list.append(current_node.previous)
        current_node = current_node.previous
    return_list.reverse()
    return return_list

# Function to calculate Euclidean distance between two points on a grid
def grid_distance(node1:Tuple[int, int], node2:Tuple[int,int]):
    return np.sqrt((node1[0] - node2[0])**2 + (node1[1] - node2[1])**2)

def a_star_grid(map: np.ndarray, start:Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    """
    This function will compute the optimal path between a start point and an end point given a grid-based
    map. It is up to the student to implement the heuristic function and cost function. Assume a cell's 
    indices represent it's position in cartesian space. (e.g. cells [1,3] and [1,5] are 2 units apart). 

    If no path exists then this function should return an empty list.

    Worth 50 pts
    
    Input
      :param map: An np.ndarray representing free space and occupied space
      :param start: A tuple of indicies indicating the starting cell of the search
      :param goal: A tuple of indicies indicating the goal cell of the search

    Output
      :return: path: a list of Tuples indicating the indicies of the cells that make up the path with 
                    the starting cell as the first element of the list and the ending cell as the last
                    element in the list
    """

    # Function to determine all possible neighbors of a node
    def get_grid_neighbors(node: Node):
        coords = node.name
        all_neighbors = []
        all_neighbors.append((coords[0]+1, coords[1]))
        #all_neighbors.append((coords[0]+1, coords[1]+1))
        all_neighbors.append((coords[0], coords[1]+1))
        #all_neighbors.append((coords[0]-1, coords[1]+1))
        all_neighbors.append((coords[0]-1, coords[1]))
        #all_neighbors.append((coords[0]-1, coords[1]-1))
        all_neighbors.append((coords[0], coords[1]-1))
        #all_neighbors.append((coords[0]+1, coords[1]-1))

        legal_neighbors = []
        grid_size = np.shape(map)
        for coord in all_neighbors:
            if coord[0] >= 0 and coord[0] < grid_size[0] and coord[1] >= 0 and coord[1] < grid_size[1]:
                if map[coord] == 0:
                    legal_neighbors.append(coord)
        return legal_neighbors

    # Recursive function for an a* grid
    def a_star_grid_recursive():
        if len(open_list) == 0:
            return []
        
        # Sort open list by current best guess cost
        open_list.sort(key=lambda x:x.f)

        # Removed best node from open list and add to closed list
        best_node = open_list.pop(0)
        closed_list.append(best_node)

        # Check if goal is found
        if best_node.name == goal:
            return get_node_previous_list(best_node)

        # Add neighbors of best node and update distance estimates
        legal_neighbors = get_grid_neighbors(best_node)
        legal_neighbors[:] = [neighbor for neighbor in legal_neighbors if not any(node.name == neighbor for node in closed_list)]
        for neighbor in legal_neighbors:    # Update/add neighbor to the open list
            if any(node.name == neighbor for node in open_list):
                same_neighbor = next(node for node in open_list if node.name == neighbor)
                if best_node.g + grid_distance(best_node.name, neighbor) < same_neighbor.g:
                        same_neighbor.g = best_node.g + grid_distance(best_node.name, neighbor)
                        same_neighbor.f = same_neighbor.g + same_neighbor.h
                        same_neighbor.previous = best_node
            else:
                new_node = Node(neighbor, None)
                new_node.g = best_node.g + grid_distance(best_node.name, neighbor)
                new_node.h = grid_distance(new_node.name, goal)
                new_node.f = new_node.g + new_node.h
                new_node.previous = best_node
                open_list.append(new_node)

        return a_star_grid_recursive() # Recursive call
    
    # Initialize the recursive function
    open_list = []
    closed_list = []
    start_node = Node(start, None)
    start_node.g = 0
    start_node.h = grid_distance(start, goal)
    start_node.f = start_node.g + start_node.h
    open_list.append(start_node)

    return a_star_grid_recursive() # Start recursion

def a_star_graph(start: Node, goal: Node) -> List[Node]:
    """
    This function will compute the optimal path between a starting node and an ending node.
    The result should be a list of the Edges that represent the optimal path to the goal. 
    For this function the cost and heuristic functions are defined when the node is originally created.

    
    If no path exists then this function should return an empty list.

    Worth 50 pts
    
    Input
      :param start: The starting node of the search
      :param goal: The ending node of the search

    Output
      :return: path: a list of Node objects representing the optimal path to the goal 
    """

    # Recursive function for an a* graph
    def a_star_graph_recursive():
        if len(open_list) == 0:
            return []
        
        # Sort open list by current best guess cost
        open_list.sort(key=lambda x:x.f)

        # Removed best node from open list and add to closed list
        best_node = open_list.pop(0)
        closed_list.append(best_node)

        # Check if goal is found
        if best_node.name == goal.name:
            return get_node_previous_list(best_node)

        # Add neighbors of best node and update distance estimates
        for edge in best_node.edges:
            if any(node == edge.end for node in closed_list):
                continue
            if any(node.name == edge.end.name for node in open_list):
                node_to_update = next(node for node in open_list if node.name == edge.end.name)
                if best_node.g + edge.cost < node_to_update.g:
                    node_to_update.g = best_node.g + edge.cost
                    node_to_update.f = node_to_update.g + node_to_update.h
            else:
                new_node = edge.end
                new_node.g = best_node.g + edge.cost
                new_node.f = best_node.g + best_node.f
                new_node.previous = best_node
                open_list.append(new_node)

        return a_star_graph_recursive() # Recursive call
    

    # Initialize the recursive function
    open_list = []
    closed_list = []
    start.g = 0
    start.f = start.g + start.h
    open_list.append(start)

    return a_star_graph_recursive() # Start recursion

def graph_demo():
    nodes = []

    nodes.append(Node('A', 10)) # A
    nodes.append(Node('B', 5))  # B
    nodes.append(Node('C', 6))  # C
    nodes.append(Node('D', 2))  # D
    nodes.append(Node('E', 3))  # E
    nodes.append(Node('F', 0))  # F

    nodes[0].add_neighbor(nodes[1],3)
    nodes[0].add_neighbor(nodes[2],4)
    nodes[1].add_neighbor(nodes[3],2)
    nodes[1].add_neighbor(nodes[4],2)
    nodes[2].add_neighbor(nodes[4],4)
    nodes[3].add_neighbor(nodes[5],5)
    nodes[3].add_neighbor(nodes[4],4)
    nodes[4].add_neighbor(nodes[5],4)

    path = a_star_graph(nodes[0],nodes[-1])

    for e_i in path:
        print(e_i)

def grid_demo():

    map = np.array([[0,0,1,0,0,0,0,0,0],
                    [0,0,1,0,0,0,0,0,0],
                    [0,0,1,0,0,1,1,1,0],
                    [0,0,1,0,0,1,0,0,0],
                    [0,0,1,0,0,1,0,1,1],
                    [0,0,1,0,0,1,0,0,0],
                    [0,0,0,0,0,1,0,0,0],
                    [0,0,0,0,0,1,0,0,0],
                    [0,0,0,0,0,1,0,0,0],
        ])

    start = (0,0)
    goal =  (8,8)
    path = a_star_grid(map, start, goal)
    for c_i in path:
        print(c_i)

def main():
    graph_demo()
    grid_demo()

if __name__ == '__main__':
    main()
