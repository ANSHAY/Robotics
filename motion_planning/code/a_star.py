# Peer assignment 1 for Coursera Course
# Modern Robotics: Robot Motion Planning and Control
# for week 1.
# Implementation of A* algorithm which finds the
# optimal path in a graph defined by nodes and edges
#
# inputs:
#     nodes: nodes in a graph read from nodes.csv
#     edges: edges in the graph read from edges.csv
# outputs:
#     path: writes the optimal path to path.csv
#
# Author: Anshay
# Date: 07/01/2021

# import libraries
import heapq                                # minheap for open_list of nodes
from Nodes import *
from read_write import *

# A* algorithm
def a_star(nodes, edges):
    path = []                                           # path for output
    open = list(nodes.values())                         # open list of nodes
    heapq.heapify(open)
    closed = {}                                         # closed list
    for n in nodes.keys():
        closed[n] = False
    goal_id = len(nodes)                                # goal node id
    while (len(open) > 0):
        # take first node from open_list and put into current
        current = heapq.heappop(open)
        closed[current.id] = True
        # if current is same as goal then success. Build the path and return
        if current.id == goal_id:
            # update path from current to 1
            while current.id != 1:
                path.append(current.id)
                current = current.parent
            path.append(1)
            return path[::-1]
        # if current is not the goal then look at all neighbors of current
        for n in edges[current.id].keys():
            nbr = nodes[n]
            if not closed[nbr.id]:
                tentative_past_cost = current.past_cost + edges[current.id][nbr.id]
                if tentative_past_cost < nbr.past_cost:
                    nbr.past_cost = tentative_past_cost
                    nbr.parent = current
                    nbr.est_cost = nbr.past_cost + nbr.h_cost
                    # heapify updates the open_list according to estimated cost
                    heapq.heapify(open)
    return [1]

if __name__ == "__main__":
    # nodes is a dictionary mapping node_id to node
    nodes = read_nodes("../results/nodes.csv")
    # edge is a dictionary mapping node to its connected nodes and edge cost. Just like a matrix
    edges = read_edges("../results/edges.csv", nodes)
    path = a_star(nodes, edges)
    write_path(path, "../results/path.csv")
