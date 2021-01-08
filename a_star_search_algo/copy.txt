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
import csv
import heapq                                # minheap for open_list of nodes

# class that defines nodes of a graph
class Nodes:
    def __init__(self, id, h_cost, parent_node=0):
        self.id = id
        self.h_cost = h_cost
        self.parent = parent_node
        self.past_cost = 0.0 if(id==1) else float('inf')
        self.est_cost = self.h_cost + self.past_cost
    def __lt__(self, other):                     # comparison function on nodes
        return self.est_cost <= other.est_cost

# read edges from file
def fetch_edges(file_name, nodes):
    edges = {}
    for n in nodes.keys():
        edges[n] = {}
    with open(file_name, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        for row in reader:
            if(row[0][0].isnumeric()):
                edges[int(row[0])][int(row[1])] = float(row[2])
                edges[int(row[1])][int(row[0])] = float(row[2])
    return edges

# read nodes form file
def fetch_nodes(file_name):
    nodes = {}
    with open(file_name, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        for row in reader:
            if(row[0][0].isnumeric()):
                node = Nodes(int(row[0]), float(row[3]))
                nodes[node.id] = node
    return nodes

# write path to file
def write_path(path, file_name):
    with open(file_name, 'w+') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow(path)

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
    nodes = fetch_nodes("nodes.csv")
    # edge is a dictionary mapping node to its connected nodes and edge cost. Just like a matrix
    edges = fetch_edges("edges.csv", nodes)
    path = a_star(nodes, edges)
    write_path(path, "path.csv")
