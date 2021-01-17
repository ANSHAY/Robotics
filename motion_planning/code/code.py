# Peer assignment 2 for Coursera Course
# Modern Robotics: Robot Motion Planning and Control
# for week 2.
# Implementation of Path planner algorithm which finds the
# nodes and edges and optimal path in a cluttered planar
# environment.
#
# inputs:
#     obstacles.csv: obstacles in the environment as circles
#                   with location [-0.5,0.5]x[-0.5,0.5] and radius
# outputs:
#     nodes: nodes in a graph written to nodes.csv
#     edges: edges in the graph written to edges.csv
#     path: writes the optimal path to path.csv
#
# Author: Anshay
# Date: 15/01/2021

# import libraries
from a_star import *
from utils import *
from RRT import *
from PRM import *
from read_write import *
from Tree import *
from Obstacles import *

if __name__=="__main__":
    ## Define file names for input data
    file_obstacles = "../results/obstacles.csv"
    file_path = "../results/path.csv"
    file_nodes = "../results/nodes.csv"
    file_edges = "../results/edges.csv"
    file_param = "../results/params.json"
    ## read obstacles from file
    obstacles = read_obstacles(file_obstacles)
    ## read params from file
    PARAM = read_params(file_param)
    ## Run RRT algo to create a Tree of nodes and edges
    x_start = Nodes(PARAM['x_start']['x'],PARAM['x_start']['y'], 1)
    x_goal = Nodes(PARAM['x_goal']['x'], PARAM['x_goal']['y'])
    print("\nFiles and parameters read...")
    print("\nRunning Algo...")
    if PARAM["algo"] == "PRM":
        T = PRM(obstacles, x_start, x_goal, PARAM['num_samples'], PARAM['k_neighbors'], PARAM['X'])
    else:
        T = RRT(obstacles, x_start, x_goal, PARAM['max_tree_size'], PARAM['X'])
    print("\nAlgo completed...")
    if T.size >= PARAM['max_tree_size']:
        print("\nMax tree size reached!!")
    nodes = T.nodes
    edges = T.edges
    print("\nWriting nodes and edges to file...")
    write_nodes(nodes, file_nodes)
    write_edges(edges, file_edges)
    ## Find the optimal path using A* and write it to file
    print("\nComputing Optimal Path")
    path = a_star(nodes, edges)
    write_path(path, file_path)
    print("\nSUCCESS")
