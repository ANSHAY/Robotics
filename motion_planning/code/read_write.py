import csv
import json
from Obstacles import *

# read obstacles form file
def read_obstacles(file_name):
    obstacles = []
    with open(file_name, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        for row in reader:
            if(row[0][0].isnumeric()):
                obstacle = Obstacles(float(row[0]), float(row[1]), float(row[2])/2)
                obstacles.append(obstacle)
    return obstacles

# write nodes to file
def write_nodes(nodes, file_name):
    with open(file_name, 'w+') as file:
        writer = csv.writer(file, delimiter=',')
        for n in nodes.values():
            writer.writerow([n.id,n.x,n.y,n.h_cost])

# write edges to file
def write_edges(edges, file_name):
    with open(file_name, 'w+') as file:
        writer = csv.writer(file, delimiter=',')
        for n1,dict in edges.items():
            for n2,c in dict.items():
                writer.writerow([n1,n2,c])

# read config parameters
def read_params(file_name):
    with open(file_name, 'r') as file:
        params = json.load(file)
    return params

# read edges from file
def read_edges(file_name, nodes):
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
def read_nodes(file_name):
    nodes = {}
    with open(file_name, 'r') as file:
        reader = csv.reader(file, delimiter=',')
        for row in reader:
            if(row[0][0].isnumeric()):
                node = Nodes(float(row[1]), float(row[2]), int(row[0]), float(row[3]))
                nodes[node.id] = node
    return nodes

# write path to file
def write_path(path, file_name):
    with open(file_name, 'w+') as file:
        writer = csv.writer(file, delimiter=',')
        writer.writerow(path)
