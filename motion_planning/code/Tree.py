import math
import heapq

class Tree:
    ''' Tree contains nodes and edges and functions to insert a'''
    '''new node in tree and find nearest node in tree'''
    '''nodes: nodes is a dictionary mapping node_id to the node'''
    '''       for faster access'''
    '''edges: edges is a dictionary mapping a node to its'''
    '''       connected nodes and the edge cost'''
    def __init__(self, node):
        node.id = 1
        self.nodes = {1:node}
        self.edges = {node.id:{}}
        self.size = 1
    def dist(self, n1, n2):
        ''' finds Eucledian distance between two nodes'''
        return math.sqrt((n1.x-n2.x)**2+(n1.y-n2.y)**2)
    def connect(self, x_near, x_new):
        '''connect x_new node to x_near and adds to tree if new'''
        '''assigns an id to the node and h_cost before inserting in tree'''
        x_new = self.insert(x_new)
        d = self.dist(x_near, x_new)
        self.edges[x_new.id][x_near.id] = d
        self.edges[x_near.id][x_new.id] = d
    def insert(self, node):
        '''inserts a new node in tree and returns the inserted node'''
        '''if a node exists, returns the existing node'''
        nn = self.nearest(node)
        if(self.dist(node,nn) > 0.001):
            self.size += 1
            node.id = self.size
            self.edges[node.id] = {}
            self.nodes[node.id] = node
            return node
        return nn
    def nearest(self, node):
        '''search for nearest node to node'''
        d = 999
        for n in self.nodes.values():
            d_new = self.dist(n,node)
            if(d_new<=d):
                x_near = n
                d = d_new
        return x_near
    def update_h_cost(self, goal):
        for n in self.nodes.values():
            n.h_cost = self.dist(n, goal)
    def k_nearest_neighbors(self, node, k=3):
        '''find k nearest neighbor nodes to the given node'''
        '''return list of nodes neighbor to node'''
        neighbors = []
        for n in self.nodes.values():
            if(node.id!=n.id):
                heapq.heappush(neighbors, (self.dist(n,node),n))
        return [heapq.heappop(neighbors)[1] for _ in range(k)]
