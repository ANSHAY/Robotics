from utils import *
from Tree import *

def PRM(obstacles, x_start, x_goal, num_samples, k_neighbors, X):
    '''PRM Algo to compute a graph of nodes and edges'''
    '''inputs:'''
    ''' x_start:        starting node'''
    ''' num_samples:    number of nodes to pick in environment'''
    ''' k_neighbors:    number of neighors to connect every node'''
    ''' obstacles:      all the obstacles that are in the environment'''
    ''' X:              bounds of the environment [x0, x1, y0, y1]'''
    ''' x_goal:         goal node'''
    '''output:'''
    ''' T:              Tree consisting of nodes and edges'''
    # initialize Tree
    T = Tree(x_start)
    R = Tree(x_start)
    i = 0
    while(i < num_samples):
        node = sampler(X)
        if (inCfree(node, obstacles)):
            T.insert(node)
            i += 1
    for node in T.nodes.values():
        neighbors = T.k_nearest_neighbors(node, k_neighbors)
        for n in neighbors:
            if(not is_collision(n, node, obstacles)):
                R.insert(node)
                R.connect(node,n)
    node = R.nearest(x_goal)
    R.connect(node, x_goal)
    R.update_h_cost(x_goal)
    return R
