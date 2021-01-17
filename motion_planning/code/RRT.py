from utils import *
from Tree import *
from Nodes import *

def RRT(obstacles, x_start, x_goal, max_tree_size, X):
    '''RRT Algo to compute a tree of nodes and edges'''
    '''inputs:'''
    ''' x_start:        starting node'''
    ''' max_tree_size:  maximum allowed size of tree'''
    ''' obstacles:      all the obstacles that are in the environment'''
    ''' X:              bounds of the environment [x0, x1, y0, y1]'''
    ''' x_goal:         goal node'''
    '''output:'''
    ''' T:              Tree consisting of nodes and edges'''
    T = Tree(x_start)
    eps = 0.01
    count = 0
    while(T.size <= max_tree_size):             ## loop until max size is reached
        if(count % (max_tree_size*0.1)==0):     ## use goal node as sample every 10% samples
            x_samp = x_goal
            count = 0
            print(".", end='')
        else:
            x_samp = sampler(X)
        x_near = T.nearest(x_samp)
        x_new = local_planner(x_near, x_samp, 0.5)
        if(not is_collision(x_new, x_near, obstacles)):
            T.connect(x_near, x_new)
            if (abs(x_new.x - x_goal.x) < eps and abs(x_new.y - x_goal.y) < eps):
                T.update_h_cost(x_goal)
                return T
        count+=1
    print("\nSolution not found in given Tree size limit. Try bigger tree\n")
    return T
