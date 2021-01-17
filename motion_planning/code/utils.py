import math
import random
from Nodes import *
from Obstacles import *

def dist(n1, n2):
    ''' finds Eucledian distance between two nodes'''
    return math.sqrt((n1.x-n2.x)**2+(n1.y-n2.y)**2)

def sampler(X):
    '''returns a sample from X'''
    x = random.uniform(X[0], X[1])
    y = random.uniform(X[2], X[3])
    x_samp = Nodes(x, y)
    return x_samp

def local_planner(x_near, x_samp, factor=0.5):
    '''returns a new node along the line from'''
    '''x_near to x_samp at distance d'''
    if factor < 0 or factor > 1:
        factor = 0.5
    x = x_near.x + (x_samp.x - x_near.x)*factor
    y = x_near.y + (x_samp.y - x_near.y)*factor
    x_new = Nodes(x, y)
    return x_new

def is_collision(A, B, obstacles):
    '''checks if any obstacle is in collision with line segment'''
    '''between nodes A and B'''
    eps = 0.001
    d_AB = dist(A,B)
    for C in obstacles:
        # D is a point on AB such that CD is normal to AB
        dot = ((C.x-A.x)*(B.x-A.x) + (C.y-A.y)*(B.y-A.y))/(d_AB*d_AB+eps)
        d_AD = abs(dot*d_AB)
        x = A.x + (B.x-A.x)*dot
        y = A.y + (B.y-A.y)*dot
        D = Obstacles(x, y)
        d_BD = dist(B, D)
        d_DC = dist(D, C)
        if d_DC > C.r:
            continue
        if (abs(d_AD+d_BD-d_AB) < eps):
            return True
        if (d_AD<=C.r or d_BD<=C.r):
            return True
    return False

def inCfree(node, obstacles):
    '''checks if a given node lies in Cfree or not'''
    for C in obstacles:
        if (dist(node, C) <= C.r+0.01):
            return False
    return True
