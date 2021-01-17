# class that defines nodes of a graph
class Nodes:
    def __init__(self, x=0, y=0, id=-1, h_cost=0, parent_node=0):
        self.id = id
        self.x = x
        self.y = y
        self.children = {}
        self.h_cost = h_cost
        self.parent = parent_node
        self.past_cost = 0.0 if(id==1) else float('inf')
        self.est_cost = self.h_cost + self.past_cost
    def __lt__(self, other):                     # comparison function on nodes
        return self.est_cost <= other.est_cost
