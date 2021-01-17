# Modern Robotics

### Robot Motion Planning and Control
### Assignment 2: Sampling based Planning

#### Code Structure--

| File | Description     |
| :------------- | :------------- |
| a_star.py      | A* algo which was part of previous assignment  |
| code.py        | Main file which calls upon other files. Run this file to run the assignment |
| code.txt       | All code in one place in a text file |
| Nodes.py       | Defines Nodes class for every node |
| Obstacles.py   | Defines Obstacles class for obstacles |
| Tree.py        | Defines Tree class for Tree containing nodes and edges and several functions defined below |
| utils.py       | Contains many utility functions defined below |
| read_write.py  | Contains all read and write functions in one place |
| PRM.py         | Contains PRM algorithm |
| RRT.py         | Contains RRT algorithm |

The `results` directory contains 6 files namely edges.csv, nodes.csv, obstacles.csv,path.csv, params.json, PRM_path.png   
`params.json` contains parameters that can be changed for different execution of code. It contains following parameters--
```
"algo":"PRM",                         Algo can be PRM or RRT
"x_start":{"x":-0.5, "y":-0.5},       starting node
"x_goal":{"x":0.5, "y":0.5},          goal node
"max_tree_size":500,                  maximum tree size for RRT
"X":[-0.5,0.5,-0.5,0.5],              Field parameters in the form [x0,x1,y0,y1] (a rectangle)
"num_samples": 100,                   number of samples for PRM algo
"k_neighbors": 5                      k nearest neighbors for PRM algo
```   
`PRM_path.png` is the screenshot of Coppeliasim with the path found in assignment

### Description of Code
I have implemented both PRM and RRT algo here which can be changed in parameters to select the algo of your choice.
The code is divided into several files as shown above.
The PRM and RRT algo are just as described in the book Modern Robotics. I'll explain the helper and utility functions here.
I have used the A* algo from the previous assignment to find the optimal path.  
There are 3 classes namely `Nodes`, `Obstacles` and `Tree`. Nodes contains location, cost, id and parent node for the nodes in a graph. Obstacles contains location and radius. Tree contains several functions--   
`dist()`: returns Euclidean distance between two nodes   
`connect()`: takes in 2 nodes. The first node is supposed to be in the tree. Inserts the second node into the tree and connects an edge between the two nodes.   
`insert()`: inserts a new node in tree and returns the inserted node if a node exists, returns the existing node   
`nearest()`: finds the node in the tree nearest to the given node   
`update_h_cost()`: updates the heuristic cost after building the complete tree. The h_cost is used in A* algo for path finding   
`k_nearest_neighbors()`: Returns `k` nearest neighbors to the given node in a tree. The given node must algo be in the tree.   

The utility functions are described below--   
`dist()`: returns Euclidean distance between two nodes   
`sampler()`: Samples a point in the given R2 space. Both x and y of the point are sampled with uniform random distribution. Returns a `Node` at the sampled point.   
`local_planner()`: Returns a new node between two given nodes along the line joining the nodes. It has a factor as a parameter that can be used to decide how far the new node is chosen along the line segment. A factor of `0` means the first point is chosen and `1` means second point is returned. Any value between 0 and 1 returns corresponding Node on line joining 2 given Nodes.   
`is_collision()`: It takes in 2 Nodes and a list of obstacles. Returns True if the line segment between 2 nodes is in collision with any of the obstacles. Returns False if there is no collision. To determine if there is a collision, it finds the point at which normal from `C` (center of obstacle) intersects line segment `AB` (two given nodes). It checks if any obstacle lies close to the line segment using geometry and returns false if there is no collision.   
`inCfree()`: Checks if given node lies in free C_space or not. Takes in a list of obstacles and a node. Computes distance of node from every obstacle. if distance is less than or equal to the radius of the node then returns false.

The helper functions for read and write are all packaged in read_write.py and their function is self explanatory.

### RRT Algo   
The algo is implemented as mentioned in the book. It samples a node in the space. Then finds a node in the tree nearest to the sampled node. Then uses `local_planner` to find a new node between sampled node and nearest node. The checks for collision with any obstacle. If there is no collision the it connects the new node to the nearest node in the tree. If this new node is in the goal, then it updates the h_cost and writes the nodes and edges to file.

### PRM Algo   
First it initializes a tree `T` and samples a given number of nodes in the given environment such that the nodes lie on C_free space i.e. no node lies on the obstacle, and inserts the nodes into the tree `T`. Then for every node in the tree `T`, it finds `k` neighbors. Then for every neighbors it checks if there is a direct path. If there is a direct path then connects the nodes in a different tree. Finally adds the goal node in the tree `R`, updates the cost and returns tree `R`. The second tree is built to remove unconnected nodes and keep the tree as one single connected graph.
