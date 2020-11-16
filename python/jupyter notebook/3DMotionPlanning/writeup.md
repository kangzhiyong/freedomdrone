### Writeup / README
### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes
##### 1.in motion_planning.py plan_path:
        Load the 2.5D map in the colliders.csv file describing the environment，Discretize the environment into a grid representation.
        Define the start on the grid center and goal locations on the grid center from 10m. Perform a search using A*. Use a collinearity test to remove unnecessary waypoints.Return waypoints in local ECEF coordinates.
##### 2.in planning_utils.py:
        create_grid: Returns a grid representation of a 2D configuration space based on given obstacle data
        define a Action class that represented by a 3 element tuple
        valid_actions: Returns a list of valid actions given a grid and current node.
        a_star: Returns the motion planning from A* search algorithm


### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
in function plan_path:130: read the first line of the 'colliders.csv' file, get lat0 and lon0
and then use the self.set_home_position() method to set global home

#### 2. Set your current local position
in function plan_path:136: get current global position and then convert to current local position using global_to_local()

#### 3. Set grid start position from local position
in function plan_path:151: set start position to local position

#### 4. Set grid goal position from geodetic coords
in function plan_path:152 set geodetic coords of map to goal and convert to local position using global_to_local()

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
my implemention is based Probabilistic-Roadmap 
in file sampling.py define the obstacle Polygon class, 
in file sampling.py define function extract_polygons Generate 2.5D obstacle Polygon with safe_distance
in file sampling.py define the Sampler class, Generate random free area nodes
in file planning_utils.py function a_star_for_graph: Implement graph-based A * search algorithm
in file motion_planning.py in Class MotionPlanning define fuctions:
    can_connect:Determine whether there is a feasible route between two nodes
    create_graph: Create a feasible roadmap, use KDTree to find the k nearest neighbors, detect  collisions, and build a roadmap
    bresenham_check: Use bresenham algorithm to determine whether there is a feasible route between p1 and p2
    prune_path: Remove redundant nodes in the path
    find_closest_node: Find the node closest to a given point in the graph
#### 6. Cull waypoints 
in file motion_planning.py function prune_path: use Bresenham to prune the path of unnecessary waypoints
