# Import KDTree and numpy
from sklearn.neighbors import KDTree
import networkx as nx
import numpy as np

# Generate some random 3-dimensional points
np.random.seed(0)
points = np.random.random((10, 3))  # 10 points in 3 dimensions
# Cast points into a KDTree data structure
tree = KDTree(points)              
# Extract indices of 3 closest points
# Note: need to cast search point as a list 
# and return 0th element only to get back list of indices
idxs = tree.query([points[0]], k=3, return_distance=False)[0]              
# indices of 3 closest neighbors (will vary due to random sample)
print(idxs) 

g = nx.Graph()
g.add_edge()