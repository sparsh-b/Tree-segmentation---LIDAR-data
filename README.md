# Tree-segmentation---LIDAR-data
C++ code to segment trees from LIDAR maps of urban outdoors

## Algorithm:
The algorithm uses the 3D geometric shape of the trees to find the same. It contains 2 main parts:
1) Group the 3-D points into clusters, based on proximity.
2) Filter out irrelevant clusters to finally contain only those clusters corresponding to trees.

### Forming Clusters
1) 
