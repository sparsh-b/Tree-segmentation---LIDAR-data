# Tree-segmentation---LIDAR-data
Algorithm & C++ code to segment trees from LIDAR maps of urban outdoors.

## Algorithm:
The algorithm uses a tree's 3D geometric shape to find the tree. It contains 2 main parts:
1) Group the 3-D points into clusters, based on proximity.
2) Filter out irrelevant clusters to finally contain only those clusters corresponding to trees.

### Forming Clusters
1) A point is randomly chosen from the map & all of its neighbors (points which fall within a certain threshold amount of distance from the initial point) are found. Then, neighbors to these 1st set of neighbors are found & so on. The cluster is expanded in this way.
2) At each step of expansion:
   1)  If the variation of points' z-coordinates is less, it means that the cluster is part of a horizontal object(eg: road) & hence it is discarded.
   2)  If the variation of points' z-coordinates is sufficient, but there is crowding in the bottom, it means that the cluster is a part of the base of a vertical object(eg: tree, building). These crowded points are removed from the cluster so that, in the next step, the culster will expand in the direction of the vertical object.![cluster_expansion](https://user-images.githubusercontent.com/45140884/122601840-89755180-d08f-11eb-978a-9bfac2e28501.jpg)



## Performance:
The algorithm was tested on the [CMU Oakland dataset](https://www.cs.cmu.edu/~vmr/datasets/oakland_3d/cvpr09/doc/) & it delivered a median accuracy of 89% across all the maps in the dataset. [This table](https://docs.google.com/spreadsheets/d/1yMoqAky3DIHIxovXaNnt9_2AXkzP6ENaPm4fmeurqHU/edit?usp=sharing) contains the performance of the segmentation algorithm on each map.
