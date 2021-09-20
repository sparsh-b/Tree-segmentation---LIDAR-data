# Tree-segmentation---LIDAR-data
Algorithm & C++ code to segment trees from LIDAR maps of urban outdoors.

## Performance:
The algorithm was tested on the [CMU Oakland dataset](https://www.cs.cmu.edu/~vmr/datasets/oakland_3d/cvpr09/doc/) & it delivered a median accuracy of 89% across all the maps in the dataset. [This table](https://docs.google.com/spreadsheets/d/1yMoqAky3DIHIxovXaNnt9_2AXkzP6ENaPm4fmeurqHU/edit?usp=sharing) contains the performance of the segmentation algorithm on each map.

## Algorithm:
The algorithm uses a tree's 3D geometric shape to recognize a tree. It contains 2 main parts:
1) Group the 3D points into clusters, based on proximity & variation of z-coordinates.
2) Filter out irrelevant clusters so that only those corresponding to trees remain.

### 1) Forming Clusters:
1) A point is randomly chosen from the scene & all of its neighbors (points within a fixed threshold distance) are found. Then, neighbors to these 1st set of neighboring points are found. This process is continued, until no neighbors are found, to expand the cluster.
2) At each step of expansion:
   i)  If the variation of points' z-coordinates is less, it means that the cluster is part of a horizontal object(eg: road) & since trees are vertical, the cluster is discarded.
   ii)  If the variation of points' z-coordinates is sufficient, but there is crowding in the bottom, it means that the cluster contains both a vertical object & a horizontal object surrounding its base(eg: tree's trunk + pavement surrounding the trunk). These crowded points are removed from the cluster so that, in the next step, the culster expands in the vertical direction & not in horizontal direction.![cluster_expansion](https://user-images.githubusercontent.com/45140884/122601840-89755180-d08f-11eb-978a-9bfac2e28501.jpg)
     In the above picture, it can be seen that without 2-(ii) part of the algo, cluster expanded towards pavement & included more points from the same. But with 2-(ii) part of the algo, it expanded in the direction of the tree.

Output of 'Forming Clusters' part of the algo for the map 2_ac in the dataset(each cluster shown with a separate color):![all_clusters_vs_GT](https://user-images.githubusercontent.com/45140884/122628801-fdd2e380-d0d5-11eb-8bfb-bafa23984d64.jpg)



### 2) Filtering clusters:
Three levels of filtering are applied
1) First, those clusters which correspond to the general size of a tree(for the above reported performance, size = 50 to 2400 points) are kept & the rest are discarded.
![clusters_after_stage2_step1](https://user-images.githubusercontent.com/45140884/122628805-03c8c480-d0d6-11eb-9320-9a94a744f072.jpg)
2) Second stage of filtering is done by discarding those clusters with high radius of curvature.
   #### Cluster's curvature:
   1) Project all the points in the cluster onto XY-plane.
   2) Find the median of radii of circles passing through all possible 3-point combinations from these projections.
   3) This gives an estimate of the curvature of a cluster's top view.
   4) Visualization: A circle is drawn with the above obtained median radius & it is projected onto planes parallel to XY-plane but with different z-coordinate, so that a partially/completely enclosing cylinder is formed for each cluster.(referred to as median cylinder.)
      The below image shows clusters along with their median cylinders:![before_vs_after_stage2_step2](https://user-images.githubusercontent.com/45140884/122648330-77a2b580-d146-11eb-9465-e76e6777fe8f.jpg)


3) Third stage of filtering is based on the observation that:
   1) Planar surfaces (of a facade, grass, vehicle, etc) have large radius & hence, more likely than not, they completely reside inside the median cylinder.
   2) Whereas, curved surfaces (like foliage/trunk/entire tree) have smaller radius & hence, clusters corresponding to curved surfaces tend to protrude out of their median cylinder.
   This was able to remove: portions of facades(map 3_aj), current poles & vehicles(map 3_ak)![FINAL_3_aj_after_stage2_step3](https://user-images.githubusercontent.com/45140884/122637020-10671000-d10a-11eb-9503-e6698d8e4c11.jpg)
![FINAL_3_ak_before_vs_after_stage2_step3](https://user-images.githubusercontent.com/45140884/122637025-13fa9700-d10a-11eb-879a-ee2de46ea058.jpg)


## Output
The final output of the algorithm for the maps: 2_ac, 2_al, 3_ak, 3_al of the dataset are shown below.![2_ac_GTvsOP](https://user-images.githubusercontent.com/45140884/122637113-8cf9ee80-d10a-11eb-9c85-49d10268e99d.jpg)
![2_al_GTvsOP(1)](https://user-images.githubusercontent.com/45140884/122650574-614e2700-d151-11eb-9f6a-796ebb8f7211.jpg)
![3_ak_GTvsOP(1)](https://user-images.githubusercontent.com/45140884/122650579-63b08100-d151-11eb-9aaf-bc8b6f86627f.jpg)
![3_al_GTvsOP](https://user-images.githubusercontent.com/45140884/122637121-97b48380-d10a-11eb-857d-6b6b2689d166.jpg)


## Visualization:
The VRML Viewer [Qiew](http://www.qiew.org/) was used to visualize the wrl files.
