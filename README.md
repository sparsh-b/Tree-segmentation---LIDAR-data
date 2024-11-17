# Tree-segmentation---LiDAR-data
- This repository contains an algorithm to segment trees from LiDAR maps of urban outdoors - implemented in C++.
- The algorithm, as described below, leverages the geometrical shape of the trees to segment them.
- Neither Deep Learning nor any software library is used.

## Performance:
- The algorithm was tested on the [Oakland 3-D Point Cloud Dataset](https://www.cs.cmu.edu/~vmr/datasets/oakland_3d/cvpr09/doc/).
- A median accuracy of 89% was obtained. 

<!---
[//]: # ([This table](https://docs.google.com/spreadsheets/d/1yMoqAky3DIHIxovXaNnt9_2AXkzP6ENaPm4fmeurqHU/edit?usp=sharing) contains the performance of the segmentation algorithm on each map.)
-->

## Algorithm:
- The algorithm recognizes trees using their 3D geometric shape. It consists of 2 parts:
   - A. <b>Forming Clusters</b> - Group the points into clusters based on proximity & the range of cluster's z-coordinates.
   - B. <b>Filtering Clusters</b> - Filter out irrelevant clusters so that only those corresponding to the trees remain.

- ### A. Forming Clusters:
   1. A point is randomly chosen from the entire map & its neighbors are found. Then, neighbors to these 1st set of neighbors are found & this process is continued (until no neighbors are found).
   2. At each step of expansion:   
      a.  If the range of z-coordinates of the cluster doesn't vary much (it means the cluster is part of a horizontal object & since trees are vertical), the cluster is discarded. Then, the above 1st step is repeated.
      
      b.  Else, if the range of z-coordinates of the cluster vary enough but there is crowding in the bottom, it means that the cluster contains both a vertical object & a surrounding horizontal object (eg: tree's trunk + side walk around the trunk). These crowded points are removed so that, in the next step, the cluster expands only in the vertical direction & not in the horizontal direction.
      <p align="center">
         <img src="https://drive.google.com/uc?export=view&id=1AiuiaWgM8Ju43MGILoV95q0QFUNxwM_Z"/>
      </p>
      In the above picture, it can be seen that without ii-b part of the algo, cluster expanded (also) towards the side walk & included more points from the same. But with ii-b part of the algo in place, cluster expanded only towards the top of the tree.

   3. Output of 'Forming Clusters' part of the algorithm for the map 2_ac in the dataset (each cluster is randomly assigned a color):
      <p align="center">
        <img src="https://drive.google.com/uc?export=view&id=1kfwn9Jr4_T5JPQqRXTeU7uhtqz-dG2sj"/>
      </p>


- ### B. Filtering clusters:
   - Three levels of filtering are applied in this stage. 
   - #### First level:
      - Those clusters which correspond to the general size of a tree are kept & the rest are discarded (here, a size of 50 to 2400 points is used).
      <p align="center">
         <img src="https://drive.google.com/uc?export=view&id=1vlSHuNPj9c8eE3nWa0vYZGZk3FOqRXo_"/>
      </p>   

   - #### Second level:
      - This stage of filtering is based on Cluster's curvature.
      - ##### Calculation of Cluster's curvature:
         - Project all the points of the cluster onto XY-plane (A tree's trunk & foliage are curved along the Z-axis. Hence, projections of the cluster onto XY-plane is considered).
         - Find the median of radii of circles passing through all possible 3-point combinations from these projections.
         - In practice, this becomes a computational bottleneck for big clusters. So, only 10^6 randomly chosen combinations are used. This gives an estimate of the curvature of a cluster's top view.
      - Finally, those clusters with big radii of curvature (less curved) are discarded.
      
      - ##### Visualization:
         - For each cluster, a circle is drawn with the above obtained median radius & it is projected onto planes parallel to XY-plane so that a cylinder is formed around each cluster (referred to as median cylinder).

         - The below image shows clusters before & after 2nd level of filtering along with their median cylinders:
         <p align="center">
            <img src="https://drive.google.com/uc?export=view&id=1wYgkr7RQtMUMx_z_pK4ehdZN4qDfIkd1"/>
         </p>

   - #### Third level:
      - This level of filtering is based on the observation that:
         - 1) Planar surfaces (like facade, lawn grass, vehicle, etc) have large radius of curvature & more likely than not, they completely reside inside the median cylinder.
         - 2) Whereas, curved surfaces (like foliage/trunk/entire tree) have smaller radius of curvature. Consequently, clusters corresponding to curved surfaces tend to protrude out of their median cylinder.
      - This was able to remove: portions of facades (map 3_aj), current poles & vehicles (map 3_ak)
      <p align="center">
         <img src="https://drive.google.com/uc?export=view&id=1Y_PAWtws060hQF_EMXZgXN6h6vsXVn0H"/>
      </p>
      <p align="center">
         <img src="https://drive.google.com/uc?export=view&id=1vIvr21Y9VfoA_rBv_xfNVCKppwta4YLp"/>
      </p>


## Output
The final output of the algorithm for the maps: 2_al, 2_ac, 3_ak, 3_al of the dataset are shown below.
<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=1fjRXNIqlZPKc6fI-PF2-iWLIpmEr4m1L"/>
</p>
<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=1pa-OVX_5ZZXDgqWh_0JLNr--hLVr2NKZ"/>
</p>
<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=199m0XL0Z8tPX3zj3bzDcsC0txqLpSbmi"/>
</p>
<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=1mRZkTitOZ-niq2u6ZGSEINNYHeTDQ7Ma"/>
</p>


## Visualization:
The VRML Viewer [Qiew](http://www.qiew.org/) was used to visualize the wrl files.
