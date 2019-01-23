## How to use superquadric-lib
The file [`main.cpp`](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/main.cpp)
shows an example of usage of `SuperquadricLibModel`, `SuperquadricLibGrasp` and `SuperquadricLibVis`.

### How to run the code
Before explaining the code, here is the command  to launch the executable (if installed):
```
$  Superquadric-Pipeline path/to/point_cloud_file
```

An example of `point_cloud_file` is provided [here]().


### Outcome example
This is the  outcome you should obtain:


The visualizer shows:
- the object **point cloud**;
- the object point cloud **downsampled** (red dots);
- the reconstructed **superquadric**;
- the **grasping candidates** for the right and the left hand.

### How to write an executable
Here is a brief description of the main steps required to obtain such an outcome.

1. Instantiate the following quantities:
```
    PointCloud point_cloud;        // Object point cloud
    Superquadric superq;           // Object Superquadric
    Visualizer vis;                // VTK visualizer
    SuperqEstimatorApp estim;      // Superquadric Estimator
    GraspEstimatorApp grasp_estim; // Grasping pose Estimator
    GraspResults grasp_res;        // The results of the grasping computation
    deque<Vector3d> all_points;    // To read the object point cloud
```
2. Define all the parameters:
    -  `iparams_superq` for superquadric estimation
    (just copy and paste from [here](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/main.cpp#L52));
    -  `iparams_grasp` and `params_grasp` for grasping pose computation 
    (just copy and past from [here](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/main.cpp#L66) 
    and [here](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/main.cpp#L126)).
3. [Read the object point cloud](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/main.cpp#L79)
and save them in `all_points`.
4. Estimate the superquadric:
```
   point_cloud.setPoints(all_points);                        // Fill the point cloud class  
   superq=estim.computeSuperq(iparams_superq, point_cloud); // Compute superq
```
5. Add the estimated superquadric to the `params_grasp` used for computing the grasping candidates:
```
   params_grasp.object_superq = superq;
```
6. Compute the grasping candidate:
```
   grasp_res=grasp_estim.computeGraspPoses(iparams_grasp, params_grasp);
```
7. Visualize everything
```
    grasp_poses.push_back(grasp_res.grasp_pose);    // This allows you to visualize multiple poses
    superqs.push_back(superq);                      // This allows you to visualize multiple superqs
    vis.addSuperq(superqs);                         // Add superquadric to visualizer
    vis.addPoints(point_cloud, true);               // Add points to visualizer
                                                    // (true/false to show downsampled points
                                                    //    used for superq estimation)
    vis.addPlane(params_grasp.pl(3));               // Add plane for grasping
    vis.addPoses(grasp_poses);                      // Add poses for grasping
    vis.visualize();                                // Visualize
```
