## How to use superquadric-lib
The files [`single-superq/main.cpp`](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/single-superq/main.cpp) and [`multiple-superq/main.cpp`](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/multiple-superq/main.cpp)
show examples of usage of `SuperquadricLibModel`, `SuperquadricLibGrasp` and `SuperquadricLibVis`.

### How to run the code
Before explaining the code, here is the command  to launch the executable (if installed):
```
$  Superquadric-Pipeline-Single path/to/point_cloud_file
```
```
$  Superquadric-Pipeline-Multiple path/to/point_cloud_file
```
An example of `point_cloud_file` for single superquadric modeling is provided [here](https://github.com/robotology/superquadric-lib/blob/master/misc/example-bottle).
An example of `point_cloud_file` for multiple superquadric modeling is provided [here](https://github.com/robotology/superquadric-lib/blob/master/misc/example-drill).


### Outcome example
This is the  outcome you should obtain:

<img src="https://github.com/robotology/superquadric-lib/blob/master/misc/example-bottle.png" width = "250"> <img src="https://github.com/robotology/superquadric-lib/blob/master/misc/example-drill.png" width = "250">


The visualizer shows:
- the object **point cloud**;
- the object point cloud **downsampled** (red dots, only for the single-superquadric example);
- the reconstructed **object models** made of a **single or multiple superquadrics**;
- the **grasping candidates** for the right and the left hand (for the sake of clarity, only for the right hand in the multi-superquadric example).

### How to write an executable
Here is a brief description of the main steps required to play with `superquadric-lib` tools.

1. Instantiate the following quantities:
```
   PointCloud point_cloud;                 // Object point cloud
   vector<Superquadric> supersq;           // Object Superquadric
   Visualizer vis;                         // VTK visualizer
   SuperqEstimatorApp estim;               // Superquadric Estimator
   GraspEstimatorApp grasp_estim;          // Grasping pose Estimator
   GraspResults grasp_res;                 // The results of the grasping computation
   deque<Vector3d> all_points;             // To read the object point cloud
```
2. Define all the parameters:

    | Set of params | Description | Link to the code (single superq) | Link to the code (multiple superq) |
    | ------------- | ------------- |------------- |------------- |
    | `iparams_superq` | Superquadric estimation | [single-superq/main.cpp](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/single-superq/main.cpp#L51) | [multiple-superq/main.cpp](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/multiple-superq/main.cpp#L49) |
     | `m_pars` | Multiple superquadric estimation |  | [multiple-superq/main.cpp](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/multiple-superq/main.cpp#L63) |
    |`iparams_grasp` | Grasping pose computation | [single-superq/main.cpp](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/single-superq/main.cpp#L65)  |  [multiple-superq/main.cpp](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/multiple-superq/main.cpp#L72)|
    | `params_grasp` | Grasping pose computation | [single-superq/main.cpp](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/single-superq/main.cpp#L127)|[multiple-superq/main.cpp](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/multiple-superq/main.cpp#L132)|

3. [Read the object point cloud](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/single-superq/main.cpp#L78)
and save them in `all_points`.
4. Estimate the superquadric:
```
  point_cloud.setPoints(all_points);                                         // Fill the point cloud    
  superqs = estim.computeSuperq(iparams_superq, point_cloud);                // Compute single 
  superqs = estim.computeMultipleSuperq(iparams_superq, m_pars, point_cloud);// or multiple superqs
```
5. Add the estimated superquadric(s) to the `params_grasp` used for computing the grasping candidates:
```
  params_grasp.object_superqs = superqs;
```
6. Compute the grasping candidate:
```
  grasp_res = grasp_estim.computeGraspPoses(iparams_grasp, params_grasp);
```
7. Visualize everything
```
   vis.addSuperq(superqs);                                   // Add superquadric to visualizer
   vis.addPoints(point_cloud, true);                         // Add points to visualizer
                                                              // (true/false to show downsampled points
                                                              //    used for superq estimation)
   vis.addPlane(params_grasp.pl(3));                         // Add plane for grasping
   vis.addPoses(grasp_res.grasp_poses);                      // Add poses for grasping
   vis.visualize();                                          // Visualize
```
