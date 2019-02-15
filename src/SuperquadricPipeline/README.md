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

:warning: **Note**: `superquadric-lib` does not provide any pre-processing for point clouds, such as filtering or outlier removals. It just downsamples the point cloud to estimate the superquadric. Therefore, please **provide already filtered point cloud to the library**. 



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
   vector<Superquadric> superqs;           // Object Superquadric
   Visualizer vis;                         // VTK visualizer
   SuperqEstimatorApp estim;               // Superquadric Estimator
   GraspEstimatorApp grasp_estim;          // Grasping pose Estimator
   GraspResults grasp_res;                 // The results of the grasping computation
   deque<Vector3d> all_points;             // To read the object point cloud
```
2. (Optional) Change algorithm parameters. Otherwise, the following default values are used:

    | Set of params | Description | Default values  | 
    | ------------- | ------------- |------------- |
    | `iparams_superq` | Superquadric estimation | [Link to the code](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricLib/SuperquadricModel/src/superquadricEstimator.cpp#L332) | 
     | `m_pars` | Multiple superquadric estimation |  [Link to the code](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricLib/SuperquadricModel/src/superquadricEstimator.cpp#L344) |
    |`iparams_grasp` | Grasping pose computation | [Link to the code](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricLib/SuperquadricGrasp/src/graspComputation.cpp#L875)  |  
    | `g_params` | Grasping pose computation | [Link to the code](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricLib/SuperquadricGrasp/src/graspComputation.cpp#L885)|
    
    **Note:** Setting the parameters can be done using the following functions of the `SuperquadricEstimatorApp` and `GraspEstimatorApp` classes:
    - `SetStringValue(<tag>, <value>)`, for strings;
    - `SetNumericValue(<tag>, <value>)`, for doubles;
    - `SetIntegerValue(<tag>, <value>)`, for ints;
    - `SetBoolValue(<tag>, <value>)`, for bools;
    - `setVector(<tag>, <value>)`, for Eigen vectors;
    - `setMatrix(<tag>, <value>)`, for Eigen matrices;
    
    Example:
    ```
    estim.SetStringValue("object_class", "box");
    grasp_estim.SetDoubleValue("tol", 1e-5);
    ```

3. [Read the object point cloud](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/single-superq/main.cpp#L78)
and save them in `all_points`.
4. Estimate the superquadric:
```
  point_cloud.setPoints(all_points);                           // Fill the point cloud    
  superqs = estim.computeSuperq(point_cloud);                  // Estimate single superquadric model
  superqs = estim.computeMultipleSuperq(point_cloud);          // or multi-superquadric model
```
5. Compute the grasping candidate:
```
  grasp_res = grasp_estim.computeGraspPoses(superqs);
```
6. Visualize everything
```
   vis.addSuperq(superqs);                                   // Add superquadric to visualizer
   vis.addPoints(point_cloud, true);                         // Add points to visualizer
                                                              // (true/false to show downsampled points
                                                              //    used for superq estimation)
   vis.addPlane(grasp_estim.getPlaneHeight();                         // Add plane for grasping
   vis.addPoses(grasp_res.grasp_poses);                      // Add poses for grasping
   vis.visualize();                                          // Visualize
```
