# Superquadric-lib python bindings
This repository contains the code to generate a python-based interface of SuperquadricLibModel, SuperquadricLibGrasp and SuperquadricLibVis.

### Build
To generate the bindings, it is necessary to:
1. Install [SWIG - 4.0.1](http://www.swig.org/) with:
```bash
$ curl https://netcologne.dl.sourceforge.net/project/swig/swig/swig-4.0.1/swig-4.0.1.tar.gz | tar xvz
$ cd swig-4.0.1 && ./configure --prefix=<path-to-install-directory>
$ make -j4
$ make install
```
2. Enable the flag `ENABLE_BINDINGS` at the time of building the whole library.

After the installation, you may need to add the `superquadriclib/bindings` to the `PYTHONPATH` in your `.bashrc`:
```bash
export PYTHONPATH=${PYTHONPATH}:${INSTALL_DIR}/lib/superquadriclib/bindings
```
### Interface

The python interface wraps most of the features of the library, but not all of them. This depends on the data types required by the specific method parameters / class members.

Specifically, it supports:

- **Eigen types**: the variables of type **Eigen*** are translated into `numpy arrays`, thanks to the file [eigen.i](eigen.i)
- **std::string**
- **std::vector**
- **std::deque**

It does not supports:
- **Ipopt types**: a wrapper for variables/classes of type **Ipopt*** is not provided
- **VTK types**: a wrapper for variables/classes of type **VTK*** is not provided

We suggest to inspect the file [superquadric_bindings.i](superquadric_bindings.i) to understand what is supported and how.

### Example of usage
The following code shows a basic usage of the library in python:

```python
import superquadric_bindings as sb
import numpy as np

sq_estimator = sb.SuperqEstimatorApp()
grasp_estimator = sb.GraspEstimatorApp()
visualizer = sb.Visualizer()
# here you may need to configure the parameters of sq_estimator, grasp_estimator, visualizer

# ---------------------------- #
# --- Generate point cloud --- #
# ---------------------------- #
pointcloud = sb.PointCloud()
points = sb.deque_Vector3d()
colors = sb.vector_vector_uchar()

# Suppose the pointcloud is a list of numpy arrays (floats):
pc = [np.array([0.1, 0.3, 0.4]), ...]

# Suppose the colors are a list of ints:
cl = [[0, 255, 255], ...]

for p in pc:
  points.push_back(p)

for c in cl:
  colors.push_back(c)

pointcloud.setPoints(points)
pointcloud.setColors(colors)

# ---------------------------- #
# --- Compute superquadric --- #
# ---------------------------- #
sq_vec = sb.vector_superquadric(sq_estimator.computeSuperq(self._pointcloud))

# -------------------------- #
# --- Compute grasp pose --- #
# -------------------------- #
grasp_res_hand = grasp_estimator.computeGraspPoses(sq_vec)

best_grasp_pose = grasp_res_hand.grasp_poses[grasp_res_hand.best_pose]

# ---------------- #
# --- visualize -- #
# ---------------- #
visualizer.addPoints(pointcloud, False)
visualizer.addSuperq(sq_vec)
visualizer.addPoses(grasp_res_hand.grasp_poses)
visualizer.highlightBestPose("right", "right", grasp_res_hand.best_pose)
```
