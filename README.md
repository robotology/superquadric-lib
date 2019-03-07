# superquadric-library
The goal of this project is to provide  a `Yarp-free` **library** for **computing and visualizing** the **superquadric** representing an **object** and the relative **grasping candidates for a generic robot**.

This is a new design of the code included in the repositories:
- [superquadric-model](https://github.com/robotology/superquadric-model);
- [superquadric-grasp](https://github.com/robotology/superquadric-grasp);
- [superquadric-grasp-demo](https://github.com/robotology/superquadric-grasp-demo).
## Release 0.3
This release provides methods to:
- reconstruct the object model with **single or multiple superquadric**;
- compute  **grasping candidates** for any robot;
- **visualize** superquadrics, planes, point clouds and grasping poses.

### Relevant features:
- Only three dependencies: `Ipopt`,`Eigen3` and `VTK`;
- _Faster computation of grasping poses_ w.r.t the implementation of [superquadric-grasp](https://github.com/robotology/superquadric-grasp);
- _High-level interface_. 

### Dependencies
 - [`Ipopt`](https://projects.coin-or.org/Ipopt)
 - [`Eigen3`](https://bitbucket.org/eigen/eigen/)
 - [`VTK >= 8.1`](https://vtk.org/)

 
 ### How to build
 Here are the instructions to build and  install the library:
 
 ```
 $ git clone https://github.com/robotology/superquadric-lib.git
 $ cd superquadric-lib
 $ mkdir build && cd build
 $ cmake ..
 $ make
 $ [sudo] make install
 
 ```
 By default, this command will build and install:
 1. `SuperquadricLibModel`, that includes all the tools to play with superquadrics;
 2. `SuperquadricLibGrasp`, to compute the grasping poses for an object represented with a superquadric (it requires 1.);
 3. `SuperquadricLibVis`, to visualize everything (it requires 1. and 2.);
 4. `Superquadric-Pipeline-Single` and `Superquadric-Pipeline-Multiple`, executables providing an example on how to compute single/multiple superquadric object models and the grasping pose starting from the object point cloud and show the results using the visualizer. They require 1 - 3.
 
 **Optional:**
 
 Enabling the cmake flag `YARP_EXE` will compile also a `Yarp-based-Demo`, named `Superquadric-Lib-Demo`. This is a demo that uses the `superquadric-lib` to execute superquadric modeling and grasping on the iCub robot, iCub simulator and R1 robot. 
 
The following dependencies are required for this demo:

- [`Yarp`](https://github.com/robotology/yarp)
- [`icub-main`](https://github.com/robotology/icub-main)
- [`icub-contrib-common`](https://github.com/robotology/icub-contrib-common)

An overview of the `Superquadric-Lib-Demo` is provided [here](https://github.com/robotology/superquadric-lib/tree/master/src/SuperquadricPipeline/yarp-demo).
 
 
 ## How to link
 Once the library is installed, you can link it using `CMake` by writing the following line of code in your project `CMakeLists.txt`:
 
 ```
find_package(SuperquadricLib 0.1.0.0 EXACT REQUIRED)
target_link_libraries(<target> SuperquadricLib::SuperquadricLib<tag>)
 ```
 where `<tag>` can be:
 - `Model`
 - `Grasp`
 - `Vis`
 
 ## How to use the library
 
 An example on how to use the library in your code is provided [here](https://github.com/robotology/superquadric-lib/tree/master/src/SuperquadricPipeline).
