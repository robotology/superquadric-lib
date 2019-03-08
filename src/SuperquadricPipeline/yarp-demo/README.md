## Superquadric-Lib-Demo
This folder contains the source code for compiling and executing on the iCub robot, 
simulator and R1 robot a demo based on `superquadric-lib`. 

### Dependencies

- [`Yarp`](https://github.com/robotology/yarp)
- [`icub-main`](https://github.com/robotology/icub-main)
- [`icub-contrib-common`](https://github.com/robotology/icub-contrib-common)
- `SuperquadricLibModel`
- `SuperquadricLibVis`
- `SuperquadricLibGrasp`
- [iol](https://github.com/robotology/iol) (only for the online execution)
- [point-cloud-read](https://github.com/robotology/point-cloud-read) (only for the online execution)

### How to build
Instruction on how to build this demo are provided in the main [README.md](https://github.com/robotology/superquadric-lib/blob/master/README.md#how-to-build).

### How to run

#### Offline
1. Turn the simulator on;
2. Modules required:
    - `yarprobotinterface`
    - `iKinCartesianSolver` for one or both arms
3. Communicate with the module through `rpc` commands:
   - `set_single_superq <value>`:
   
      to use a single superquadric (`<value> = on`) or multiple superquadrics (`<value> = off`) to model the object
   - `from_off_file <file_name> <arm>`
      
      to compute the superquadric model and grasping pose of the object with point cloud 
      in `<file_name>` for just one arm `<arm>=right or left` or for both `<arm>=both`
   - `grasp`
   
      to simulate the grasp execution

#### Online
1. Turn the robot on
    (_**Note** so far it has been tested only on the iCub_)
2. Launch and connect [iol modules](https://github.com/robotology/iol)
3. Launch and connect the modules in [superquadric-lib.xml.template](https://github.com/robotology/superquadric-lib/blob/master/src/SuperquadricPipeline/yarp-demo/app/scripts/superquadric-lib-yarp-demo.xml.template)
4. Communicate with the module through `rpc` commands:
     - `set_single_superq <value>`
      
        to use a single superquadric (`<value> = on`) or multiple superquadrics to model the object
     - `compute_superq_and_grasp <object_name> <arm>` 
        
        to compute the superquadric model and grasping pose of 
      `<object_name>` for just one arm `<arm>=right or left` or for both `<arm>=both`
     - `grasp`
        
        to execute the grasp
     - `drop`
        
        to drop the object
