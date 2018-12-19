#ifndef VTKRENDERER_H
#define VTKRENDERER_H

#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

#include "vis.h"
#include "planeVis.h"
#include "pointsVis.h"
#include "poseVis.h"
#include "superqVis.h"

namespace SuperqVis {

class Visualizer
{
public:
    /**
    * Constructor
    */
    Visualizer();

    /**
    * Destructor
    */
    ~Visualizer();

    /**
    * Start visualizer
    * @return true/false on success/failuer
    */
    bool visualize();

    /**
    * Save a screenshot without visualization
    * @param name is the string that is add in the file name
    * @return true/false on success/failuer
    */
    bool saveScreenshot(string &name);

    /** Start visualizer
    * @param s is a superquadric visualizer
    * @return true/false on success/failuer
    */
    bool addSuperq(Superquadric &s);

    /** Start visualizer
    * @param p is a plane visualizer
    * @return true/false on success/failuer
    */
    bool addPlane(Plane &p);

    /** Start visualizer
    * @param p is a points visualizer
    * @return true/false on success/failuer
    */
    bool addPoints(Points &pp);

    /** Start visualizer
    * @param p is a pose visualizer
    * @return true/false on success/failuer
    */
    bool addPoses(Poses &pose);
};
}
#endif
