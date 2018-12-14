#ifndef PLANEVTK_H
#define PLANEVTK_H

#include "vis.h"

#include <vtkPlaneSource.h>


namespace SuperqVis {

/**
* \class SuperqVis::Plane
* \headerfile plane.h <SuperquadricVis/include/plane.h>
*
* \brief A class from SuperqVis namespace.
*
* This class implements a VTK visualizer for a plane.
*/
class Plane : public Object
{
protected:
    vtkSmartPointer<vtkPlaneSource> plane_source;
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;

public:
    /**
    * Constructor
    */
    Plane(double z_height);

    /**
    * Destructory
    */
    ~Plane();


};


}

#endif
