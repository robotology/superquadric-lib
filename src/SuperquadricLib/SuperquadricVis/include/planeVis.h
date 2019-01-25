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
class PlaneVis : public Object
{
protected:

    vtkSmartPointer<vtkPlaneSource> plane_source;
    vtkSmartPointer<vtkPolyDataMapper> vtk_mapper;

public:
    double z_height;
    /**
    * Constructor
    */
    PlaneVis(const double &z_height);

    /**
    * Destructory
    */
    ~PlaneVis() { }

    /**
     * Set the heigh of the plane
     * @param z_height is the height of the plane
     */
    void setHeight(const double &z_height);


};


}

#endif
