#ifndef SUPERQVTK_H
#define SUPERQVTK_H

#include "vis.h"

#include <vtkSuperquadric.h>
#include <vtkSampleFunction.h>
#include <vtkContourFilter.h>
#include <vtkTransform.h>
#include <vtkProperty.h>

#include <Eigen/Dense>

namespace SuperqVis {

/**
* \class SuperqVis::Superquadric
* \headerfile superquadric.h <SuperquadricVis/include/superquadric.h>
*
* \brief A class from SuperqVis namespace.
*
* This class implements a VTK visualizer for a Superquadric with 11 parameters.
*/
class Superquadric : public Object
{
protected:
    vtkSmartPointer<vtkSuperquadric> vtk_superquadric;
    vtkSmartPointer<vtkSampleFunction> vtk_sample;
    vtkSmartPointer<vtkContourFilter> vtk_contours;
    vtkSmartPointer<vtkTransform> vtk_transform;
public:
    /**
    * Constructor
    */
    Superquadric(const Eigen::VectorXd &s);

    /**
    * Destructory
    */
    ~Superquadric();

    /**
     * Set superquadric parameters
     * @param s is a Eigen vector of 11 parameters
     */
    void set_parameters(const Eigen::VectorXd &s);

};


}

#endif
