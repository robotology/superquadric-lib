/******************************************************************************
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This program is free software; you can redistribute it and/or modify it under
* the terms of the GNU General Public License as published by the Free Software
* Foundation; either version 2 of the License, or (at your option) any later
* version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
* details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.                                                                     *
 ******************************************************************************/

 /**
  * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
  */

#ifndef SUPERQVTK_H
#define SUPERQVTK_H

#include <SuperquadricLibVis/vis.h>

#include <vtkSuperquadric.h>
#include <vtkSampleFunction.h>
#include <vtkContourFilter.h>
#include <vtkTransform.h>
#include <vtkProperty.h>

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 11, 1> Vector11d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;

namespace SuperqVis {

/**
* \class SuperqVis::Superquadric
* \headerfile superquadric.h <SuperquadricVis/include/superquadric.h>
*
* \brief A class from SuperqVis namespace.
*
* This class implements a VTK visualizer for a Superquadric with 11 parameters.
*/
class SuperquadricVis : public Object
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
    SuperquadricVis(const Vector12d &r, const std::string &type);

    /**
    * Destructory
    */
    ~SuperquadricVis() { }

    /**
     * Set superquadric parameters
     * @param s is a Eigen vector of 11 parameters
     */
    void set_parameters(const Vector12d &s);

};


}

#endif
