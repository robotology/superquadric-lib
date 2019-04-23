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

#ifndef PLANEVTK_H
#define PLANEVTK_H

#include <SuperquadricLibVis/vis.h>

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
