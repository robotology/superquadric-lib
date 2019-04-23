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
  
#include <SuperquadricLibVis/planeVis.h>

using namespace std;
using namespace SuperqVis;


/****************************************************************/
PlaneVis::PlaneVis(const double &z_height)
{
    plane_source = vtkSmartPointer<vtkPlaneSource>::New();
    plane_source->SetCenter(0.0, 0.0, z_height);
    plane_source->SetNormal(0.0, 0.0, 1.0);
    plane_source->Update();

    vtkPolyData* plane = plane_source->GetOutput();

    // Create a mapper and actor
    vtk_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtk_mapper->SetInputData(plane);

    vtk_actor = vtkSmartPointer<vtkActor>::New();
    vtk_actor->SetMapper(vtk_mapper);
}

/****************************************************************/
void PlaneVis::setHeight(const double &z_height)
{
    plane_source->SetCenter(0.0, 0.0, z_height);
    plane_source->Update();
}
