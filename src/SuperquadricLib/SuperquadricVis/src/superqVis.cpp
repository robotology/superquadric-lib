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

#include <SuperquadricLibVis/superqVis.h>

#include <cmath>

using namespace std;
using namespace Eigen;
using namespace SuperqVis;


/**********************************************/
SuperquadricVis::SuperquadricVis(const Vector12d &r, const string &type)
{
    double bx = r(7);
    double by = r(9);
    double bz = r(8);

    vtk_superquadric = vtkSmartPointer<vtkSuperquadric>::New();
    vtk_superquadric->ToroidalOff();
    vtk_superquadric->SetSize(1.0);
    vtk_superquadric->SetCenter(0.0, 0.0, 0.0);

    vtk_superquadric->SetScale(r(7),r(9),r(8));
    vtk_superquadric->SetPhiRoundness(r(10));
    vtk_superquadric->SetThetaRoundness(r(11));

    vtk_sample = vtkSmartPointer<vtkSampleFunction>::New();
    vtk_sample->SetSampleDimensions(50,50,50);
    vtk_sample->SetImplicitFunction(vtk_superquadric);
    vtk_sample->SetModelBounds(-bx,bx,-by,by,-bz,bz);

    // The isosurface is defined at 0.0 as specified in
    // https://github.com/Kitware/VTK/blob/master/Common/DataModel/vtkSuperquadric.cxx
    vtk_contours = vtkSmartPointer<vtkContourFilter>::New();
    vtk_contours->SetInputConnection(vtk_sample->GetOutputPort());
    vtk_contours->GenerateValues(1,0.0,0.0);

    vtk_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtk_mapper->SetInputConnection(vtk_contours->GetOutputPort());
    vtk_mapper->ScalarVisibilityOff();

    vtk_actor = vtkSmartPointer<vtkActor>::New();
    vtk_actor->SetMapper(vtk_mapper);
    if (type == "superq")
        vtk_actor->GetProperty()->SetColor(0.0,0.3,0.6);
    else
        vtk_actor->GetProperty()->SetColor(0.0,0.6,0.0);

    vtk_actor->GetProperty()->SetOpacity(0.25);

    vtk_transform = vtkSmartPointer<vtkTransform>::New();
    vtk_transform->Translate(r.segment(0,3).data());
    vtk_transform->RotateWXYZ((180.0/M_PI)*r(6),r.segment(3,3).data());
    vtk_transform->RotateX(-90.0);
    vtk_actor->SetUserTransform(vtk_transform);
}

/**********************************************/
void SuperquadricVis::set_parameters(const Vector12d &r)
{
    double bx = r(7);
    double by = r(9);
    double bz = r(8);

    vtk_superquadric->SetScale(r(7),r(9),r(8));
    vtk_superquadric->SetPhiRoundness(r(10));
    vtk_superquadric->SetThetaRoundness(r(11));

    vtk_sample->SetModelBounds(-bx,bx,-by,by,-bz,bz);

    vtk_transform->Identity();
    vtk_transform->Translate(r.segment(0,3).data());
    vtk_transform->RotateWXYZ((180.0/M_PI)*r(6),r.segment(3,3).data());
    vtk_transform->RotateX(-90.0);
}
