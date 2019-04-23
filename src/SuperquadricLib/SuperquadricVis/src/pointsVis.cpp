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

#include <SuperquadricLibVis/pointsVis.h>

using namespace std;
using namespace Eigen;
using namespace SuperqVis;

/**********************************************/
PointsVis::PointsVis(const vector<Vector3d, aligned_allocator<Vector3d>> &points, const int &point_size)
{
    if (points[0].size() == 3 || points[0].size() == 6)
    {
        vtk_points = vtkSmartPointer<vtkPoints>::New();
        for (auto point:points)
            vtk_points->InsertNextPoint(point[0],point[1],point[2]);

        vtk_polydata = vtkSmartPointer<vtkPolyData>::New();
        vtk_polydata->SetPoints(vtk_points);

        vtk_glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vtk_glyphFilter->SetInputData(vtk_polydata);
        vtk_glyphFilter->Update();

        vtk_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_glyphFilter->GetOutputPort());

        vtk_actor = vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetPointSize(point_size);
    }
}

/**********************************************/
void PointsVis::set_points(const vector<Vector3d, aligned_allocator<Vector3d>> &points)
{
    vtk_points = vtkSmartPointer<vtkPoints>::New();
    for (auto point:points)
        vtk_points->InsertNextPoint(point[0],point[1],point[2]);

    vtk_polydata->SetPoints(vtk_points);
}

/**********************************************/
bool PointsVis::set_colors(const vector<vector<unsigned char>> &colors)
{
    if (colors.size() == vtk_points->GetNumberOfPoints())
    {
        vtk_colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
        vtk_colors->SetNumberOfComponents(3);
        for (auto color:colors)
            vtk_colors->InsertNextTypedTuple(color.data());

        vtk_polydata->GetPointData()->SetScalars(vtk_colors);
        return true;
    }
    else
        return false;
}

/**********************************************/
vtkSmartPointer<vtkPolyData> &PointsVis::get_polydata()
{
    return vtk_polydata;
}
