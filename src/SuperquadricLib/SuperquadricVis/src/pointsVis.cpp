#include "pointsVis.h"

using namespace std;
using namespace Eigen;
using namespace SuperqVis;

/**********************************************/
PointsVis::PointsVis(const vector<Vector3d> &points, const int point_size)
{
    if (points[0].size() == 3 || points.size() == 6)
    {
        vtk_points=vtkSmartPointer<vtkPoints>::New();
        for (auto point:points)
            vtk_points->InsertNextPoint(point[0],point[1],point[2]);

        vtk_polydata=vtkSmartPointer<vtkPolyData>::New();
        vtk_polydata->SetPoints(vtk_points);

        vtk_glyphFilter=vtkSmartPointer<vtkVertexGlyphFilter>::New();
        vtk_glyphFilter->SetInputData(vtk_polydata);
        vtk_glyphFilter->Update();

        vtk_mapper=vtkSmartPointer<vtkPolyDataMapper>::New();
        vtk_mapper->SetInputConnection(vtk_glyphFilter->GetOutputPort());

        vtk_actor=vtkSmartPointer<vtkActor>::New();
        vtk_actor->SetMapper(vtk_mapper);
        vtk_actor->GetProperty()->SetPointSize(point_size);
    }
}

/**********************************************/
void PointsVis::set_points(const vector<Vector3d> &points)
{
    vtk_points=vtkSmartPointer<vtkPoints>::New();
    for (auto point:points)
        vtk_points->InsertNextPoint(point[0],point[1],point[2]);

    vtk_polydata->SetPoints(vtk_points);
}

/**********************************************/
bool PointsVis::set_colors(const vector<vector<unsigned char>> &colors)
{
    if (colors.size()==vtk_points->GetNumberOfPoints())
    {
        vtk_colors=vtkSmartPointer<vtkUnsignedCharArray>::New();
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
