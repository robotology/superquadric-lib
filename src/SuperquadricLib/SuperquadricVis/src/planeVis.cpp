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
