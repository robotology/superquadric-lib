#include "vis.h"

using namespace std;
using namespace SuperqVis;

/**********************************************/
//static UpdateCommand *New();
//{
//    return new UpdateCommand;
//}

/**********************************************/
void UpdateCommand::set_closing(const bool &closing)
{
    this->closing=&closing;
}

/**********************************************/
void UpdateCommand::Execute(vtkObject *caller, unsigned long vtkNotUsed(eventId),
             void *vtkNotUsed(callData))
{
    vtkRenderWindowInteractor* iren=static_cast<vtkRenderWindowInteractor*>(caller);
    if (closing!=nullptr)
    {
        if (*closing)
        {
            iren->GetRenderWindow()->Finalize();
            iren->TerminateApp();
            return;
        }
    }

    iren->GetRenderWindow()->SetWindowName("Superquadric visualizer");
    iren->Render();
}

/**********************************************/
vtkSmartPointer<vtkActor> &Object::get_actor()
{
    return vtk_actor;
}
