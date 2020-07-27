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
  
#include <SuperquadricLibVis/vis.h>

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
    const std::lock_guard<std::mutex> lock(mtx);

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

    //iren->GetRenderWindow()->SetWindowName("Superquadric visualizer");
    iren->Render();
}

/**********************************************/
vtkSmartPointer<vtkActor> &Object::get_actor()
{
    return vtk_actor;
}
