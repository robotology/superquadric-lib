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

#include <SuperquadricLibVis/poseVis.h>

using namespace std;
using namespace Eigen;
using namespace SuperqVis;


/**********************************************/
PoseVis::PoseVis()
{
    pose.setIdentity();
    pose_vtk_actor = vtkSmartPointer<vtkAxesActor>::New();
    pose_vtk_transform = vtkSmartPointer<vtkTransform>::New();
    pose_vtk_caption_actor = vtkSmartPointer<vtkCaptionActor2D>::New();
}

/**********************************************/
void PoseVis::setvtkTransform(const VectorXd &pose_vect)
{
    if (pose_vect.size() == 6 and pose_vect.norm() > 0.0)
    {
        pose.block<3,3>(0,0) = (AngleAxisd(pose_vect(3), Vector3d::UnitZ())*
                                AngleAxisd(pose_vect(4), Vector3d::UnitY())*
                                AngleAxisd(pose_vect(5), Vector3d::UnitZ())).toRotationMatrix();

        pose.block<3,1>(0,3) = pose_vect.head(3);

        vtkSmartPointer<vtkMatrix4x4> m_vtk = vtkSmartPointer<vtkMatrix4x4>::New();
        m_vtk->Zero();
        for (auto i = 0; i < pose.rows(); i++)
        {
            for(auto j = 0; j < pose.cols(); j++)
            {
                m_vtk->SetElement(i, j, pose(i, j));
            }
        }

        pose_vtk_transform->SetMatrix(m_vtk);
    }
}

/**********************************************/
void PoseVis::setvtkActorCaption(const string &caption, double &offset)
{
    pose_vtk_caption_actor->GetTextActor()->SetTextScaleModeToNone();
    pose_vtk_caption_actor->SetCaption(caption.c_str());
    pose_vtk_caption_actor->BorderOff();
    pose_vtk_caption_actor->LeaderOn();
    pose_vtk_caption_actor->GetCaptionTextProperty()->SetFontSize(20);
    pose_vtk_caption_actor->GetCaptionTextProperty()->FrameOff();
    pose_vtk_caption_actor->GetCaptionTextProperty()->ShadowOff();
    pose_vtk_caption_actor->GetCaptionTextProperty()->BoldOff();
    pose_vtk_caption_actor->GetCaptionTextProperty()->ItalicOff();
    pose_vtk_caption_actor->SetAttachmentPoint(pose(0,3), pose(1,3), pose(2,3)+offset);
}
