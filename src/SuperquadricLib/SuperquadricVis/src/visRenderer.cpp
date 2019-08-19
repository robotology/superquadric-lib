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

#include <SuperquadricLibVis/visRenderer.h>

using namespace std;
using namespace SuperqVis;
using namespace SuperqModel;
using namespace SuperqGrasp;
using namespace Eigen;

/**********************************************/
Visualizer::Visualizer()
{
    height = -0.2;
    size_points = 4;
    num_poses = 0;
    backgroundColor = {1.0,1.0,1.0};
    int max_superq_vis = 20;

    closing = false;

    vtk_renderer = vtkSmartPointer<vtkRenderer>::New();
    vtk_renderWindow = vtkSmartPointer<vtkRenderWindow>::New();

    vtk_renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
    vtk_renderWindowInteractor->SetRenderWindow(vtk_renderWindow);

    vtk_renderWindow->SetSize(600,600);
    vtk_renderWindow->AddRenderer(vtk_renderer);

    vtk_renderer->SetBackground(backgroundColor.data());

    vtk_axes = vtkSmartPointer<vtkAxesActor>::New();
    vtk_widget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    vtk_widget->SetOutlineColor(0.9300,0.5700,0.1300);
    vtk_widget->SetOrientationMarker(vtk_axes);
    vtk_widget->SetInteractor(vtk_renderWindowInteractor);
    vtk_widget->SetViewport(0.0,0.0,0.2,0.2);
    vtk_widget->SetEnabled(1);
    vtk_widget->InteractiveOn();

    vtk_camera = vtkSmartPointer<vtkCamera>::New();
    vtk_camera->SetPosition(0.1, 0.0, 0.5);
    vtk_camera->SetViewUp(0.0,0.0,1.0);
    vtk_renderer->SetActiveCamera(vtk_camera);

    vtk_style = vtkSmartPointer<vtkInteractorStyleSwitch>::New();
    vtk_style->SetCurrentStyleToTrackballCamera();
    vtk_renderWindowInteractor->SetInteractorStyle(vtk_style);

    vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> all_points;
    vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> dwn_points;

    vtk_all_points = unique_ptr<PointsVis>(new PointsVis(all_points,size_points));
    vtk_dwn_points = unique_ptr<PointsVis>(new PointsVis(dwn_points,size_points));
    vtk_hand_points = unique_ptr<PointsVis>(new PointsVis(all_points,size_points));
    vtk_renderer->AddActor(vtk_all_points->get_actor());
    vtk_renderer->AddActor(vtk_dwn_points->get_actor());
    vtk_renderer->AddActor(vtk_hand_points->get_actor());

    vtk_plane = unique_ptr<PlaneVis>(new PlaneVis(-0.18));
    vtk_renderer->AddActor(vtk_plane->get_actor());

    for (int i = 0; i < max_superq_vis; i++)
    {
        Vector12d r;
        r.setZero();
        vtk_superquadrics.push_back(unique_ptr<SuperquadricVis>(new SuperquadricVis(r, "superq")));
        vtk_renderer->AddActor(vtk_superquadrics[i]->get_actor());
    }

    for (int i = 0; i < max_superq_vis; i++)
    {
        Vector12d r;
        r.setZero();
        vtk_hand_superquadrics.push_back(unique_ptr<SuperquadricVis>(new SuperquadricVis(r, "hand")));
        vtk_renderer->AddActor(vtk_hand_superquadrics[i]->get_actor());
    }

    for (size_t idx = 0; idx < max_superq_vis * 2; idx++)
    {
        vtkSmartPointer<vtkAxesActor> ax_actor = vtkSmartPointer<vtkAxesActor>::New();
        vtkSmartPointer<vtkCaptionActor2D> cap_actor = vtkSmartPointer<vtkCaptionActor2D>::New();
        shared_ptr<PoseVis> candidate_pose = shared_ptr<PoseVis>(new PoseVis);
        ax_actor->VisibilityOff();
        cap_actor->VisibilityOff();
        pose_actors.push_back(ax_actor);
        cap_actors.push_back(cap_actor);
        vtk_renderer->AddActor(pose_actors[idx]);
        vtk_renderer->AddActor(cap_actors[idx]);
        pose_candidates.push_back(candidate_pose);
    }

    vtk_renderWindowInteractor->Initialize();
    vtk_renderWindowInteractor->CreateRepeatingTimer(10);

    vtk_updateCallback = vtkSmartPointer<UpdateCommand>::New();

    vtk_updateCallback->set_closing(closing);
    vtk_renderWindowInteractor->AddObserver(vtkCommand::TimerEvent,vtk_updateCallback);
}

/**********************************************/
Visualizer::~Visualizer()
{
    closing=true;
}

/**********************************************/
void Visualizer::setPosition(int x, int y)
{
    vtk_renderWindow->SetPosition(x,y);
}

/**********************************************/
void Visualizer::setSize(int w, int h)
{
    vtk_renderWindow->SetSize(w,h);
}

/**********************************************/
void Visualizer::addPoints(PointCloud point_cloud, const bool &show_downsample)
{
    vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> all_points = point_cloud.points_for_vis;
    vector<vector<unsigned char>> all_colors = point_cloud.colors;

    size_points = 4;
    mtx.lock();
    vtk_all_points->set_points(all_points);
    vtk_all_points->set_colors(all_colors);

    if (show_downsample)
    {
        size_points = 8;
        vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> dwn_points = point_cloud.points;
        vtk_dwn_points->set_points(dwn_points);
        vtk_dwn_points->get_actor()->GetProperty()->SetColor(1.0,0.0,0.0);
    }

    vector<double> bounds(6),centroid(3);
    vtk_all_points->get_polydata()->GetBounds(bounds.data());

    for (size_t i = 0; i < centroid.size(); i++)
        centroid[i] = 0.5*(bounds[i << 1] + bounds[(i << 1) + 1]);

    vtk_camera->SetPosition(centroid[0] + 0.5,centroid[1],centroid[2] + 0.4);
    vtk_camera->SetFocalPoint(centroid.data());
    vtk_camera->SetViewUp(0.0,0.0,1.0);
    mtx.unlock();
}

/**********************************************/
void Visualizer::addPointsHands(PointCloud point_cloud)
{
    vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> all_points = point_cloud.points_for_vis;

    mtx.lock();
    vtk_hand_points->set_points(all_points);
    mtx.unlock();
}

/**********************************************/
void Visualizer::addPlane(const double &z)
{
    mtx.lock();
    vtk_plane->setHeight(-z);
    mtx.unlock();
}

/**********************************************/
void Visualizer::addSuperq(vector<SuperqModel::Superquadric> &s)
{
    Vector12d r;

    mtx.lock();

    for (size_t i = 0; i < s.size(); i++)
    {
        r.segment(0,3) = s[i].getSuperqCenter();
        r.segment(3,4) = s[i].getSuperqAxisAngle();
        r.segment(7,3) = s[i].getSuperqDims();
        r.segment(10, 2) = s[i].getSuperqExps();

        vtk_superquadrics[i]->set_parameters(r);
    }

    Vector3d center(3);
    if (s.size() == 1)
      center = s[0].getSuperqCenter();
    else
    {
        center.setZero();
        for (auto sup:s)
        {
          center += sup.getSuperqCenter();
        }

        center /=s .size();
    }

    vtk_camera->SetPosition(center(0)+0.5,center(1),center(2)+0.4);
    vtk_camera->SetFocalPoint(center.data());
    vtk_camera->SetViewUp(0.0,0.0,1.0);

    mtx.unlock();
}

/**********************************************/
void Visualizer::addSuperqHands(vector<SuperqModel::Superquadric> &s)
{
    Vector12d r;

    mtx.lock();

    for (size_t i = 0; i < s.size(); i++)
    {
        r.segment(0,3) = s[i].getSuperqCenter();
        r.segment(3,4) = s[i].getSuperqAxisAngle();
        r.segment(7,3) = s[i].getSuperqDims();
        r.segment(10, 2) = s[i].getSuperqExps();

        vtk_hand_superquadrics[i]->set_parameters(r);
    }

    mtx.unlock();
}


/**********************************************/
void Visualizer::resetSuperq()
{
    mtx.lock();

    for (size_t i = 0; i < vtk_superquadrics.size(); i++)
    {
        Vector12d r;
        r.setZero();

        vtk_superquadrics[i]->set_parameters(r);
    }

    for (size_t i = 0; i < vtk_hand_superquadrics.size(); i++)
    {
        Vector12d r;
        r.setZero();

        vtk_hand_superquadrics[i]->set_parameters(r);
    }

    mtx.unlock();
}

/**********************************************/
void Visualizer::addPoses(vector<GraspPoses> &poses1)
{
    mtx.lock();

    num_poses = poses1.size();
    addPosesAux(0, poses1);

    mtx.unlock();
}

/**********************************************/
void Visualizer::addPoses(vector<GraspPoses> &poses1, vector<GraspPoses> &poses2)
{
    mtx.lock();

    num_poses = poses1.size() + poses2.size();
    addPosesAux(0, poses1);
    addPosesAux(poses1.size(), poses2);

    mtx.unlock();
}

/**********************************************/
void Visualizer::addPosesAux(const size_t start, vector<GraspPoses> &poses)
{
    Vector6d pose_vect;
    double offset=0.0;
    int i=0;

    for (size_t idx = 0; idx < poses.size(); idx++)
    {
        GraspPoses pose = poses[idx];
        if (pose.getGraspParams().norm() > 0.0)
        {
            pose_vect = pose.getGraspParams();
            pose_actors[idx + start]->VisibilityOff();
            cap_actors[idx + start]->VisibilityOff();

            pose_candidates[idx + start]->setvtkTransform(pose_vect);
            pose_candidates[idx + start]->pose_vtk_actor->SetUserTransform(pose_candidates[idx + start]->pose_vtk_transform);
            pose_actors[idx + start]->SetUserTransform(pose_candidates[idx + start]->pose_vtk_transform);

            pose_candidates[idx + start]->pose_vtk_actor->ShallowCopy(pose_actors[idx + start]);
            pose_actors[idx + start]->AxisLabelsOff();
            pose_actors[idx + start]->SetTotalLength(0.02, 0.02, 0.02);
            pose_actors[idx + start]->SetShaftTypeToCylinder();
            pose_actors[idx + start]->VisibilityOn();

            cap_actors[idx + start]->VisibilityOn();
            cap_actors[idx + start]->GetTextActor()->SetTextScaleModeToNone();

            stringstream ss;
            ss << pose.getHandName()<<" : "<<setprecision(3)<<pose.cost;

            offset += 0.01;

            pose_candidates[idx + start]->setvtkActorCaption(ss.str(), offset);
            cap_actors[idx + start]->SetCaption(pose_candidates[idx + start]->pose_vtk_caption_actor->GetCaption());
            cap_actors[idx + start]->BorderOff();
            cap_actors[idx + start]->LeaderOn();
            cap_actors[idx + start]->GetCaptionTextProperty()->SetFontSize(15);
            cap_actors[idx + start]->GetCaptionTextProperty()->FrameOff();
            cap_actors[idx + start]->GetCaptionTextProperty()->ShadowOff();

            cap_actors[idx + start]->GetCaptionTextProperty()->BoldOff();
            cap_actors[idx + start]->GetCaptionTextProperty()->ItalicOff();
            cap_actors[idx + start]->GetCaptionTextProperty()->SetColor(0.1, 0.1, 0.1);

            cap_actors[idx + start]->SetAttachmentPoint(pose_candidates[idx + start]->pose_vtk_caption_actor->GetAttachmentPoint());

            i++;
        }
    }
}

/**********************************************/
void Visualizer::highlightBestPose(const string &hand, const string &both_or_not, const int &best)
{
    int idx;
    if (both_or_not == "both")
    {
        if (hand == "right")
          idx = best;
        else
          idx = best + num_poses / 2.0;
    }
    else
      idx = best;

    cap_actors[idx]->GetCaptionTextProperty()->BoldOn();
    cap_actors[idx]->GetCaptionTextProperty()->SetColor(0., 0.35, 0.0);
    cap_actors[idx]->GetCaptionTextProperty()->SetFontSize(20);
}

/**********************************************/
void Visualizer::resetPoses()
{
    mtx.lock();

    for (size_t i = 0; i < pose_candidates.size(); i++)
    {
        pose_actors[i]->VisibilityOff();
        cap_actors[i]->VisibilityOff();
    }

    num_poses = 0;

    mtx.unlock();
}

/**********************************************/
void Visualizer::resetPoints()
{
    vector<Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> all_points;

    mtx.lock();
    vtk_all_points->set_points(all_points);
    vtk_dwn_points->set_points(all_points);
    vtk_hand_points->set_points(all_points);
    mtx.unlock();
}

/**********************************************/
void Visualizer::visualize()
{
    vtk_renderWindowInteractor->Start();
}

/**********************************************/
void Visualizer::saveScreenshot(const string &object, const int &number)
{
    vtk_renderWindowInteractor->Render();

    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
    vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(vtk_renderWindow);
    //windowToImageFilter->SetScale(1);
    windowToImageFilter->ReadFrontBufferOff();
    windowToImageFilter->Update();

    vtkSmartPointer<vtkPNGWriter> writer =
    vtkSmartPointer<vtkPNGWriter>::New();

    stringstream ss;
    ss << number;
    string number_str = ss.str();

    writer->SetFileName((object + "_screenshot_no_" + number_str + ".png").c_str());
    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    writer->Write();

    vtk_renderWindowInteractor->GetRenderWindow()->Finalize();
    vtk_renderWindowInteractor->TerminateApp();
}
