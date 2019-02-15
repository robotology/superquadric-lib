/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Giulia Vezzani <giulia.vezzani@iit.it>
 */

#ifndef VTKRENDERER_H
#define VTKRENDERER_H

#include <memory>
#include <vtkSmartPointer.h>
#include <vtkCommand.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#include "vis.h"
#include "planeVis.h"
#include "pointsVis.h"
#include "poseVis.h"
#include "superqVis.h"
#include "superquadric.h"
#include "pointCloud.h"
#include "graspPoses.h"

namespace SuperqVis {

class Visualizer : public UpdateCommand
{
protected:

    int size_points;
    int height;
    int num_poses;
    bool closing;
    std::vector<double> backgroundColor;

    // For visualizing point clouds
    std::unique_ptr<SuperqVis::PointsVis> vtk_all_points;
    std::unique_ptr<SuperqVis::PointsVis> vtk_dwn_points;
    std::vector<std::vector<unsigned char>> all_colors;
    // For visualizing superquadrics
    std::vector<std::unique_ptr<SuperqVis::SuperquadricVis>> vtk_superquadrics;
    // For visualizing hand superquadrics
    std::vector<std::unique_ptr<SuperqVis::SuperquadricVis>> vtk_hand_superquadrics;
    std::unique_ptr<SuperqVis::PointsVis> vtk_hand_points;
    // For visualizing plane
    std::unique_ptr<SuperqVis::PlaneVis> vtk_plane;

    // For visualizing poses
    std::vector<std::shared_ptr<SuperqVis::PoseVis>> pose_candidates;
    std::vector<vtkSmartPointer<vtkAxesActor>> pose_actors;
    std::vector<vtkSmartPointer<vtkCaptionActor2D>> cap_actors;

    // Renderer members
    vtkSmartPointer<vtkRenderer> vtk_renderer;
    vtkSmartPointer<vtkRenderWindow> vtk_renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> vtk_renderWindowInteractor;
    vtkSmartPointer<vtkAxesActor> vtk_axes;
    vtkSmartPointer<vtkOrientationMarkerWidget> vtk_widget;
    vtkSmartPointer<vtkCamera> vtk_camera;
    vtkSmartPointer<vtkInteractorStyleSwitch> vtk_style;
    vtkSmartPointer<UpdateCommand> vtk_updateCallback;

public:
    /**
    * Constructor
    */
    Visualizer();

    /**
    * Destructor
    */
    ~Visualizer();

    /**
    * Start visualizer
    */
    void visualize();

    /**
    * Save a screenshot without visualization
    * @param name is the string that is add in the file name
    * @return true/false on success/failuer
    */
    void saveScreenshot(const std::string &name, const int &number);

    /** Start visualizer
    * @param s is a superquadric visualizer
    * @return true/false on success/failuer
    */
    void addSuperq(std::vector<SuperqModel::Superquadric> &superq);

    /** Start visualizer
    * @param s is a superquadric visualizer
    * @return true/false on success/failuer
    */
    void addSuperqHands(std::vector<SuperqModel::Superquadric> &s);

    /** Start visualizer
    * @param p is a plane visualizer
    * @return true/false on success/failuer
    */
    void addPlane(const double &z);

    /** Start visualizer
    * @param p is a points visualizer
    * @return true/false on success/failuer
    */
    void addPoints(SuperqModel::PointCloud point_cloud, const bool &show_downsample);

    /** Start visualizer
    * @param p is a points visualizer
    * @return true/false on success/failuer
    */
    void addPointsHands(SuperqModel::PointCloud point_cloud);

    /** Start visualizer
    * @param p is a pose visualizer
    * @return true/false on success/failuer
    */
    void addPoses(std::vector<SuperqGrasp::GraspPoses> &poses);

    /**********************************************/
    void addPoses(std::vector<SuperqGrasp::GraspPoses> &poses1, std::vector<SuperqGrasp::GraspPoses> &poses2);

    /**********************************************/
    void addPosesAux(const size_t start, std::vector<SuperqGrasp::GraspPoses> &poses);

    /**********************************************/
    void highlightBestPose(const std::string &hand, const std::string &both_or_not, const int &best);

    /**********************************************/
    void resetSuperq();

    /**********************************************/
    void resetPoses();

    /**********************************************/
    void resetPoints();

    /**********************************************/
    void setPosition(int i, int y);

    /**********************************************/
    void setSize(int w, int h);
};
}
#endif
