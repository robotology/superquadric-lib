#ifndef VTKRENDERER_H
#define VTKRENDERER_H

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

namespace SuperqVis {

class Visualizer
{
protected:

    int size_points;
    int height;
    bool closing;
    std::vector<double> backgroundColor;

    std::unique_ptr<SuperqVis::PointsVis> vtk_all_points;
    std::unique_ptr<SuperqVis::PointsVis> vtk_dwn_points;
    std::unique_ptr<SuperqVis::SuperquadricVis> vtk_superquadric;
    std::unique_ptr<SuperqVis::PlaneVis> vtk_plane;
    std::vector<std::vector<unsigned char>> all_colors;

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
    * @return true/false on success/failuer
    */
    void visualize();

    /**
    * Save a screenshot without visualization
    * @param name is the string that is add in the file name
    * @return true/false on success/failuer
    */
    void saveScreenshot(std::string &name, int &number);

    /** Start visualizer
    * @param s is a superquadric visualizer
    * @return true/false on success/failuer
    */
    void addSuperq(std::vector<SuperqModel::Superquadric> &superq);

    /** Start visualizer
    * @param p is a plane visualizer
    * @return true/false on success/failuer
    */
    void addPlane(double &z);

    /** Start visualizer
    * @param p is a points visualizer
    * @return true/false on success/failuer
    */
    void addPoints(SuperqModel::PointCloud &point_cloud);

    /** Start visualizer
    * @param p is a points visualizer
    * @return true/false on success/failuer
    */
    void addPoints(std::vector<Eigen::VectorXd> &all_points, std::vector<Eigen::VectorXd> &dwn_points, std::vector<std::vector<unsigned char>> &all_colors);

    /** Start visualizer
    * @param p is a pose visualizer
    * @return true/false on success/failuer
    */
    void addPoses(std::vector<Eigen::VectorXd> &poses);
};
}
#endif
