#ifndef POSEVIS_H
#define POSEVIS_H

#include "vis.h"

#include <Eigen/Dense>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTransform.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

namespace SuperqVis {

/**
* \class SuperqVis::GraspingPose
* \headerfile graspingPose.h <SuperquadricVis/include/graspingPose.h>
*
* \brief A class from SuperqVis namespace.
*
* This class implements a VTK class for visualizing a grasping pose.
*/
class Pose : public Object
{
protected:
    vtkSmartPointer<vtkAxesActor> pose_vtk_actor;
    vtkSmartPointer<vtkCaptionActor2D> pose_vtk_caption_actor;
    vtkSmartPointer<vtkTransform> pose_vtk_transform;
    Eigen::Matrix4d pose;
public:
    /**
    * Constructor
    */
    Pose();

    /**
    * Destructory
    */
    ~Pose();

    /**
     * Set the desired pose of the reference frame
     * @param pose_vect Eigen vector including the desired pose
     */
    void setvtkTransform(const Eigen::VectorXd &pose_vect);

    /**
     * Set the caption of the reference frame
     * @param caption of the desired caption
     * @param offset is the offset of the caption position w.r.t the center of the reference frame
     */
    void setvtkActorCaption(const std::string &caption, double &offset);


};


}

#endif
