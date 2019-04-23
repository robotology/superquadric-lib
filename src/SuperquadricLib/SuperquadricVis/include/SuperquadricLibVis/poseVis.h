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

#ifndef POSEVIS_H
#define POSEVIS_H

#include <SuperquadricLibVis/vis.h>

#include <Eigen/Dense>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTransform.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

namespace SuperqVis {

class PoseVis : public Object
{
public:
    vtkSmartPointer<vtkAxesActor> pose_vtk_actor;
    vtkSmartPointer<vtkCaptionActor2D> pose_vtk_caption_actor;
    vtkSmartPointer<vtkTransform> pose_vtk_transform;
    Eigen::Matrix4d pose;

    /**
    * Constructor
    */
    PoseVis();

    /**
    * Destructory
    */
    ~PoseVis() { }

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
