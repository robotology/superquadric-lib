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

#include <SuperquadricLibModel/superquadricEstimator.h>
#include <SuperquadricLibVis/visRenderer.h>
#include <SuperquadricLibGrasp/graspComputation.h>

#include <cstdlib>
#include <cmath>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace SuperqModel;
using namespace SuperqVis;
using namespace SuperqGrasp;

int main(int argc, char* argv[])
{
    /*******************************************/
    // Check on parameters
    if (argc < 2)
    {
        cout<<endl;
        cout << "     Missing path file: " << endl;
        cout<< "     Provide the complete path of the file containing a partial point cloud " << endl<<endl;
        return 0;
    }
    /*******************************************/
    // PointCloud class
    PointCloud point_cloud;

    PointCloud points_hand; // Uncomment this to visualize points on hand ellipsoid in the final pose

    // Object Superquadric
    vector<Superquadric> superqs;

    // Superq Estimator
    SuperqEstimatorApp estim;

    // Grasping pose Estimator
    GraspEstimatorApp grasp_estim;

    // Grasp pose
    GraspResults grasp_res;
    vector<GraspPoses> grasp_poses;

    // VTK visualizer
    Visualizer vis;

    /*******************************************/
    // Read point cloud
    if (!point_cloud.readFromFile(argv[1]))
        return 0;

    /*******************************************/
    // Compute superq
    superqs = estim.computeSuperq(point_cloud);

      /*******************************************/
    // Compute grasp pose for right hand
    grasp_res = grasp_estim.computeGraspPoses(superqs);

    // Add poses for grasping
    vis.addPoses(grasp_res.grasp_poses);

    vis.highlightBestPose("right", "right", 0);

    /*******************************************/
    // Outcome visualization
    // Add superquadric to visualizer
    vis.addSuperq(superqs);

    // Add points to visualizer
    // (true/false to show downsample points used for superq estimation)
    vis.addPoints(point_cloud, false);

    // Add plane for grasping
    //vis.addPlane(grasp_estim.getPlaneHeight());

    // Add hands in final pose for grasping
    //vis.addSuperqHands(grasp_res.hand_superq); // Uncomment this to visualize hand ellipsoid in the final pose

    points_hand.setPoints(grasp_res.points_on[0]);  // Uncomment this to visualize points on the hand ellipsoid
    //vis.addPointsHands(points_hand);

    // Visualize
    vis.visualize();

    return 0;
}
